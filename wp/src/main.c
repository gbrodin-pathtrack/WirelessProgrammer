#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/byteorder.h>
#include <esb.h> // ESB wireless communication library.
#include <errno.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <string.h>
#include "serialMessages.h"
#include "UHFMessages.h"

LOG_MODULE_REGISTER(wireless_programmer, LOG_LEVEL_INF);

// Get node identifier for a /chosen node property.
#define UART_NODE DT_NODELABEL(uart20)

#define FEM_PDN_PIN 0

// Base station ID sent to the tag, which addresses its messages to it. Taken from the SoC's factory device ID so
// every programmer is different.
static uint32_t programmer_id;

// PAIR_ACK status.
#define PAIR_STATUS_HOST_PRESENT    0
#define PAIR_STATUS_NO_HOST         1

// Largest PicoFix radio packet (the CC1120 fast-mode limit), so pass-through chunks carry 90 bytes.
#define PICOFIX_MAX_PACKET          104
#define PASSTHROUGH_CHUNK_LEN       (PICOFIX_MAX_PACKET - sizeof(picofix32UHFHeader_t))

// Matches the tag's pass-through prep buffer (SCRATCH_UHF_PREP).
#define TAG_MESSAGE_MAX_LEN         1408

#define TAG_MESSAGE_TIMEOUT_MS      5000    // The tag pings roughly every 2 s, and waits 1 s after PAIR_ACK before its first message.
#define TAG_CHUNK_TIMEOUT_MS        1000
#define TAG_CONT_ACK_TIMEOUT_MS     1000
#define PC_REPLY_TIMEOUT_MS         800     // The tag only listens for 1 s after sending, so a later reply is useless.
#define PC_LINK_TIMEOUT_MS          2000
#define ESB_TX_TIMEOUT_MS           100     // Covers every retransmission.
#define IDLE_SERVICE_SLICE_MS       10      // How often blocking waits answer PAIR_INIT.

#define KEEP_ALVIE_INTERVAL_MS  2000
static volatile uint32_t lastMessageTime;

// Radio settings must match the other device
#define RF_CHANNEL              23      // 2423 MHz: between Wi-Fi channels 1 and 6, clear of BLE advertising channel 38 (2426 MHz).
#define RETRANSMIT_COUNT        3
#define RETRANSMIT_DELAY_US     5000

// ESB raises RX_RECEIVED before the ACK is on air, so stay in PRX this long before switching away to let every
// retransmission be ACKed.
#define ACK_GRACE_MS            ((RETRANSMIT_COUNT * RETRANSMIT_DELAY_US) / 1000 + 1)

// ESB address for pipe 0, the only pipe used. 0xE7 rather than 0xA_ bytes so the address doesn't continue the preamble's alternating 1010 pattern.
static const uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
static const uint8_t addr_prefix[1] = {0xE7};

// Get device reference from a devicetree node identifier.
static const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);

// FEM PDN pin initialisation.
static const struct device *gpio2_dev = DEVICE_DT_GET(DT_NODELABEL(gpio2));

static serialTXMessage_t serialTXBuffer;
static serialRXMessage_t serialRXBuffer;
K_MSGQ_DEFINE(serial_rx_queue, sizeof(uint8_t),
             SERIAL_RX_MAX_PAYLOAD + sizeof(serialPacketHeader_t) + 2, 1);
static atomic_t serial_rx_overflow;

// Received radio packets, drained from the ESB event handler. Deep enough to hold a tag's back-to-back chunks.
K_MSGQ_DEFINE(radio_rx_queue, sizeof(struct esb_payload), 16, 1);
static K_SEM_DEFINE(tx_done_sem, 0, 1);
static volatile bool tx_ok;

enum programmer_state {
    DISCONNECTED,       // No PC link.
    WAIT_FOR_TAG_PAIR,  // PC connected, waiting for a tag's PAIR_INIT.
    TAG_SESSION         // Passing messages between the paired tag and the PC.
};
static volatile enum programmer_state programmer_state = DISCONNECTED;

// Set once a tag has been sent PAIR_ACK, so its following packets are left queued for the session.
static bool session_pending;
static uint32_t session_tag_id;

// The tag's current pass-through message, assembled from its chunks.
static uint8_t tag_message[TAG_MESSAGE_MAX_LEN];

static void service_radio_idle(void);

/*
    PicoFix CRC: two running sums over everything after the tag byte and the CRC itself, stored in bytes 1 and 2.
*/
static void picofix_calculate_crc(uint8_t *message, uint16_t length)
{
    message[1] = 0;
    message[2] = 0;
    for (uint16_t i = 3; i < length; i++) {
        message[1] += message[i];
        message[2] += message[1];
    }
}

static bool picofix_crc_ok(const uint8_t *message, uint16_t length)
{
    uint8_t check_a = 0;
    uint8_t check_b = 0;

    for (uint16_t i = 3; i < length; i++) {
        check_a += message[i];
        check_b += check_a;
    }
    return check_a == message[1] && check_b == message[2];
}

/*
    Serial packet transmission/reception helpers.
*/
static void serial_transmit_raw(const uint8_t *buffer, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uart_poll_out(uart_dev, buffer[i]);
    }

    lastMessageTime = k_uptime_get_32();
}

static void serial_transmit_packet()
{
    uint8_t checkA = 0;
    uint8_t checkB = 0;

    // calc checksum over the six-byte header and payload
    for(uint16_t i = 0; i < serialTXBuffer.header.u16length + sizeof(serialPacketHeader_t); i++){
        checkA += serialTXBuffer.blob[i];
        checkB += checkA;
    }

    // add checksum to end of message
    serialTXBuffer.blob[serialTXBuffer.header.u16length + sizeof(serialPacketHeader_t)] = checkA;
    serialTXBuffer.blob[serialTXBuffer.header.u16length + sizeof(serialPacketHeader_t) + 1] = checkB;

    serial_transmit_raw(serialTXBuffer.blob, serialTXBuffer.header.u16length + sizeof(serialPacketHeader_t) + 2);
}

static void serial_rx_callback(const struct device *dev, void *user_data)
{
    uint8_t rx_byte;

    ARG_UNUSED(user_data);

    if (!uart_irq_update(dev) || !uart_irq_rx_ready(dev)) {
        return;
    }

    while (uart_fifo_read(dev, &rx_byte, 1) == 1) {
        if (k_msgq_put(&serial_rx_queue, &rx_byte, K_NO_WAIT) != 0) {
            atomic_set(&serial_rx_overflow, 1);
        }
    }
}

static bool serial_receive_packet(uint32_t timeout_ms)
{
    uint32_t start_ms = k_uptime_get_32();
    uint16_t msg_idx = 0;
    uint16_t expected_len = 0;
    uint8_t rx_byte;

    while (true) {
        uint32_t elapsed_ms = (uint32_t)(k_uptime_get_32() - start_ms);

        if (elapsed_ms >= timeout_ms) {
            return false;
        }

        uint32_t remaining_ms = timeout_ms - elapsed_ms;

        if (atomic_get(&serial_rx_overflow)) {
            atomic_set(&serial_rx_overflow, 0);
            k_msgq_purge(&serial_rx_queue);
            LOG_WRN("Serial RX queue overflow");
            return false;
        }

        // Wait in short slices so a tag's PAIR_INIT is still answered while the PC is slow to reply.
        if (k_msgq_get(&serial_rx_queue, &rx_byte, K_MSEC(MIN(remaining_ms, IDLE_SERVICE_SLICE_MS))) != 0) {
            service_radio_idle();
            continue;
        }

        if (msg_idx == 0) {
            if (rx_byte == (SERIAL_RX_HEADER & 0xFF)) {
                serialRXBuffer.blob[msg_idx++] = rx_byte;
            }
            continue;
        }

        if (msg_idx == 1) {
            if (rx_byte == ((SERIAL_RX_HEADER >> 8) & 0xFF)) {
                serialRXBuffer.blob[msg_idx++] = rx_byte;
            } else {
                msg_idx = (rx_byte == (SERIAL_RX_HEADER & 0xFF)) ? 1 : 0;
            }
            continue;
        }

        serialRXBuffer.blob[msg_idx++] = rx_byte;

        if (msg_idx == sizeof(serialPacketHeader_t)) {
            if (serialRXBuffer.header.u16length > SERIAL_RX_MAX_PAYLOAD) {
                msg_idx = 0;
                continue;
            }
            expected_len = serialRXBuffer.header.u16length + sizeof(serialPacketHeader_t) + 2;
        }

        if (expected_len != 0 && msg_idx == expected_len) {
            uint8_t check_a = 0;
            uint8_t check_b = 0;

            for (uint16_t i = 0;
                 i < expected_len - 2; i++) {
                check_a += serialRXBuffer.blob[i];
                check_b += check_a;
            }

            return check_a == serialRXBuffer.blob[expected_len - 2] &&
                   check_b == serialRXBuffer.blob[expected_len - 1];
        }
    }
}

/*
    FEM initialisation.
*/
static int fem_pdn_init(void)
{
    int err;

    // Ensure that GPIO device is ready to be utilised prior to configuration.
    if (!device_is_ready(gpio2_dev)) { // GPIO2.00 corresponds to FEM PDN. This is handled manually since Nordic doesn't support FEM controls split across multiple GPIO ports.
        LOG_ERR("GPIO2 not ready");
        return -ENODEV;
    }

    err = gpio_pin_configure(gpio2_dev, FEM_PDN_PIN, GPIO_OUTPUT_LOW); // COnfigure GPIO2.00 as an output in the low state.
    if (err) {
        LOG_ERR("Failed to configure FEM PDN: %d", err);
        return err;
    }

    // Bring nRF21540 out of power-down. MPSL will not control PDN because pdn_gpios is deliberately absent from FEM devicetree node.
    err = gpio_pin_set(gpio2_dev, FEM_PDN_PIN, 1); // Set the pin into its logically active state.
    if (err) {
        LOG_ERR("Failed to enable FEM PDN: %d", err);
        return err;
    }

    // Allow the FEM to leave power-down before any radio activity by setting a conservative settling time.
    k_sleep(K_MSEC(1));
    LOG_INF("nRF21540 PDN enabled on P2.00");
    return 0; // Successful return.
}

/*
    Programmer ID.
*/
static int programmer_id_init(void)
{
    uint8_t device_id[8];

    // nRF hwinfo returns the 64-bit FICR DEVICEID big-endian, so the last four bytes are DEVICEID[0].
    if (hwinfo_get_device_id(device_id, sizeof(device_id)) != sizeof(device_id)) {
        LOG_ERR("Failed to read device ID");
        return -EIO;
    }

    programmer_id = sys_get_be32(&device_id[4]);
    LOG_INF("Programmer ID 0x%08X", programmer_id);
    return 0;
}

/*
        ESB event handler.
*/
static void esb_eventhandler(const struct esb_evt *event)
{
    static struct esb_payload payload;

    switch (event->evt_id) {
    case ESB_EVENT_RX_RECEIVED:
        while (esb_read_rx_payload(&payload) == 0) {
            if (k_msgq_put(&radio_rx_queue, &payload, K_NO_WAIT) != 0) {
                LOG_WRN("Radio RX queue full, packet dropped");
            }
        }
        break;

    case ESB_EVENT_TX_SUCCESS:
        tx_ok = true;
        k_sem_give(&tx_done_sem);
        break;

    case ESB_EVENT_TX_FAILED:
        tx_ok = false;
        k_sem_give(&tx_done_sem);
        break;

    default:
        break;
    }
}

static int esb_radio_init(enum esb_mode mode)
{
    int err;
    // Create configuration with default parameters initially.
    struct esb_config config = ESB_DEFAULT_CONFIG;
    // Set specific configuration information (i.e. mode, event handling, auto-acknowledgement).
    config.mode = mode;
    config.event_handler = esb_eventhandler; // Manually link event handling function.
    config.selective_auto_ack = true; // Every packet transmitted requires acknowledgement
    config.bitrate = ESB_BITRATE_4MBPS;
    config.retransmit_count = RETRANSMIT_COUNT;
    config.retransmit_delay = RETRANSMIT_DELAY_US;

    err = esb_init(&config); // Initialise ESB module.
    if (err) {
        LOG_ERR("esb_init failed: %d", err);
        return err;
    }

    err = esb_set_base_address_0(base_addr_0); // Set base addresss of pipe 0.
    if (err) {
        LOG_ERR("base address 0 failed: %d", err);
        return err;
    }

    err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix)); // Single prefix, so only pipe 0 is enabled.
    if (err) {
        LOG_ERR("prefix configuration failed: %d", err);
        return err;
    }

    err = esb_set_rf_channel(RF_CHANNEL);
    if (err) {
        LOG_ERR("Failed to set RF channel: %d", err);
        return err;
    }

    if (mode == ESB_MODE_PRX) { // RX is interrupt-based so it needs to be manually enabled/disabled unlike TX.
        err = esb_start_rx();
        if (err) {
            LOG_ERR("Failed to start ESB RX: %d", err);
            return err;
        }
        LOG_DBG("ESB initialised as PRX");
    } else {
        LOG_DBG("ESB initialised as PTX"); // No additional steps since ESB transmissions can be called on-demand in automatic mode (which we are using).
    }
    return 0;
}

/*
        ESB mode switching.
*/
static int esb_switch_mode(enum esb_mode current_mode, enum esb_mode new_mode)
{
    int err;

    if (current_mode == ESB_MODE_PRX) {
        err = esb_stop_rx(); // RX needs to be stopped manually since it runs continuously in the background.
        if (err && err != -EALREADY) {
            LOG_ERR("Failed to stop ESB RX: %d", err);
            return err;
        }
    }

    esb_disable(); // Disable ESB module and flush FIFO.

    // Initialise the tag in the new ESB mode.
    err = esb_radio_init(new_mode);
    if (err) {
        LOG_ERR("Failed to initialise new ESB mode: %d", err);
        return err;
    }
    return 0;
}

/*
    Radio send/receive. The programmer sits in PRX and only switches to PTX to send, so the tag's ACKs stay empty.
*/
static bool radio_send(picofix32UHFTagRX_t *message)
{
    struct esb_payload payload = {
        .pipe = 0,
        .noack = false,
    };
    uint16_t length = sizeof(picofix32UHFHeader_t) + message->header.u16length;
    bool sent = false;

    picofix_calculate_crc((uint8_t *)message, length);
    payload.length = length;
    memcpy(payload.data, message, length);

    // Let the ACK for the packet that prompted this (and any retransmissions) go out before leaving PRX.
    k_sleep(K_MSEC(ACK_GRACE_MS));

    if (esb_switch_mode(ESB_MODE_PRX, ESB_MODE_PTX) == 0) {
        k_sem_reset(&tx_done_sem);
        if (esb_write_payload(&payload) == 0 &&
            k_sem_take(&tx_done_sem, K_MSEC(ESB_TX_TIMEOUT_MS)) == 0) {
            sent = tx_ok;
        }
    }

    if (esb_switch_mode(ESB_MODE_PTX, ESB_MODE_PRX)) {
        LOG_ERR("Failed to return to PRX");
    }
    return sent;
}

// Returns the packet as a PicoFix tag message if it is well formed, otherwise NULL.
static const picofix32UHFTagTX_t *tag_message_from_packet(const struct esb_payload *packet)
{
    const picofix32UHFTagTX_t *message = (const picofix32UHFTagTX_t *)packet->data;

    if (packet->length < sizeof(picofix32UHFHeader_t) ||
        message->header.u8picofixTag != UHF_PICOFIX_32_TAG ||
        packet->length != sizeof(picofix32UHFHeader_t) + message->header.u16length ||
        !picofix_crc_ok(packet->data, packet->length)) {
        return NULL;
    }
    return message;
}

// Waits for a message from the session tag, ignoring anything else. Returns NULL on timeout.
static const picofix32UHFTagTX_t *receive_from_tag(struct esb_payload *packet, uint32_t timeout_ms)
{
    int64_t deadline = k_uptime_get() + timeout_ms;

    while (true) {
        int64_t remaining_ms = deadline - k_uptime_get();

        if (remaining_ms <= 0 || k_msgq_get(&radio_rx_queue, packet, K_MSEC(remaining_ms)) != 0) {
            return NULL;
        }

        const picofix32UHFTagTX_t *message = tag_message_from_packet(packet);

        if (message != NULL &&
            message->header.u32senderID == session_tag_id &&
            message->header.u32receiverID == programmer_id) {
            return message;
        }
        LOG_WRN("Ignoring packet that isn't from the session tag");
    }
}

/*
    Pairing. The tag only tries once per boot and listens for 200 ms, so PAIR_INIT is answered from the main loop and
    from every blocking serial wait.
*/
static bool send_pair_ack(uint32_t tag_id, uint8_t status)
{
    picofix32UHFTagRX_t ack = {0};

    ack.header.u8picofixTag = UHF_PICOFIX_32_BS;
    ack.header.u8messageType = PICOFIX_V3_CONFIG_ACK;
    ack.header.u32senderID = programmer_id;
    ack.header.u32receiverID = tag_id;
    ack.header.u16length = sizeof(picofix32UHF_pairAck_t);
    ack.message.pairAck.u8status = status;

    if (!radio_send(&ack)) {
        LOG_WRN("Tag 0x%08X didn't ACK PAIR_ACK", tag_id);
        return false;
    }
    return true;
}

static void service_radio_idle(void)
{
    struct esb_payload packet;

    if (programmer_state == TAG_SESSION || session_pending) {
        return;
    }

    while (k_msgq_get(&radio_rx_queue, &packet, K_NO_WAIT) == 0) {
        const picofix32UHFTagTX_t *message = tag_message_from_packet(&packet);

        if (message == NULL ||
            message->header.u8messageType != PICOFIX_V3_CONFIG_INIT ||
            packet.length != PICOFIX_32_MESSAGE_SIZE_PAIR_INIT) {
            continue; // Only PAIR_INIT needs an answer outside a session.
        }

        uint32_t tag_id = message->header.u32senderID;

        if (programmer_state != WAIT_FOR_TAG_PAIR) {
            LOG_INF("Tag 0x%08X refused, no PC connected", tag_id);
            send_pair_ack(tag_id, PAIR_STATUS_NO_HOST);
            continue;
        }

        LOG_INF("Tag 0x%08X pairing", tag_id);
        if (send_pair_ack(tag_id, PAIR_STATUS_HOST_PRESENT)) {
            session_tag_id = tag_id;
            session_pending = true;
            return; // Leave the tag's following packets queued for the session.
        }
    }
}

// Sleeps while still answering PAIR_INIT.
static void idle_wait(uint32_t ms)
{
    int64_t end = k_uptime_get() + ms;

    while (k_uptime_get() < end) {
        service_radio_idle();
        k_sleep(K_MSEC(IDLE_SERVICE_SLICE_MS));
    }
}

/*
    Tag session.
*/

// Assembles the tag's next message: any TAG_PASSTHROUGH_CONT chunks (only ESB-ACKed), then TAG_PASSTHROUGH.
// Returns its length, or -1 if the tag goes quiet or sends something else.
static int receive_tag_passthrough(void)
{
    struct esb_payload packet;
    uint16_t length = 0;
    uint32_t timeout_ms = TAG_MESSAGE_TIMEOUT_MS;

    while (true) {
        const picofix32UHFTagTX_t *message = receive_from_tag(&packet, timeout_ms);

        if (message == NULL) {
            LOG_WRN("Tag message timed out");
            return -1;
        }

        if (message->header.u8messageType != PICOFIX_V3_TAG_PASSTHROUGH &&
            message->header.u8messageType != PICOFIX_V3_TAG_PASSTHROUGH_CONT) {
            LOG_WRN("Unexpected tag message type 0x%02X", message->header.u8messageType);
            return -1;
        }

        if (length + message->header.u16length > sizeof(tag_message)) {
            LOG_WRN("Tag message too long");
            return -1;
        }

        memcpy(&tag_message[length], message->message.blob, message->header.u16length);
        length += message->header.u16length;

        if (message->header.u8messageType == PICOFIX_V3_TAG_PASSTHROUGH) {
            return length;
        }
        timeout_ms = TAG_CHUNK_TIMEOUT_MS;
    }
}

// Sends a PC message to the tag as BASE_PASSTHROUGH_CONT chunks, each answered by the tag's BASE_CONT_ACK, then a
// final BASE_PASSTHROUGH.
static bool send_tag_passthrough(const uint8_t *data, uint16_t length)
{
    picofix32UHFTagRX_t message = {0};
    struct esb_payload packet;

    message.header.u8picofixTag = UHF_PICOFIX_32_BS;
    message.header.u32senderID = programmer_id;
    message.header.u32receiverID = session_tag_id;

    while (length > PASSTHROUGH_CHUNK_LEN) {
        message.header.u8messageType = PICOFIX_V3_BASE_PASSTHROUGH_CONT;
        message.header.u16length = PASSTHROUGH_CHUNK_LEN;
        memcpy(message.message.blob, data, PASSTHROUGH_CHUNK_LEN);

        if (!radio_send(&message)) {
            return false;
        }

        const picofix32UHFTagTX_t *ack = receive_from_tag(&packet, TAG_CONT_ACK_TIMEOUT_MS);

        if (ack == NULL || ack->header.u8messageType != PICOFIX_V3_BASE_CONT_ACK) {
            LOG_WRN("No BASE_CONT_ACK from tag");
            return false;
        }

        data += PASSTHROUGH_CHUNK_LEN;
        length -= PASSTHROUGH_CHUNK_LEN;
    }

    message.header.u8messageType = PICOFIX_V3_BASE_PASSTHROUGH;
    message.header.u16length = length;
    memcpy(message.message.blob, data, length);
    return radio_send(&message);
}

// Forwards each tag message to the PC and the PC's reply to the tag, until the PC sends PASS_THROUGH_DISCONNECT or
// either side stops responding. Leaves programmer_state set for what comes next.
static void run_tag_session(void)
{
    LOG_INF("Tag 0x%08X session started", session_tag_id);

    while (true) {
        int length = receive_tag_passthrough();

        if (length < 0) {
            programmer_state = WAIT_FOR_TAG_PAIR;
            break;
        }

        // The tag builds its pass-through messages as complete PC frames, so they're forwarded unchanged.
        serial_transmit_raw(tag_message, length);

        if (!serial_receive_packet(PC_REPLY_TIMEOUT_MS)) {
            LOG_WRN("No reply from PC");
            programmer_state = DISCONNECTED;
            break;
        }

        uint16_t type = serialRXBuffer.header.u16messageType;

        if (type != SERIAL_RX_PASS_THROUGH && type != SERIAL_RX_PASS_THROUGH_DISCONNECT) {
            LOG_WRN("Unexpected PC message 0x%04X during tag session", type);
            programmer_state = WAIT_FOR_TAG_PAIR;
            break;
        }

        if (!send_tag_passthrough(serialRXBuffer.passThrough.payload, serialRXBuffer.header.u16length)) {
            LOG_WRN("Tag stopped responding");
            programmer_state = WAIT_FOR_TAG_PAIR;
            break;
        }

        // The forwarded message tells the tag to disconnect, so the session ends without further messages.
        if (type == SERIAL_RX_PASS_THROUGH_DISCONNECT) {
            programmer_state = WAIT_FOR_TAG_PAIR;
            break;
        }
    }

    LOG_INF("Tag 0x%08X session ended", session_tag_id);
    session_pending = false;
    k_msgq_purge(&radio_rx_queue);
}

/*
    Main.
*/
int main(void)
{
    serialTXBuffer.header.u16header = SERIAL_TX_HEADER;

    int err;

    LOG_INF("Wireless programmer starting");

    // Confirm that serial peripheral is ready
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
        return 0; // Exit if there is an error with the device.
    }

    err = uart_irq_callback_user_data_set(uart_dev, serial_rx_callback, NULL);
    if (err) {
        LOG_ERR("Failed to install UART RX callback: %d", err);
        return 0;
    }
    uart_irq_rx_enable(uart_dev);

    err = programmer_id_init();
    if (err) {
        return 0;
    }

    // Enable the FEM by manually activating the PDN pin.
    err = fem_pdn_init();
    if (err) {
        LOG_ERR("FEM PDN initialisation failed: %d", err);
        return 0;
    }

    err = esb_radio_init(ESB_MODE_PRX); // Initialise ESB module.
    if (err) {
        LOG_ERR("ESB initialisation failed: %d", err);
        return 0;
    }

    LOG_INF("Wireless programmer waiting for PC");

    // Keep looping over this logic.
    while (1){
        switch (programmer_state) {
        case DISCONNECTED:
            session_pending = false;
            idle_wait(1000); //attempt connection only every second
            serialTXBuffer.header.u16messageType = SERIAL_TX_PAIR_REQUEST;
            serialTXBuffer.header.u16length = SERIAL_TX_PAIR_REQUEST_LEN;
            serialTXBuffer.pairRequest.u8devType = 1;

            serial_transmit_packet();

            if(!serial_receive_packet(PC_LINK_TIMEOUT_MS) ||
                serialRXBuffer.header.u16messageType != SERIAL_RX_PAIR_REQUEST_ACK){
                break;
            }

            serialTXBuffer.header.u16messageType = SERIAL_TX_FW_INFO;
            serialTXBuffer.header.u16length = SERIAL_TX_FW_INFO_LEN;
            serialTXBuffer.fwInfo.u32gitRev = 0x12345678; //TODO: Add git revision with gitrev.h generated by prebuild
            serialTXBuffer.fwInfo.u8protocolVer = 0;

            serial_transmit_packet();

            if(!serial_receive_packet(PC_LINK_TIMEOUT_MS) ||
                serialRXBuffer.header.u16messageType != SERIAL_RX_FW_INFO_ACK ||
                serialRXBuffer.fwInfoAck.u8status != 0){
                break;
            }
            LOG_INF("PC connected, waiting for tag");
            programmer_state = WAIT_FOR_TAG_PAIR;
            break;

        case WAIT_FOR_TAG_PAIR:
            service_radio_idle();
            if (session_pending) {
                programmer_state = TAG_SESSION;
                break;
            }

            if(k_uptime_get_32() - lastMessageTime > KEEP_ALVIE_INTERVAL_MS){
                serialTXBuffer.header.u16messageType = SERIAL_TX_KEEP_ALIVE;
                serialTXBuffer.header.u16length = SERIAL_TX_KEEP_ALIVE_LEN;

                serial_transmit_packet();

                if(!serial_receive_packet(PC_LINK_TIMEOUT_MS) || serialRXBuffer.header.u16messageType != SERIAL_RX_KEEP_ALIVE_ACK){
                    programmer_state = DISCONNECTED;
                    break;
                }
                LOG_DBG("KEEP_ALIVE sent");
            }
            break;

        case TAG_SESSION:
            run_tag_session();
            break;

        default:
            break;
        }
        k_sleep(K_MSEC(10)); // Small break to avoid constantly polling ESB.
    }
    return 0;
}
