#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <esb.h>
#include <string.h>

LOG_MODULE_REGISTER(esb_tx, LOG_LEVEL_INF); // Enable logging over the connected COM port.

// Packet constraint information (value based on Nordic ESB limit)
#define MAX_PAYLOAD_LEN 252

/* ESB packet structure:
        Byte 0 = Header
        Byte 1-2 = Sender ID
        Byte 3-4 = Receiver ID
        Byte 5 = Message-type
        Byte 6 = Payload length
        Byte 7+ = Payload
*/

#define PROTOCOL_HEADER 0xAA

// ID structure. leftmost 4 bits are the device-type, remaining 12 are the device number.
#define TAG_ID          0x1001
#define PROGRAMMER_ID  0x2001

// Message types:
#define SERIAL_PAIR_REQUEST     0x01
#define SERIAL_PAIR_ACK         0x02
#define ESB_PAIR_REQUEST        0x03
#define ESB_DATA                0x04
#define SERIAL_DATA             0x05
#define SERIAL_RECEIVED         0x06
#define ESB_RECEIVED            0x07

#define WIRELESS_HEADER_LEN     7

static const uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7}; // Pipe 0 address.
static const uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2}; // Pipe 1-7 addresses.
static const uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8}; // Address prefixes for pipe 0-7.

// Tag state machine.
enum tag_state {
        SEND_PROGRAMMER_PAIR,
        WAIT_FOR_PAIR_ACK,
        SEND_TAG_DATA,
        WAIT_FOR_DATA_ACK,
        SWITCH_TO_RX,
        WAIT_FOR_PC_RECEIVED,
        TAG_COMPLETE
};

static volatile enum tag_state tag_state = SEND_PROGRAMMER_PAIR;

// ESB payloads.
static struct esb_payload rx_payload;
static struct esb_payload tx_payload;

/*
        uint8 <--> uint16 conversion helpers.
*/
static uint16_t read_u16(const uint8_t *data)
{
        // Left-shift the first byte and append it to the second.
        return ((uint16_t)data[0] << 8 | data[1]);
}

static void write_u16(uint8_t *data, uint16_t value)
{
        // extract the first and second byte from the data and sore them as separate uint8's.
        data[0] = (uint8_t)(value >> 8); // Right-shift removes scaling.
        data[1] = (uint8_t)value;
}

/*
        ESB RX packet handling helpers.
*/
static bool validate_esb_packet(const struct esb_payload *packet, uint16_t expected_sender, uint16_t expected_receiver, uint8_t expected_type)
{
        // Packet is invalid if it doesn't contain the minimum required information from the header (7 bytes).
        if (packet->length < WIRELESS_HEADER_LEN) {
                LOG_WRN("ESB packet too short: %u", packet->length);
                return false;
        }

        // Extract header information from the packet.
        uint8_t header = packet->data[0];
        uint16_t sender_id = read_u16(&packet->data[1]);
        uint16_t receiver_id = read_u16(&packet->data[3]);
        uint8_t message_type = packet->data[5];
        uint8_t payload_length = packet->data[6];

        // Validity checks.
        if (header != PROTOCOL_HEADER) { // Header must match expected value (0xAA).
                LOG_WRN("Invalid wireless header");
                return false;
        }

        if (sender_id != expected_sender) {
                LOG_WRN("Unexpected sender ID: 0x%04X", sender_id);
                return false;
        }

        if (receiver_id != expected_receiver) {
                LOG_WRN("Unexpected receiver ID: 0x%04X", receiver_id);
                return false;
        }

        if (message_type != expected_type) { // Message type must match expected ID value based state-machine expectation.
                LOG_WRN("Unexpected wireless message type: 0x%02X", message_type);
                return false;
        }

        if (packet->length != (WIRELESS_HEADER_LEN + payload_length)) { // Total packet length must be equivalent to the sum of the header and payload length.
                LOG_WRN("Wireless payload length mismatch");
                return false;
        }

        return true;
}

static void handle_esb_message(const struct esb_payload *packet)
{
        switch (tag_state) {
        case WAIT_FOR_PC_RECEIVED: // Currently only one expected state for when tag is in RX mode.
                if (validate_esb_packet(packet, PROGRAMMER_ID, TAG_ID, ESB_RECEIVED)) { // Make sure packet follows expected structure.
                        LOG_INF("Valid PC data acknowledgement received");
                        tag_state = TAG_COMPLETE; // Complete state (protocol finished)
                        LOG_INF("Tag data-transfer complete");
                }
                break;

        default: // Log warning message if a packet is somehow received in wrong tag state.
                LOG_WRN("Unexpected tag protocol state%d", tag_state);
                break;
        }
}

/*
        ESB TX packet construction helper.
*/
static void prepare_esb_packet(struct esb_payload *packet, uint16_t sender_id, uint16_t receiver_id, uint8_t message_type, const uint8_t *payload, uint8_t payload_length)
{
        // Specify transmission pipe and whether outbound packet requires acknowledgement.
        packet->pipe = 0;
        packet->noack = false;
        // Write header information to the ESB packet.
        packet->data[0] = PROTOCOL_HEADER;
        write_u16(&packet->data[1], sender_id);
        write_u16(&packet->data[3], receiver_id);
        packet->data[5] = message_type;
        packet->data[6] = payload_length;

        // Iterate over the payload data and copy it into the ESB packet. Ignore step if payload is empty/NULL (occurs for acknowledgements/pair requests).
        if (payload_length > 0 && payload != NULL) {
                memcpy(&packet->data[7], payload, payload_length);
        }
        
        packet->length = WIRELESS_HEADER_LEN + payload_length; // Length is an attribute of ESB packets and exists separately of the 'length' header byte.
}

/*
        Use-cases of packet transmission during the protocol.
*/
static int send_pair_request(void)
{
        prepare_esb_packet(&tx_payload, TAG_ID, PROGRAMMER_ID, ESB_PAIR_REQUEST, NULL, 0);
        LOG_INF("Sending tag pair request");
        return esb_write_payload(&tx_payload);
}

static int send_test_data(void)
{
        static uint8_t test_data[(MAX_PAYLOAD_LEN - WIRELESS_HEADER_LEN)]; // Data buffer which has the equivalent capacity as the maximum ESB transmission (minus the header).

        // For every buffer position, add an incrementing byte value.
        for (uint16_t i = 0; i < (MAX_PAYLOAD_LEN - WIRELESS_HEADER_LEN); i++) {
                test_data[i] = (uint8_t)i;
        }

        prepare_esb_packet(&tx_payload, TAG_ID, PROGRAMMER_ID, ESB_DATA, test_data, (MAX_PAYLOAD_LEN - WIRELESS_HEADER_LEN));
        LOG_INF("Sending tag data");
        return esb_write_payload(&tx_payload);
}

/*
        Unique ESB events.
*/
static void handle_rx_received(void)
{
        // Keep attempting to receive transmissions while esb_read_rx_payload returns successes (0).
        while (esb_read_rx_payload(&rx_payload) == 0) {
                handle_esb_message(&rx_payload); // Validate and process the message.
        }
}

static void handle_tx_success(const struct esb_evt *event)
{
        LOG_INF("ESB TX success, attempts: %u", event->tx_attempts); // Log number of retransmission attempts.

        // Different actions based on current tag state.
        switch (tag_state) {
        case WAIT_FOR_PAIR_ACK: // If pair acknowledgement is received, then next tag state corresponds to sending the main data.
                LOG_INF("Pair request acknowledged");
                tag_state = SEND_TAG_DATA;
                break;

        case WAIT_FOR_DATA_ACK: // If acknowledgement is received for transmitted data, then next tag state corresponds to switching ESB mode to listen for PC receive acknowledgement.
                LOG_INF("Data acknowledged by programmer");
                tag_state = SWITCH_TO_RX;
                break;

        default: // Unexpected success from unspecified ESB transmission.
                LOG_WRN("Unexpected TX success in tag state %d", tag_state);
                break;
        }
}

static void handle_tx_failed(const struct esb_evt *event)
{
        LOG_WRN("ESB TX failed, attempts: %u", event->tx_attempts);

        int err;
        err = esb_pop_tx(); // Remove failed packet from the TX FIFO buffer so transmission can continue.
        if (err) {
                LOG_ERR("esb_pop_tx failed: %d", err);
        }
}

/*
        ESB event handler.
*/
static void esb_eventhandler(const struct esb_evt *event)
{
        // Capture ESB event id property of ESB event (cases seen below).
        switch (event->evt_id) {
        case ESB_EVENT_RX_RECEIVED:
                handle_rx_received();
                break;

        case ESB_EVENT_TX_SUCCESS:
                handle_tx_success(event);
                break;

        case ESB_EVENT_TX_FAILED:
                handle_tx_failed(event);
                break;

        default:
                break;
        }
}

/*
        ESB initialisation.
*/
static int esb_radio_init(enum esb_mode mode)
{
        int err;
        // Create configuration with default parameters initially.
        struct esb_config config = ESB_DEFAULT_CONFIG;
        // Set specific configuration information (i.e. mode, event handling, auto-acknowledgement).
        config.mode = mode;
        config.event_handler = esb_eventhandler; // Manually link event handling function.
        config.selective_auto_ack = true; // Every packet transmitted requires acknowledgement

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

        err = esb_set_base_address_1(base_addr_1); // Set base address of pipe 1-7.
        if (err) {
                LOG_ERR("base address 1 failed: %d", err);
                return err;
        }

        err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix)); // Attach unique prefixes to each pipe address.
        if (err) {
                LOG_ERR("prefix configuration failed: %d", err);
                return err;
        }

        err = esb_set_rf_channel(40); // Set RF channel to match protocol standard.
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
                LOG_INF("ESB initialised as PRX");
        } else {
                LOG_INF("ESB initialised as PTX"); // No additional steps since ESB transmissions can be called on-demand in automatic mode (which we are using).
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
        Main.
*/
int main(void)
{
        int err;

        LOG_INF("nRF54L15 DK ESB starting");

        err = esb_radio_init(ESB_MODE_PTX); // Initialise ESB module.
        if (err) {
                LOG_ERR("ESB initialisation failed: %d", err);
                return 0;
        }

        // Keep looping over this logic.
        while (1) {
                switch(tag_state) {
                case SEND_PROGRAMMER_PAIR: // Initial state where tag requests a paired connection with the wireless programmer.
                        err = send_pair_request(); // Call pair-request function.
                        if (err) {
                                LOG_ERR("Failed to send pair request: %d", err);
                        } else {
                                tag_state = WAIT_FOR_PAIR_ACK; // Transition to awaiting pair-request acknowledgement from the wireless programmer.
                        }
                        break;

                case WAIT_FOR_PAIR_ACK: // Waiting for acknowledgement after transmitting an ESB pair request.
                        // State is handled via ESB_EVENT_TX_SUCCESS / TX_FAILED since it corresponds to an automatic acknowledgement.
                        break;

                case SEND_TAG_DATA: // After getting a pair acknowledgement, transmit stored data to be relayed to the PC.
                        err = send_test_data(); // Call mock data-transmission function.
                        if (err) {
                                LOG_ERR("Failed to send data: %d", err);
                        } else {
                                tag_state = WAIT_FOR_DATA_ACK; // Transition to awaiting data-transmission acknowledgement from the wireless programmer.
                        }
                        break;

                case WAIT_FOR_DATA_ACK:
                        // Also handled by ESB_EVENT_TX_SUCCESS / TX_FAILED.
                        break;

                case SWITCH_TO_RX: // Switch to RX mode after wireless programmer acknowledges the data.
                        LOG_INF("Switching tag to PRX");
                        
                        err = esb_switch_mode(ESB_MODE_PTX, ESB_MODE_PRX); // Change from TX to RX.
                        if (err) {
                                LOG_ERR("Failed to switch tag to PRX: %d", err);
                                break;
                        }

                        tag_state = WAIT_FOR_PC_RECEIVED; // Transition to state awaiting data confirmation from the PC.
                        LOG_INF("Waiting for programmer RECEIVED message");
                        break;

                case WAIT_FOR_PC_RECEIVED:
                        // Also handled by ESB event-handler.
                        break;

                case TAG_COMPLETE:
                        // Protocol completed.
                        LOG_INF("Transfer complete");
                        k_sleep(K_FOREVER); // Stop checking states and become idle.
                        break;

                default:
                        break;
                }
                k_sleep(K_MSEC(10)); // Small break to avoid constantly polling ESB.
        }
        return 0;
}
