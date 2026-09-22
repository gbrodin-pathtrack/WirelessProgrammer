#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <esb.h> // ESB wireless communication library.
#include <errno.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(wireless_programmer, LOG_LEVEL_INF);

// Get node identifier for a /chosen node property.
#define UART_NODE DT_NODELABEL(uart20)

// Packet constraint information (value based on Nordic ESB limit)
#define MAX_PAYLOAD_LEN 252

#define FEM_PDN_PIN 0

/* Serial packet format:
        Byte 0 = Header
        Byte 1 = Message type
        Byte 2 = Length
        Byte 3+ = Payload
*/
/* ESB packet format:
        Byte 0 = Header
        Byte 1-2 = Sender ID
        Byte 3-4 = Receiver ID
        Byte 5 = Message type
        Byte 6 = Length
        Byte 7+ = Payload

*/

// constant protocol header for testing.
#define PROTOCOL_HEADER 0xAA

// ID structure. leftmost 4 bits are the device-type, remaining 12 are the device number.
#define PROGRAMMER_ID   0x2001

// Message types:
#define SERIAL_PAIR_REQUEST     0x01
#define SERIAL_PAIR_ACK         0x02
#define ESB_PAIR_REQUEST        0x03
#define ESB_DATA                0x04
#define SERIAL_DATA             0x05
#define SERIAL_RECEIVED         0x06
#define ESB_RECEIVED            0x07

#define WIRELESS_HEADER_LEN 7

// Configure ESB ddress for pipes.
static const uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7}; // Pipe 0 is unique because it handles ACK transmission.
static const uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2}; // General-use address for pipes 1-7.
static const uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8}; // Address prefixes to give each pipe a unique identifier (one for pipe 0-7).

// Get device reference from a devicetree node identifier.
static const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);

// FEM PDN pin initialisation.
static const struct device *gpio2_dev = DEVICE_DT_GET(DT_NODELABEL(gpio2));

// Group of constants representing each serial receiver state.
enum serial_rx_state {
        WAIT_FOR_HEADER,
        WAIT_FOR_MESSAGE_TYPE,
        WAIT_FOR_LENGTH,
        RECEIVE_PAYLOAD
};

// Group of constants represetning each ESB receiver state.
enum programmer_state {
        WAIT_FOR_PC_PAIR_ACK,
        WAIT_FOR_TAG_PAIR,
        WAIT_FOR_TAG_DATA,
        RELAY_DATA_TO_PC,
        WAIT_FOR_PC_RECEIVED,
        SEND_RECEIVED_TO_TAG,
        WAIT_FOR_RECEIVED_ACK,
        SWITCH_TO_RX
};
static volatile enum programmer_state programmer_state = WAIT_FOR_PC_PAIR_ACK;

// ESB payloads.
static struct esb_payload rx_payload;
static struct esb_payload tx_payload;

// global session information about tag.
static uint16_t active_tag_id = 0;
// Persistent application payload storage.
static uint8_t tag_data[MAX_PAYLOAD_LEN];
static uint8_t tag_data_length;
// Persistent serial receive storage (Capacity bounded by Nordic's ESB packet limit).
static uint8_t serial_rx_buffer[MAX_PAYLOAD_LEN];

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
        Serial packet transmission/reception helpers.
*/
static void serial_transmit(uint8_t message_type, const uint8_t *payload, uint8_t length)
{
        // Transmit general information (header, message-type, length) over serial connection.
        uart_poll_out(uart_dev, PROTOCOL_HEADER);
        uart_poll_out(uart_dev, message_type);
        uart_poll_out(uart_dev, length);

        // Iterate over the prepared payload and transmit byte-by-byte.
        for (uint8_t i=0; i < length; i++) {
                uart_poll_out(uart_dev, payload[i]);
        }
}

static int serial_receive(uint8_t *message_type, uint8_t *payload, uint8_t *length)
{
        enum serial_rx_state state = WAIT_FOR_HEADER; // Initial listening state.
        uint8_t rx_byte; // Immediate storage location for incoming data.
        uint8_t rx_index = 0; // Next available index of incoming payload byte.
        uint8_t expected_length = 0; // Expected length based on the contents of the length byte.
        uint16_t timeout_ms = 10000;

        while (timeout_ms > 0) { // Only wait for a serial response until the timout is reached so not waiting forever.
                // Attempt to read a character from the device and write at the address of rx_byte (0 if successful)
                if (uart_poll_in(uart_dev, &rx_byte) == 0) {

                        switch (state) {
                        case WAIT_FOR_HEADER: // System is initially waiting for a matching header byte to be received.
                                if (rx_byte == PROTOCOL_HEADER) {
                                        state = WAIT_FOR_MESSAGE_TYPE; // Switches to waiting for message-type byte if received PC byte matches the expected header.
                                }
                                break; // Exit to listen loop.

                        case WAIT_FOR_MESSAGE_TYPE: // System is waiting for a message-type identifier byte .
                                *message_type = rx_byte;
                                state = WAIT_FOR_LENGTH; // Switches to next waiting for length byte.
                                break;

                        case WAIT_FOR_LENGTH: // System is waiting for next byte representing expected payload length.
                                expected_length = rx_byte;
                                *length = expected_length;
                                rx_index = 0; // reset the buffer index from potential previous message.

                                if (expected_length == 0) { // If length is 0 then there is nothing further to expect (applies to pair requests and acknowledgements).
                                        return 0; // Successful exit from function.
                                }
                                state = RECEIVE_PAYLOAD; // Switches to receiving payload data for next expected_length bytes.
                                break;

                        case RECEIVE_PAYLOAD: // System is receiving payload bytes.
                                payload[rx_index++] = rx_byte; // Simultaneously add byte to buffer and increment rx_index.
                                if (rx_index >= expected_length) { // Stop adding data to rx_buffer if all expected bytes are received.
                                        return 0; // Successful exit from function.
                                }
                                break;
                        }
                }
                k_sleep(K_MSEC(1)); // timeout for 1ms so not constantly hammering CPU while waiting for UART traffic
                timeout_ms--;
        }
        return -ETIMEDOUT; // Return timeout error if a full valid packet is never received.
}

/*
        Serial event handler.
*/
static bool handle_serial_message(uint8_t message_type, const uint8_t *payload, uint8_t length)
{
        ARG_UNUSED(payload); // Disables warning since payload field is currently unused (may eventually need it if the PC were to ever start transmitting messages with a payload)

        switch (programmer_state) {
        case WAIT_FOR_PC_PAIR_ACK: // Wireless programmer is expecting an acknowledgement after it sends a serial pair request.
                // Validating that the message-type identifier and payload length match the expected values for a serial pair acknowledgement.
                if (message_type != SERIAL_PAIR_ACK) {
                        LOG_WRN("Expected SERIAL_PAIR_ACK, received 0x%02X", message_type);
                        return false;
                }

                if (length != 0) {
                        LOG_WRN("SERIAL_PAIR_ACK must have zero payload");
                        return false;
                }

                LOG_INF("PC pair acknowledgement received");
                programmer_state = WAIT_FOR_TAG_PAIR; // Wireless programmer next expects a wireless pair request once serial connection is established (not handled in this function).
                return true; // Boolean function returns true on successfu event completions.

        case WAIT_FOR_PC_RECEIVED: // Wireless programmer expects a confirmation of reception once the tag data has been relayed to the PC.
                // Validating type and payload length just as or the serial pair acknowledgement.
                if (message_type != SERIAL_RECEIVED) {
                        LOG_WRN("Expected SERIAL_RECEIVED, received 0x%02X", message_type);
                        return false;
                }

                if (length != 0) { // Payload length is also expected to be 0 since this is essentially just a data acknowledgement.
                        LOG_WRN("SERIAL_RECEIVED must have zero payload");
                        return false;
                }

                LOG_INF("PC confirmed data received");
                programmer_state = SEND_RECEIVED_TO_TAG; // Wireless programmer should next relay the 'received' message back to the tag once it is verified on serial-side.
                return true;

        default: // Default case for unexpected messages beyond the discussed protocol states.
                LOG_WRN("Unexpeted serial message 0x%02X in programmer state %d", message_type, programmer_state);
                return false;
        }
}

/*
        Use-case of serial functions to establish serial connection between the WP and PC with acknowledgements.
*/
static int serial_pair_with_pc(void)
{
        int err;
        uint8_t message_type;
        uint8_t length;

        LOG_INF("Sending PC pair request");

        serial_transmit(SERIAL_PAIR_REQUEST, NULL, 0); // Transmit a message of type 'serial pair request' (request doesn't have a payload).

        err = serial_receive(&message_type, serial_rx_buffer, &length); // Function writes message type and length to the local variable addresses.
        if (err) {
                LOG_ERR("No serial response received %d", err);
                return err;
        }

        if (!handle_serial_message(message_type, serial_rx_buffer, length)) { // Validates received packet against constraints imposed at current internal state (WAIT_FOR_PC_PAIR_ACK).
                LOG_ERR("Invalid response to serial pair request");
                return -EPROTO; // Protocol error code.
        }

        return 0;
}

/*
        ESB RX packet handling helpers.
*/
static bool validate_pair_request(const struct esb_payload *packet)
{
        // Packet is invalid if it doesn't contain the required information from the header for a pair request (7 bytes).
        if (packet->length != WIRELESS_HEADER_LEN) {
                LOG_WRN("Wireless payload length mismatch");
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

        if (sender_id == PROGRAMMER_ID) { // Sender cannot claim to have the ID used by this device.
                LOG_WRN("Sender ID cannot belong to current device");
                return false;
        }

        if (receiver_id != PROGRAMMER_ID) { // Pair request must be addressed to the current device.
                LOG_WRN("Unexpected receiver ID: 0x%04X", receiver_id);
                return false;
        }

        if (message_type != ESB_PAIR_REQUEST) { // Pair request must have correct type.
                LOG_WRN("Unexpected wireless message type: 0x%02X", message_type);
                return false;
        }

        if (payload_length != 0) { // Pair request payload must be empty.
                LOG_WRN("Wireless data length mismatch");
                return false;
        }

        return true;
}

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
        switch (programmer_state) {
        case WAIT_FOR_TAG_PAIR: // Wireless programmer is listening for a wireless pair request from the tag.
                if (validate_pair_request(packet)) { // Make sure packet follows expected structure.
                        active_tag_id = read_u16(&packet->data[1]);
                        LOG_INF("Valid tag pair request received");
                        programmer_state = WAIT_FOR_TAG_DATA; // Wireless programmer should next expect to be sent some data from the tag once paired.
                        LOG_INF("Programmer now waiting for tag data");
                }
                break;

        case WAIT_FOR_TAG_DATA: // Wireless programmer is listening for some data with the 'ESB_DATA message identifier.
                if (validate_esb_packet(packet, active_tag_id, PROGRAMMER_ID, ESB_DATA)) {
                        tag_data_length = packet->data[6]; // Extracts the payload size according to the length byte

                        if (tag_data_length > 0) { // Only attempts to copy payload data if the length byte idicates that a payload exists.
                                memcpy(tag_data, &packet->data[7], tag_data_length); // Copy the payload data into persistent data storage.
                        }
                        LOG_INF("Valid tag data packet received, length=%u", tag_data_length);
                        
                        programmer_state = RELAY_DATA_TO_PC; // Enter next state where the received data is relayed to the PC over paired serial connection.
                }
                break;

        default: // Log warning message if a packet is somehow received in wrong programmer state.
                LOG_WRN("Unexpected programmer protocol state %d", programmer_state);
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
static int send_received_message(void)
{
        prepare_esb_packet(&tx_payload, PROGRAMMER_ID, active_tag_id, ESB_RECEIVED, NULL, 0);
        LOG_INF("Sending RECEIVED confirmation to tag");
        return esb_write_payload(&tx_payload);
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

        // Different actions based on current programmer state.
        switch(programmer_state) {
        case WAIT_FOR_RECEIVED_ACK: // Awaiting acknowledgement that the tag received the PC's data acknowledgement.
                LOG_INF("Tag acknowledged RECEIVED message");
                programmer_state = SWITCH_TO_RX; // Return to receive mode ready for the tag's next data packet.
                break;

        default: // Unexpected success from unspecified ESB transmission.
                LOG_WRN("Unexpected TX success in programmer state %d", programmer_state);
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

        // Pick an RF channel. ESB channel 40 corresponds to 2400MHz + 40 MHz = 2440MHz (receiver must be the same).
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

        LOG_INF("Wireless programmer starting");

        // Confirm that serial peripheral is ready
        if (!device_is_ready(uart_dev)) {
                LOG_ERR("UART device not ready");
                return 0; // Exit if there is an error with the device.
        }

        // Establish wired connection to PC.
        err = serial_pair_with_pc();
        if (err) {
                LOG_ERR("PC pairing failed: %d", err);
                return 0;
        }
        LOG_INF("PC paired successfully");

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

        LOG_INF("Wireless programmer waiting for tag");

        // Keep looping over this logic.
        while (1){
                switch (programmer_state) {
                case WAIT_FOR_TAG_PAIR:
                case WAIT_FOR_TAG_DATA:
                case WAIT_FOR_RECEIVED_ACK:
                        break; // These states advance from ESB events.

                case RELAY_DATA_TO_PC: // Received tag data needs to be relayed to the PC over serial connection,
                        LOG_INF("Relaying tag data to PC");
                        serial_transmit(SERIAL_DATA, tag_data, tag_data_length); // Transmit a serial message of type 'SERIAL_DATA' containing the transmitted tag data.
                        programmer_state = WAIT_FOR_PC_RECEIVED; // Transition to waiting for confirmation of received data from the PC.
                        break;

                case WAIT_FOR_PC_RECEIVED: { // Wireless programmer is receiving the confirmation of relayed data arrival.
                        uint8_t message_type;
                        uint8_t length;

                        err = serial_receive(&message_type, serial_rx_buffer, &length); // Receive incoming serial packet and locally store its content and information.
                        if (err) {
                                LOG_WRN("No PC receive confirmation: %d", err);
                                break; // Stay in the same state and continue waiting if nothing received.
                        }

                        if (!handle_serial_message(message_type, serial_rx_buffer, length)) { // Handle the packet according to the expected message-type in this state.
                                LOG_WRN("Invalid PC response");
                        }
                        break;
                }

                case SEND_RECEIVED_TO_TAG: // The PC data acknowledgement needs to be relayed back to the tag.
                        err = esb_switch_mode(ESB_MODE_PRX, ESB_MODE_PTX); // Switch the wireless programmer into ESB TX mode.
                        if (err) {
                                LOG_ERR("Failed to switch ESB to PTX: %d", err);
                                break;
                        }

                        err = send_received_message(); // Call upon function to relay the PC's 'received' message over ESB.
                        if (err) {
                                LOG_ERR("Failed to send RECEIVED to tag: %d", err);
                                break;
                        }
                        programmer_state = WAIT_FOR_RECEIVED_ACK; // Await confirmation from the tag that it received the message.
                        break;

                case SWITCH_TO_RX:
                        LOG_INF("Transfer complete; switching programmer back to PRX");

                        err = esb_switch_mode(ESB_MODE_PTX, ESB_MODE_PRX);
                        if (err) {
                                LOG_ERR("Failed to switch ESB back to PRX: %d", err);
                                break;
                        }

                        programmer_state = WAIT_FOR_TAG_DATA;
                        LOG_INF("Programmer ready for next tag data packet");
                        break;

                default:
                        break;
                }
                k_sleep(K_MSEC(10)); // Small break to avoid constantly polling ESB.
        }
        return 0;
}
