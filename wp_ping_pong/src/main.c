#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <esb.h>
#include <errno.h>
#include <stdarg.h>

LOG_MODULE_REGISTER(wp_ping_pong, LOG_LEVEL_INF);

/*
    ESB ping/pong bring-up test against NMini (sandbox_pc_bin Pathtrack/src/nrfmodules/wireless_offload/wl_ping_pong.c).
    The programmer starts as PRX and NMini as PTX. Each side adds one to the counter it receives and sends it back,
    swapping PTX/PRX for every transfer so ESB ACKs stay empty. Each exchange is printed as plain text on uart20.
    NMini appends its RSSI of the reply it is answering, so each line shows the signal strength in both directions.
*/

// Radio settings must match NMini (sandbox_pc_bin Pathtrack/minisrc/common/esb/radio_profile.c/h).
#define RF_CHANNEL              23      // 2423 MHz: between Wi-Fi channels 1 and 6, clear of BLE advertising channel 38 (2426 MHz).
#define RETRANSMIT_COUNT        3
#define RETRANSMIT_DELAY_US     5000
// 0xE7 rather than 0xA_ bytes so the address doesn't continue the preamble's alternating 1010 pattern.
static const uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
static const uint8_t addr_prefix[1] = {0xE7}; // Single pipe (pipe 0).

#define COUNTER_LEN             4
#define RX_PAYLOAD_LEN          (COUNTER_LEN + 1) // Counter followed by NMini's RSSI.
#define TAG_RSSI_NONE           0 // NMini hasn't received a reply yet.
#define TX_TIMEOUT_MS           100
#define TURNAROUND_DELAY_MS     16

#define FEM_PDN_PIN 0

static const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart20));
static const struct device *gpio2_dev = DEVICE_DT_GET(DT_NODELABEL(gpio2));

// ESB reports RSSI as a positive value meaning -rssi dBm.
struct rx_counter {
    uint32_t value;
    uint8_t rssi;       // Programmer's RSSI of this packet.
    uint8_t tag_rssi;   // NMini's RSSI of the reply this packet answers.
};

K_MSGQ_DEFINE(rx_counter_queue, sizeof(struct rx_counter), CONFIG_ESB_RX_FIFO_SIZE, 4);
static K_SEM_DEFINE(tx_done_sem, 0, 1);
static volatile bool tx_ok;

/*
    Plain-text serial output.
*/
static void serial_print(const char *fmt, ...)
{
    char line[64];
    va_list args;

    va_start(args, fmt);
    int len = vsnprintk(line, sizeof(line), fmt, args);
    va_end(args);

    len = MIN(len, (int)sizeof(line) - 1); // vsnprintk returns the untruncated length.
    for (int i = 0; i < len; i++) {
        uart_poll_out(uart_dev, line[i]);
    }
}

/*
    FEM initialisation.
*/
static int fem_pdn_init(void)
{
    int err;

    // PDN is on GPIO2.00, which MPSL can't drive because Nordic doesn't support FEM controls split across GPIO ports.
    if (!device_is_ready(gpio2_dev)) {
        LOG_ERR("GPIO2 not ready");
        return -ENODEV;
    }

    err = gpio_pin_configure(gpio2_dev, FEM_PDN_PIN, GPIO_OUTPUT_LOW);
    if (err) {
        LOG_ERR("Failed to configure FEM PDN: %d", err);
        return err;
    }

    err = gpio_pin_set(gpio2_dev, FEM_PDN_PIN, 1);
    if (err) {
        LOG_ERR("Failed to enable FEM PDN: %d", err);
        return err;
    }

    k_sleep(K_MSEC(1)); // Let the nRF21540 leave power-down before any radio activity.
    LOG_INF("nRF21540 PDN enabled on P2.00");
    return 0;
}

/*
    ESB event handler.
*/
static void esb_eventhandler(const struct esb_evt *event)
{
    static struct esb_payload rx_payload;

    switch (event->evt_id) {
    case ESB_EVENT_RX_RECEIVED:
        while (esb_read_rx_payload(&rx_payload) == 0) {
            if (rx_payload.length == RX_PAYLOAD_LEN) {
                struct rx_counter counter = {
                    .value = sys_get_le32(rx_payload.data),
                    .rssi = rx_payload.rssi,
                    .tag_rssi = rx_payload.data[COUNTER_LEN],
                };

                (void)k_msgq_put(&rx_counter_queue, &counter, K_NO_WAIT);
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

/*
    ESB initialisation and mode switching.
*/
static int esb_radio_init(enum esb_mode mode)
{
    int err;
    struct esb_config config = ESB_DEFAULT_CONFIG;

    config.mode = mode;
    config.event_handler = esb_eventhandler;
    config.bitrate = ESB_BITRATE_4MBPS;
    config.retransmit_count = RETRANSMIT_COUNT;
    config.retransmit_delay = RETRANSMIT_DELAY_US;
    config.selective_auto_ack = true;

    err = esb_init(&config);
    if (err) {
        LOG_ERR("esb_init failed: %d", err);
        return err;
    }

    err = esb_set_base_address_0(base_addr_0);
    if (err) {
        LOG_ERR("base address 0 failed: %d", err);
        return err;
    }

    err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
    if (err) {
        LOG_ERR("prefix configuration failed: %d", err);
        return err;
    }

    err = esb_set_rf_channel(RF_CHANNEL);
    if (err) {
        LOG_ERR("Failed to set RF channel: %d", err);
        return err;
    }

    if (mode == ESB_MODE_PRX) {
        err = esb_start_rx();
        if (err) {
            LOG_ERR("Failed to start ESB RX: %d", err);
            return err;
        }
    }
    return 0;
}

static int esb_switch_mode(enum esb_mode current_mode, enum esb_mode new_mode)
{
    int err;

    if (current_mode == ESB_MODE_PRX) {
        err = esb_stop_rx();
        if (err && err != -EALREADY) {
            LOG_ERR("Failed to stop ESB RX: %d", err);
            return err;
        }
    }

    esb_disable();

    if (new_mode == ESB_MODE_PRX) {
        k_msgq_purge(&rx_counter_queue); // Only answer counters received in this PRX window.
    }
    return esb_radio_init(new_mode);
}

static int send_counter(uint32_t counter)
{
    struct esb_payload tx_payload = {
        .pipe = 0,
        .noack = false,
        .length = COUNTER_LEN,
    };
    int err = esb_switch_mode(ESB_MODE_PRX, ESB_MODE_PTX);

    if (err) {
        return err;
    }

    sys_put_le32(counter, tx_payload.data);
    k_sem_reset(&tx_done_sem);
    err = esb_write_payload(&tx_payload);
    if (err) {
        return err;
    }

    if (k_sem_take(&tx_done_sem, K_MSEC(TX_TIMEOUT_MS)) != 0) {
        return -ETIMEDOUT;
    }
    return tx_ok ? 0 : -EIO;
}

/*
    Main.
*/
int main(void)
{
    int err;

    LOG_INF("Wireless programmer ping/pong starting");

    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
        return 0;
    }

    err = fem_pdn_init();
    if (err) {
        LOG_ERR("FEM PDN initialisation failed: %d", err);
        return 0;
    }

    err = esb_radio_init(ESB_MODE_PRX);
    if (err) {
        LOG_ERR("ESB initialisation failed: %d", err);
        return 0;
    }

    serial_print("wp ping/pong ready, ch=%u\r\n", (unsigned int)RF_CHANNEL);

    while (1) {
        struct rx_counter rx_counter;
        uint32_t tx_counter;

        k_msgq_get(&rx_counter_queue, &rx_counter, K_FOREVER);
        tx_counter = rx_counter.value + 1;

        // Give NMini time to swap to PRX before replying. Staying in PRX meanwhile means its
        // retransmissions are still ACKed if it missed the first ACK.
        k_sleep(K_MSEC(TURNAROUND_DELAY_MS));

        err = send_counter(tx_counter);

        // Printed after sending so the UART doesn't delay the reply.
        serial_print("RX %u -> TX %u", (unsigned int)rx_counter.value, (unsigned int)tx_counter);
        if (err) {
            serial_print(" failed (%d)", err);
        }
        // ESB measures RSSI after the nRF21540 LNA, so remove its gain to give the level at the antenna.
        serial_print(", RSSI wp %d dBm", -(int)rx_counter.rssi - CONFIG_MPSL_FEM_NRF21540_RX_GAIN_DB);
        if (rx_counter.tag_rssi != TAG_RSSI_NONE) {
            serial_print(" tag -%u dBm", (unsigned int)rx_counter.tag_rssi);
        }
        serial_print("\r\n");

        // Back to PRX whatever happened; NMini resends its counter if it missed the reply.
        err = esb_switch_mode(ESB_MODE_PTX, ESB_MODE_PRX);
        if (err) {
            LOG_ERR("Failed to return to PRX: %d", err);
        }
    }
    return 0;
}
