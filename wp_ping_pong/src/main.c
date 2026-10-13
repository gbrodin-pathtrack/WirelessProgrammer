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
    swapping PTX/PRX for every transfer so ESB ACKs stay empty. Each exchange is printed as plain text on RTT, or on
    uart20 when SERIAL_OUTPUT_UART is set.
    NMini appends its RSSI of the reply it is answering, so each line shows the signal strength in both directions.

    Builds for the wireless programmer by default (app.overlay adds the nRF21540 FEM). Build with -DFILE_SUFFIX=dk to
    use app_dk.overlay instead and run on a plain nRF54L15 DK, for comparing RF performance against the programmer.
*/

// Set to 1 to print on uart20 instead of RTT.
#define SERIAL_OUTPUT_UART      1

// Radio settings must match NMini (sandbox_pc_bin Pathtrack/minisrc/common/esb/radio_profile.c/h).
#define RF_CHANNEL              23      // 2423 MHz: between Wi-Fi channels 1 and 6, clear of BLE advertising channel 38 (2426 MHz).
#define RETRANSMIT_COUNT        3
#define RETRANSMIT_DELAY_US     5000

#if defined(CONFIG_MPSL_FEM_NRF21540_GPIO)
// The nRF21540 PA adds its fixed gain on top (ESB doesn't split TX power with the FEM on nRF54L).
#define TX_POWER_DBM            0
#define FEM_TX_GAIN_DB          CONFIG_MPSL_FEM_NRF21540_TX_GAIN_DB
#define FEM_RX_GAIN_DB          CONFIG_MPSL_FEM_NRF21540_RX_GAIN_DB
#else
#define TX_POWER_DBM            8       // nRF54L15 maximum; ESB falls back to 0 dBm for anything higher.
#define FEM_TX_GAIN_DB          0
#define FEM_RX_GAIN_DB          0
#endif

// 0xE7 rather than 0xA_ bytes so the address doesn't continue the preamble's alternating 1010 pattern.
static const uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
static const uint8_t addr_prefix[1] = {0xE7}; // Single pipe (pipe 0).

#define COUNTER_LEN             4
#define RX_PAYLOAD_LEN          (COUNTER_LEN + 1) // Counter followed by NMini's RSSI.
#define TAG_RSSI_NONE           0 // NMini hasn't received a reply yet.
#define TX_TIMEOUT_MS           100
#define TURNAROUND_DELAY_MS     16

#define FEM_PDN_PIN 0

#if SERIAL_OUTPUT_UART
static const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart20));
#endif
#if defined(CONFIG_MPSL_FEM_NRF21540_GPIO)
static const struct device *gpio2_dev = DEVICE_DT_GET(DT_NODELABEL(gpio2));
#endif

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
    char line[96];
    va_list args;

    va_start(args, fmt);
    int len = vsnprintk(line, sizeof(line), fmt, args);
    va_end(args);

    len = MIN(len, (int)sizeof(line) - 1); // vsnprintk returns the untruncated length.
#if SERIAL_OUTPUT_UART
    for (int i = 0; i < len; i++) {
        uart_poll_out(uart_dev, line[i]);
    }
#else
    // printk is routed through the logger to RTT, so each call should be a whole line to avoid interleaving.
    printk("%.*s", len, line);
#endif
}

/*
    FEM initialisation.
*/
#if defined(CONFIG_MPSL_FEM_NRF21540_GPIO)
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
#endif

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
    config.tx_output_power = TX_POWER_DBM;

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

#if SERIAL_OUTPUT_UART
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
        return 0;
    }
#endif

#if defined(CONFIG_MPSL_FEM_NRF21540_GPIO)
    err = fem_pdn_init();
    if (err) {
        LOG_ERR("FEM PDN initialisation failed: %d", err);
        return 0;
    }
#endif

    err = esb_radio_init(ESB_MODE_PRX);
    if (err) {
        LOG_ERR("ESB initialisation failed: %d", err);
        return 0;
    }

    serial_print("wp ping/pong ready, ch=%u, radio %d dBm + PA %d dB\r\n", (unsigned int)RF_CHANNEL, TX_POWER_DBM,
                 FEM_TX_GAIN_DB);

    while (1) {
        struct rx_counter rx_counter;
        uint32_t tx_counter;
        char err_text[16] = "";
        char tag_text[16] = "";

        k_msgq_get(&rx_counter_queue, &rx_counter, K_FOREVER);
        tx_counter = rx_counter.value + 1;

        // Give NMini time to swap to PRX before replying. Staying in PRX meanwhile means its
        // retransmissions are still ACKed if it missed the first ACK.
        k_sleep(K_MSEC(TURNAROUND_DELAY_MS));

        err = send_counter(tx_counter);

        // Printed after sending so the output doesn't delay the reply.
        if (err) {
            snprintk(err_text, sizeof(err_text), " failed (%d)", err);
        }
        if (rx_counter.tag_rssi != TAG_RSSI_NONE) {
            snprintk(tag_text, sizeof(tag_text), " tag -%u dBm", (unsigned int)rx_counter.tag_rssi);
        }
        // ESB measures RSSI after the nRF21540 LNA, so remove its gain to give the level at the antenna.
        serial_print("RX %u -> TX %u%s, RSSI wp %d dBm%s\r\n", (unsigned int)rx_counter.value,
                     (unsigned int)tx_counter, err_text, -(int)rx_counter.rssi - FEM_RX_GAIN_DB, tag_text);

        // Back to PRX whatever happened; NMini resends its counter if it missed the reply.
        err = esb_switch_mode(ESB_MODE_PTX, ESB_MODE_PRX);
        if (err) {
            LOG_ERR("Failed to return to PRX: %d", err);
        }
    }
    return 0;
}
