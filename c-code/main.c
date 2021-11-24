/**
 * @file  thermal-camera-demo/c-code/main.c
 * @brief Thermal camera to Chrome app demo based on Melexis MLX90640 IR array.
 *        
 *        This application includes both the Bluetooth stack and FPGA binary,
 *        however it would be too big to fit in the flash region of the nRF
 *        chip. Therefore the application must be run in two stages. One to 
 *        flash the FPGA binary to the external flash, and then again with the
 *        Bluetooth application for runtime.
 * 
 *        Build with CFLAG += DBLUETOOTH_ENABLED to exclude the FPGA binary and
 *        include the Bluetooth application. When using this option, the
 *        Softdevice must also be flashed to the nRF chip.
 * 
 * @attention Copyright 2021 Silicon Witchery AB
 *
 * Permission to use, copy, modify, and/or distribute this software for any 
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
 * REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY 
 * AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, 
 * INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM 
 * LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
 * OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR 
 * PERFORMANCE OF THIS SOFTWARE.
 */

#include <math.h>
#include "nrf_strerror.h"
#include "app_error.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_srv_common.h"
#include "ble.h"
#include "fpga_binfile.h"
#include "nordic_common.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_lesc.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh.h"
#include "nrf52811.h"
#include "nrfx_clock.h"
#include "nrfx_saadc.h"
#include "nrfx_spim.h"
#include "nrfx_twim.h"
#include "s1.h"

/**
 * @brief We have two timer instances. One for the FPGA flash state machine, and
 *        another for the Bluetooth application state machine. Only one runs at 
 *        a time and is decided on at compile time.
 */
APP_TIMER_DEF(bluetooth_app_task);
APP_TIMER_DEF(fpga_flasher_task);

/**
 * @brief Declarations for the GATT and advertising global observers.
 */
NRF_BLE_GATT_DEF(m_gatt);
BLE_ADVERTISING_DEF(m_advertising);

/**
 * @brief List of states for the FPGA flasher application.
 */
typedef enum
{
    STARTED,
    ERASING,
    FLASHING,
    DONE,
} fpga_flasher_state_t;

/**
 * @brief State machine variable and counter variables for the flashing process.
 */
static fpga_flasher_state_t fpga_flasher_state = STARTED;
static uint32_t flash_pages_remaining;
static uint32_t flash_page_address = 0x000000;

/**
 * @brief List of states for the Bluetooth application which sends camera data
 *        to the web app.
 */
typedef enum
{
    BOOT_FPGA,
    INIT_SPI,
    READ_SPI_DATA,
    SEND_PACKETS,
} bluetooth_app_state_t;

/**
 * @brief State machine variable for the bluetooth to web app transfer.
 */
static bluetooth_app_state_t bluetooth_app_state = BOOT_FPGA;

/**
 * @brief Complete image frame buffer where we store the camera data from SPI.
 */
static uint8_t image_buffer[3072];

/**
 * @brief The Bluetooth base UUID for the camera data service
 */
#define SILICONWITCHERY_BASE_UUID                          \
    {                                                      \
        0xab, 0x5a, 0x11, 0x37, 0xc5, 0xbf, 0x4d, 0xe8,    \
            0x9e, 0xfe, 0x4d, 0x18, 0x81, 0x04, 0xfd, 0x48 \
    }

/**
 * @brief This value is appended to the Bluetooth base UUID and becomes the 
 *        camera service UUID.
 */
#define SERVICE_UUID 0x1000

/**
 * @brief This value is appended to the Bluetooth base UUID and becomes the
 *        camera characteristic UUID.
 */
#define CHARACTERISTIC_UUID 0x1001

/**
 * @brief The payload size is negotiated by the receiving device. This variable
 *        keeps track of that size so we can break the camera data into chunks
 *        according to this size.
 */
static uint16_t negotiated_mtu_size;

/**
 * @brief This struct holds the connection handles for our camera service.
 */
typedef struct
{
    uint8_t uuid_type;
    uint16_t service_handle;
    ble_gatts_char_handles_t char_handles;
    uint16_t conn_handle;
} ble_service_t;

/**
 * @brief Flag we set when the Bluetooth is ready to send notifications.
 */
static bool bluetooth_ready = false;

/**
 * @brief Counter of how many chunks of data we have sent to the web app.
 */
static uint8_t chunks_sent = 0;

/**
 * @brief Declare a global ble service variable here.
 */
ble_service_t ble_service;

/**
 * @brief We need a forward declaration of the ble event handler before 
 *        registering the observer.
 */
static void ble_event_handler(ble_evt_t const *p_ble_evt, void *p_context);

/**
* @brief This macro registers the Bluetooth service event handler to the stack.
*/
NRF_SDH_BLE_OBSERVER(ble_service_obs, 2, ble_event_handler, &ble_service);

/**
 * @brief Clock event callback. Not used but we need to have it.
 */
void clock_event_handler(nrfx_clock_evt_type_t event)
{
    // We don't need the event variable so we can void it
    (void)event;
}

/**
 * @brief Starts the SPI to the FPGA for downloading camera data.
 */
void bluetooth_app_init_fpga_spi(void)
{
    nrfx_spim_config_t spi_config = NRFX_SPIM_DEFAULT_CONFIG;

    spi_config.mosi_pin = SPI_SO_PIN;
    spi_config.miso_pin = SPI_SI_PIN;
    spi_config.sck_pin = SPI_CLK_PIN;
    spi_config.ss_pin = SPI_CS_PIN;

    nrfx_spim_t spi = NRFX_SPIM_INSTANCE(0);

    APP_ERROR_CHECK(nrfx_spim_init(&spi, &spi_config, NULL, NULL));
}

/**
 * @brief This function retrieves the image data from the FPGA over SPI.
 */
void bluetooth_app_get_camera_data(void)
{
    nrfx_spim_xfer_desc_t spi_xfer = NRFX_SPIM_XFER_RX(&image_buffer,
                                                       sizeof(image_buffer));

    nrfx_spim_t spi = NRFX_SPIM_INSTANCE(0);

    APP_ERROR_CHECK(nrfx_spim_xfer(&spi, &spi_xfer, 0));
}

/**
 * @brief This function creates some dummy data which can be used for testing.
 *        It populates the image_buffer with float values of 0.1, 0.2, 0.3, and
 *        so on.
 */
static void bluetooth_app_form_dummy_data(void)
{
    for (uint16_t i = 0; i < 768; i++)
    {
        float dummy_float = 0.1f * i;

        uint8_t bytes[4];
        uint8_t reversed_bytes[4];

        memcpy(&bytes, &dummy_float, sizeof(float));

        reversed_bytes[0] = bytes[3];
        reversed_bytes[1] = bytes[2];
        reversed_bytes[2] = bytes[1];
        reversed_bytes[3] = bytes[0];

        memcpy(&image_buffer[i * 4], &reversed_bytes, 4);
    }
}

/**
 * @brief Pushes a chunk of camera frame data over bluetooth. Size of chunk is 
 *        always negotiated_mtu_size.
 * 
 * @param packet_id: A number which determines if this is the first, mid or last 
 *                   packet of the full image transfer. 0 means first, 1 means
 *                   middle, and 2 means last.
 * 
 * @param data: Pointer to the data buffer.
 * 
 * @param len: Length of the data to send.
 * 
 * @param offset: What offset to send data from in the image_buffer.
 */
void bluetooth_app_send_data_to_web_app(uint8_t packet_id,
                                        uint8_t *data,
                                        uint16_t length,
                                        uint16_t offset)
{
    uint8_t payload[NRF_SDH_BLE_GATT_MAX_MTU_SIZE];

    payload[0] = packet_id;

    memcpy(&payload[1], &data[offset], length);

    // Add one to length because we added the packet ID byte
    length += 1;

    ble_gatts_hvx_params_t hvx_params = {0};
    hvx_params.handle = ble_service.char_handles.value_handle;
    hvx_params.p_data = (uint8_t *)&payload;
    hvx_params.p_len = &length;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

    sd_ble_gatts_hvx(ble_service.conn_handle, &hvx_params);
}

/**
 * @brief GATT event handler. Only used for changing MTU payload size.
 */
static void gatt_event_handler(nrf_ble_gatt_t *p_gatt,
                               nrf_ble_gatt_evt_t const *p_evt)
{
    // We don't need the GATT pointer so we can void it
    (void)p_gatt;

    // If we receive an MTU update request, we can update the the MTU size
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        LOG("MTU length set to: %d bytes", p_evt->params.att_mtu_effective);
        negotiated_mtu_size = p_evt->params.att_mtu_effective - 3;
    }
}

/**
 * @brief The advertising event handler that informs us which advertising mode
 *        we are in.
 */
static void advertising_event_handler(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
    // We only use fast advertising in this example, but you can handle other
    // states too
    case BLE_ADV_EVT_FAST:
        LOG("Fast advertising");
        break;

    case BLE_ADV_EVT_SLOW:
    case BLE_ADV_EVT_IDLE:
    case BLE_ADV_EVT_DIRECTED_HIGH_DUTY:
    case BLE_ADV_EVT_DIRECTED:
    case BLE_ADV_EVT_FAST_WHITELIST:
    case BLE_ADV_EVT_SLOW_WHITELIST:
    case BLE_ADV_EVT_WHITELIST_REQUEST:
    case BLE_ADV_EVT_PEER_ADDR_REQUEST:
        LOG("Unused advertising state");
        break;
    }
}

/**
 * @brief Bluetooth event handler which informs us of connection status, new
 *        data, PHY update, as well as other things.
 */
static void ble_event_handler(ble_evt_t const *p_ble_evt, void *p_context)
{

    ble_service_t *p_cus = (ble_service_t *)p_context;

    // Good practice to make sure the above pointers are valid
    if (p_cus == NULL || p_ble_evt == NULL)
    {
        LOG("BLE event handler called with invalid pointers");
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {

    // When connected, set the connection handler
    case BLE_GAP_EVT_CONNECTED:
    {
        LOG("Connected");
        p_cus->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        break;
    }

    // When disconnected, we reset the connection handler
    case BLE_GAP_EVT_DISCONNECTED:
    {
        LOG("Disconnected");
        p_cus->conn_handle = BLE_CONN_HANDLE_INVALID;
        break;
    }

    // On a phy update request, we set the phy speed automatically
    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    {

        LOG("PHY update request");
        ble_gap_phys_t const phys = {
            .rx_phys = BLE_GAP_PHY_AUTO,
            .tx_phys = BLE_GAP_PHY_AUTO,
        };
        APP_ERROR_CHECK(sd_ble_gap_phy_update(
            p_ble_evt->evt.gap_evt.conn_handle,
            &phys));
        break;
    }

    // Once the notification is enabled, or data is sent, we can write data
    case BLE_GATTS_EVT_WRITE:
    case BLE_GATTS_EVT_HVN_TX_COMPLETE:
    {
        // TODO check if the endpoint is set to notifications correctly
        bluetooth_ready = true;
        break;
    }

    // Disconnect on GATT Client timeout
    case BLE_GATTC_EVT_TIMEOUT:
    {
        LOG("GATT client Timeout");
        APP_ERROR_CHECK(sd_ble_gap_disconnect(
            p_ble_evt->evt.gattc_evt.conn_handle,
            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION));

        break;
    }

    // Disconnect on GATT Server timeout
    case BLE_GATTS_EVT_TIMEOUT:
    {
        LOG("GATT server Timeout");
        APP_ERROR_CHECK(sd_ble_gap_disconnect(
            p_ble_evt->evt.gatts_evt.conn_handle,
            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION));

        break;
    }

    // Updates system attributes after a new connection event
    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
    {
        LOG("Updating system attributes");
        APP_ERROR_CHECK(sd_ble_gatts_sys_attr_set(
            p_ble_evt->evt.gatts_evt.conn_handle,
            NULL,
            0,
            0));
        break;
    }

    default:
    {
        LOG("Unused BLE event with ID: %d", p_ble_evt->header.evt_id);
        break;
    }
    }
}

static void bluetooth_app_timer_handler(void *p_context)
{
    // We don't need the context pointer so we void it
    (void)p_context;

    switch (bluetooth_app_state)
    {
    // Boot up the FPGA using image already stored in the Flash IC
    case BOOT_FPGA:
    {
        LOG("Booting FPGA");
        s1_fpga_boot();
        bluetooth_app_form_dummy_data();
        bluetooth_app_state = INIT_SPI;
        break;
    }

    // Wait for boot, and then initialize the SPI bus
    case INIT_SPI:
    {
        // if (s1_fpga_is_booted()) // TODO uncomment me!!
        {
            LOG("FPGA app started");
            bluetooth_app_init_fpga_spi();
            bluetooth_app_state = READ_SPI_DATA;
        }
    }
    break;

    // Wait for an interrupt signal on FPGA_DONE_PIN before reading SPI data
    case READ_SPI_DATA:
    {
        // if (s1_fpga_is_booted()) // TODO enable this in the FPGA code
        {
            // bluetooth_app_get_camera_data();
            chunks_sent = 0;
            bluetooth_app_state = SEND_PACKETS;
        }

        break;
    }

    case SEND_PACKETS:
    {
        if (bluetooth_ready)
        {
            uint8_t total_chunks = (uint8_t)ceil((float)sizeof(image_buffer) /
                                                 (float)negotiated_mtu_size);

            uint8_t id_flag;

            // Set ID flag according to how many chunks have been sent
            if (chunks_sent == 0)
            {
                id_flag = 0;
            }
            else if (chunks_sent == total_chunks - 1)
            {
                id_flag = 2;
            }
            else
            {
                id_flag = 1;
            }

            // Send data from image_buffer. Length is -1 because of the ID byte
            bluetooth_app_send_data_to_web_app(id_flag,
                                               (uint8_t *)&image_buffer,
                                               negotiated_mtu_size - 1,
                                               (uint16_t)(chunks_sent * (negotiated_mtu_size - 1)));

            // Increment chunk counter
            chunks_sent++;

            // Go back to waiting for interrupt when done
            if (chunks_sent == total_chunks)
            {
                bluetooth_app_state = READ_SPI_DATA;
            }

            // Wait until this flag before run again
            bluetooth_ready = false;
        }
        break;
    }
    }
}

/**
 * @brief Main application portion of the Bluetooth application. This is largely
 *        configuration and starting the bluetooth. At the bottom the
 *        application state machine is started and control is handed to the
 *        scheduler. This cannot run when the code is build with the FPGA
 *        flasher option.
 */
void main_bluetooth_app(void)
{
    LOG("Started bluetooth application.");

    // BLE stack init
    {
        APP_ERROR_CHECK(nrf_sdh_enable_request());

        // Configure the BLE stack using the default settings.
        // Fetch the start address of the application RAM.
        uint32_t ram_start = 0;
        APP_ERROR_CHECK(nrf_sdh_ble_default_cfg_set(1, &ram_start));

        // Enable BLE stack.
        APP_ERROR_CHECK(nrf_sdh_ble_enable(&ram_start));

        // This will tell you how much RAM you need to configure in the linker
        // script. Set the value there accordingly. If you get a crash before
        // this, it means the ram is too low.
        LOG("Softdevice RAM usage: 0x%x", ram_start);
    }

    // GAP parameters init
    {
        ble_gap_conn_sec_mode_t sec_mode;
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

        char device_name[25] = "";

        snprintf(device_name, 25, "S1 Thermal Cam Demo %X",
                 (uint16_t)NRF_FICR->DEVICEADDR[0]);

        APP_ERROR_CHECK(sd_ble_gap_device_name_set(&sec_mode,
                                                   (const uint8_t *)device_name,
                                                   (uint16_t)strlen((const char *)device_name)));

        ble_gap_conn_params_t gap_conn_params = {0};
        gap_conn_params.min_conn_interval = MSEC_TO_UNITS(30, UNIT_1_25_MS);
        gap_conn_params.max_conn_interval = MSEC_TO_UNITS(45, UNIT_1_25_MS);
        gap_conn_params.slave_latency = 0;
        gap_conn_params.conn_sup_timeout = MSEC_TO_UNITS(4000, UNIT_10_MS);

        APP_ERROR_CHECK(sd_ble_gap_ppcp_set(&gap_conn_params));
    }

    // GATT init
    {
        APP_ERROR_CHECK(nrf_ble_gatt_init(&m_gatt, gatt_event_handler));
    }

    // Services init
    {
        // Reset the connection handle because we're about to set stuff
        ble_service.conn_handle = BLE_CONN_HANDLE_INVALID;

        // Add Custom Service UUID
        ble_uuid128_t base_uuid = {SILICONWITCHERY_BASE_UUID};
        APP_ERROR_CHECK(sd_ble_uuid_vs_add(&base_uuid, &ble_service.uuid_type));

        ble_uuid_t ble_uuid;
        ble_uuid.type = ble_service.uuid_type;
        ble_uuid.uuid = SERVICE_UUID;

        APP_ERROR_CHECK(sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                                 &ble_uuid,
                                                 &ble_service.service_handle));

        // Add the camera data characteristic
        ble_add_char_params_t add_char_params = {0};

        add_char_params.uuid = CHARACTERISTIC_UUID;
        add_char_params.uuid_type = ble_service.uuid_type;
        add_char_params.max_len = NRF_SDH_BLE_GATT_MAX_MTU_SIZE - 3;
        add_char_params.init_len = sizeof(uint8_t);
        add_char_params.is_var_len = true;
        add_char_params.char_props.notify = 1;
        add_char_params.read_access = SEC_OPEN;
        add_char_params.cccd_write_access = SEC_OPEN;

        APP_ERROR_CHECK(characteristic_add(ble_service.service_handle,
                                           &add_char_params,
                                           &ble_service.char_handles));
    }

    // Advertising init
    {
        // UUID which will go into the advertising packet
        static ble_uuid_t adv_uuids[1];
        adv_uuids[0].uuid = SERVICE_UUID;
        adv_uuids[0].type = BLE_UUID_TYPE_VENDOR_BEGIN;

        ble_advertising_init_t init = {0};
        init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

        // Fast advertise forever
        init.config.ble_adv_fast_enabled = true;
        init.config.ble_adv_fast_interval = MSEC_TO_UNITS(20, UNIT_0_625_MS);
        init.config.ble_adv_fast_timeout = 0;

        // Advertise full name in srdata
        init.srdata.name_type = BLE_ADVDATA_FULL_NAME;

        // Advertise the custom service UUID
        init.advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
        init.advdata.uuids_complete.p_uuids = adv_uuids;

        // This is the callback handler
        init.evt_handler = advertising_event_handler;

        APP_ERROR_CHECK(ble_advertising_init(&m_advertising, &init));

        // // Start advertising
        ble_advertising_conn_cfg_tag_set(&m_advertising, 1);
        APP_ERROR_CHECK(ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST));
    }

    // Connection parameters init
    {
        ble_conn_params_init_t cp_init = {0};
        cp_init.p_conn_params = NULL;
        cp_init.first_conn_params_update_delay = APP_TIMER_TICKS(5000);
        cp_init.next_conn_params_update_delay = APP_TIMER_TICKS(30000);
        cp_init.max_conn_params_update_count = 3;
        cp_init.disconnect_on_fail = true;
        cp_init.evt_handler = NULL;
        cp_init.error_handler = NULL;

        APP_ERROR_CHECK(ble_conn_params_init(&cp_init));
    }

    // Create 1ms state machine timer for booting the FPGA and pulling data
    APP_ERROR_CHECK(app_timer_create(&bluetooth_app_task,
                                     APP_TIMER_MODE_REPEATED,
                                     bluetooth_app_timer_handler));

    APP_ERROR_CHECK(app_timer_start(bluetooth_app_task,
                                    APP_TIMER_TICKS(1),
                                    NULL));

    // Initialise power management
    APP_ERROR_CHECK(nrf_pwr_mgmt_init());

    // The CPU is free to do nothing in the meanwhile
    while (1)
    {
        app_sched_execute();
        nrf_pwr_mgmt_run();
    }
}

/**
 * @brief Timer based state machine for flashing the FPGA image and booting the 
 *        FPGA. As some of the flash operations take a lot of time, using a 
 *        timer based state machine avoids the main thread hanging while waiting
 *        for flash operations to complete.
 */
static void fpga_flasher_timer_handler(void *p_context)
{
    // We don't need the context pointer
    (void)p_context;

    switch (fpga_flasher_state)
    {
    // Configure power and erase the flash
    case STARTED:
    {
        LOG("Erasing flash.");
        s1_flash_erase_all();
        fpga_flasher_state = ERASING;
        break;
    }

    // Wait for erase to complete
    case ERASING:
    {
        if (!s1_flash_is_busy())
        {
            flash_pages_remaining = (uint32_t)ceil((float)fpga_binfile_bin_len / 256.0f);
            fpga_flasher_state = FLASHING;
            LOG("Flashing %d pages.", flash_pages_remaining);
        }
        break;
    }

    // Flash every page until done
    case FLASHING:
    {
        if (!s1_flash_is_busy())
        {
            s1_flash_page_from_image(flash_page_address, (unsigned char *)&fpga_binfile_bin);
            flash_pages_remaining--;
            flash_page_address += 0x100;
        }

        if (flash_pages_remaining == 0)
        {
            fpga_flasher_state = DONE;
            s1_fpga_boot();
            LOG("Flashing done.");
            break;
        }
        break;
    }

    // Stop the timer. We are done
    case DONE:
    {
        app_timer_stop(fpga_flasher_task);
        break;
    }
    }
}

/**
 * @brief Main application portion of the FPGA flasher. This is optimized out
 *        when building the Bluetooth application.
 */
void main_fpga_flasher_app(void)
{
    LOG("Started FPGA flash application.");

    // Initialise LFXO required by the App Timer
    APP_ERROR_CHECK(nrfx_clock_init(clock_event_handler));
    nrfx_clock_lfclk_start();

    // Create a timer event for the FPGA binary flashing process
    APP_ERROR_CHECK(app_timer_create(&fpga_flasher_task,
                                     APP_TIMER_MODE_REPEATED,
                                     fpga_flasher_timer_handler));

    // Start the task
    APP_ERROR_CHECK(app_timer_start(fpga_flasher_task,
                                    APP_TIMER_TICKS(1),
                                    NULL));

    // Initialise power management
    APP_ERROR_CHECK(nrf_pwr_mgmt_init());

    // Run the scheduler forever
    while (1)
    {
        app_sched_execute();
        nrf_pwr_mgmt_run();
    }
}

/**
 * @brief Main application entry.
 */
int main(void)
{
    // View logs using the Segger JLinkRTTClient application
    LOG_CLEAR();
    LOG("S1 thermal camera demo – Built: %s %s – SDK Version: %s.",
        __DATE__,
        __TIME__,
        __S1_SDK_VERSION__);

    // Initialize the S1 base settings
    APP_ERROR_CHECK(s1_init());

    // Configuration power rail settings
    s1_pimc_fpga_vcore(true);
    s1_pmic_set_vio(2.8f);
    s1_pmic_set_vaux(3.3f);

    // Reset the FPGA and wake up the flash IC
    s1_fpga_hold_reset();
    s1_flash_wakeup();

    // Initialize the timer module
    APP_ERROR_CHECK(app_timer_init());

    // Initialize the scheduler
    APP_SCHED_INIT(APP_TIMER_SCHED_EVENT_DATA_SIZE, 10);

#ifdef BLUETOOTH_ENABLED
    // If we're in normal operating mode, run the bluetooth side application
    main_bluetooth_app();
#else
    // Otherwise run the FPGA flasher
    main_fpga_flasher_app();
#endif
}