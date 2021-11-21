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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

uint8_t data_response_pending_flag[1] = {0};

/**
 * @brief The Bluetooth base UUID for the camera data service
 */
#define SILICONWITCHERY_BASE_UUID                                                                          \
    {                                                                                                      \
        {                                                                                                  \
            0xab, 0x5a, 0x11, 0x37, 0xc5, 0xbf, 0x4d, 0xe8, 0x9e, 0xfe, 0x4d, 0x18, 0x81, 0x04, 0xfd, 0x48 \
        }                                                                                                  \
    }

/**
 * @brief This value is appended to the Bluetooth base UUID and becomes the 
 * camera service UUID.
 */
#define SERVICE_UUID 0x0001

/**
 * @brief This value is appended to the Bluetooth base UUID and becomes the
 * camera characteristic UUID.
 */
#define CHARACTERISTIC_UUID 0x0003

/**
 * @brief The payload size is negotiated by the receiving device. This variable
 *        keeps track of that size so we can break the camera data into chunks
 *        according to this size.
 */
uint16_t max_negotiated_buffer_size = 512 - 3;

/**
 * @brief This struct holds the connection handles for our camera service.
 */
struct ble_service
{
    uint8_t uuid_type;
    uint16_t service_handle;
    ble_gatts_char_handles_t char_handles;
    uint16_t conn_handle;
} ble_service;

/**
 * @brief This large buffer stores the latest camera frame data
 */
static uint8_t image_buffer[1536];

/**
 * @brief Clock event callback. Not used but we need to have it.
 */
void clock_event_handler(nrfx_clock_evt_type_t event)
{
    (void)event;
}

/**
 * @brief Restarts the SPI lines to receive data from the FPGA.
 */
void start_image_rx_bus(void)
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
 * @brief This function retrieves the image data from the FPGA over SPI
 */
void get_image(void)
{
    nrfx_spim_xfer_desc_t spi_xfer = NRFX_SPIM_XFER_RX(&image_buffer, 1536);

    nrfx_spim_t spi = NRFX_SPIM_INSTANCE(0);

    APP_ERROR_CHECK(nrfx_spim_xfer(&spi, &spi_xfer, 0));
}

/**
 * @brief GATT event handler. Only used for changing MTU payload size.
 */
static void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt)
{
    // (void)p_gatt;

    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        LOG("MTU set to %d.", p_evt->params.att_mtu_effective);
        max_negotiated_buffer_size = p_evt->params.att_mtu_effective - 3;
    }
}

/**
 * @brief Advertising event handler that informs us which advertising mode we
 *        are in.
 */
static void on_adv_evt_handler(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
    case BLE_ADV_EVT_FAST:
        LOG("[Info] Fast advertising.");
        break;

    case BLE_ADV_EVT_SLOW:
        LOG("[Info] Slow advertising.");
        break;

    case BLE_ADV_EVT_IDLE:
        LOG("[Info] Advertising idle.");
        break;

    case BLE_ADV_EVT_DIRECTED_HIGH_DUTY:
    case BLE_ADV_EVT_DIRECTED:
    case BLE_ADV_EVT_FAST_WHITELIST:
    case BLE_ADV_EVT_SLOW_WHITELIST:
    case BLE_ADV_EVT_WHITELIST_REQUEST:
    case BLE_ADV_EVT_PEER_ADDR_REQUEST:
        LOG("[Info] Advertising other status.");
        break;
    }
}

/**
 * @brief Error handler for bad connection parameters.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**
 * @brief Bluetooth event handler which informs us of connection status as well
 *        as updating PHY and a few other things.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{
    // (void)p_context;

    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        LOG("Connected");
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        LOG("Disconnected");
        break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    {
        LOG("PHY update request.");
        ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_1MBPS,
                .tx_phys = BLE_GAP_PHY_1MBPS,
            };
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(err_code);
    }
    break;

    // Disconnect on GATT Client timeout event.
    case BLE_GATTC_EVT_TIMEOUT:
        LOG("GATT Client Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    // Disconnect on GATT Server timeout event.
    case BLE_GATTS_EVT_TIMEOUT:
        LOG("GATT Server Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        LOG("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
        break;

    case BLE_GAP_EVT_AUTH_KEY_REQUEST:
        LOG("BLE_GAP_EVT_AUTH_KEY_REQUEST");
        break;

    case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
        LOG("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
        break;

    case BLE_GAP_EVT_AUTH_STATUS:
        LOG("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
            p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
            p_ble_evt->evt.gap_evt.params.auth_status.bonded,
            p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
            *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
            *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
        break;
    }
}

/**
 * @brief Bluetooth service event handler. Sets flags for when the service is
 *        up, and allows us to handle incoming data if we like.
 */
void ble_service_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        ble_service.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        ble_service.conn_handle = BLE_CONN_HANDLE_INVALID;
        break;
    }
}

/**
* @brief This macro registers the Bluetooth service event handler to the stack.
*/
NRF_SDH_BLE_OBSERVER(blue_service_obs, 2, ble_service_evt_handler, &ble_service);

/**
 * @brief Pushes chunk of a camera frame over bluetooth when called
 */
void send_camera_data(void)
{
    ble_gatts_hvx_params_t hvx_params = {0};

    if (ble_service.conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return;
    }

    uint16_t length = sizeof(image_buffer);

    hvx_params.handle = ble_service.char_handles.value_handle;
    hvx_params.p_data = (uint8_t *)&image_buffer;
    hvx_params.p_len = &length; // TODO this is too big
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

    sd_ble_gatts_hvx(ble_service.conn_handle, &hvx_params);
}

static void bluetooth_app_timer_handler(void *p_context)
{
    // We don't need the context pointer
    (void)p_context;

    LOG("Hi!");
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
    ret_code_t err_code;

    LOG("Started Bluetooth application.");

    // BLE stack init
    {
        err_code = nrf_sdh_enable_request();
        APP_ERROR_CHECK(err_code);

        // Configure the BLE stack using the default settings.
        // Fetch the start address of the application RAM.
        uint32_t ram_start = 0;
        err_code = nrf_sdh_ble_default_cfg_set(1, &ram_start);
        APP_ERROR_CHECK(err_code);

        // Enable BLE stack.
        err_code = nrf_sdh_ble_enable(&ram_start);
        APP_ERROR_CHECK(err_code);

        // Register a handler for BLE events.
        NRF_SDH_BLE_OBSERVER(m_ble_observer, 3, ble_evt_handler, NULL);
    }

    // GAP parameters init
    {
        ble_gap_conn_sec_mode_t sec_mode;

        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

        char device_name[20] = "";

        snprintf(device_name,
                 20,
                 "SuperStack %X",
                 (uint16_t)NRF_FICR->DEVICEADDR[0]);

        err_code = sd_ble_gap_device_name_set(&sec_mode,
                                              (const uint8_t *)device_name,
                                              strlen((const char *)device_name));
        APP_ERROR_CHECK(err_code);

        err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_WATCH);
        APP_ERROR_CHECK(err_code);

        ble_gap_conn_params_t gap_conn_params;
        memset(&gap_conn_params, 0, sizeof(gap_conn_params));

        gap_conn_params.min_conn_interval = MSEC_TO_UNITS(30, UNIT_1_25_MS);
        gap_conn_params.max_conn_interval = MSEC_TO_UNITS(45, UNIT_1_25_MS);
        gap_conn_params.slave_latency = 0;
        gap_conn_params.conn_sup_timeout = MSEC_TO_UNITS(4000, UNIT_10_MS);

        err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
        APP_ERROR_CHECK(err_code);
    }

    // GATT init
    {
        err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
        APP_ERROR_CHECK(err_code);
    }

    // Services init
    {
        ble_uuid_t ble_uuid;
        ble_uuid128_t base_uuid = SILICONWITCHERY_BASE_UUID;
        ble_add_char_params_t add_char_params;

        // Set up UUID
        err_code = sd_ble_uuid_vs_add(&base_uuid, &ble_service.uuid_type);
        APP_ERROR_CHECK(err_code);

        ble_uuid.type = ble_service.uuid_type;
        ble_uuid.uuid = SERVICE_UUID;

        // Add the service
        err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                            &ble_uuid,
                                            &ble_service.service_handle);
        APP_ERROR_CHECK(err_code);

        // Add the camera data characteristic
        memset(&add_char_params, 0, sizeof(add_char_params));
        add_char_params.uuid = CHARACTERISTIC_UUID;
        add_char_params.uuid_type = ble_service.uuid_type;
        add_char_params.max_len = NRF_SDH_BLE_GATT_MAX_MTU_SIZE - 3;
        add_char_params.init_len = sizeof(uint8_t);
        add_char_params.is_var_len = true;
        add_char_params.char_props.notify = 1;
        add_char_params.read_access = SEC_OPEN;
        add_char_params.cccd_write_access = SEC_OPEN;

        err_code = characteristic_add(ble_service.service_handle, &add_char_params, &ble_service.char_handles);
        APP_ERROR_CHECK(err_code);

        // Init connection handle
        ble_service.conn_handle = BLE_CONN_HANDLE_INVALID;
    }

    // Advertising init
    {
        // UUID which will go into the advertising packet
        static ble_uuid_t adv_uuids[1];
        adv_uuids[0].uuid = SERVICE_UUID;
        adv_uuids[0].type = BLE_UUID_TYPE_VENDOR_BEGIN;

        ble_advertising_init_t init;
        memset(&init, 0, sizeof(init));

        // Name has to go into scan response packet
        init.srdata.name_type = BLE_ADVDATA_FULL_NAME;

        // UUID can go into main advertising packet
        init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
        init.advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
        init.advdata.uuids_complete.p_uuids = adv_uuids;

        // Data pending flag can go into main advertising packet
        ble_advdata_manuf_data_t manuf_data;
        manuf_data.company_identifier = 0xFFFF;
        manuf_data.data.p_data = data_response_pending_flag;       // TODO: Needed? this was a flag before
        manuf_data.data.size = sizeof(data_response_pending_flag); // Also set this size
        init.advdata.p_manuf_specific_data = &manuf_data;

        // Only use fast advertising forever
        init.config.ble_adv_fast_enabled = true;
        init.config.ble_adv_fast_interval = MSEC_TO_UNITS(20, UNIT_0_625_MS); // was 760
        init.config.ble_adv_fast_timeout = 0;

        init.evt_handler = on_adv_evt_handler;

        err_code = ble_advertising_init(&m_advertising, &init);
        APP_ERROR_CHECK(err_code);

        ble_advertising_conn_cfg_tag_set(&m_advertising, 1);
    }

    // Connection parameters init
    {
        ble_conn_params_init_t cp_init;

        memset(&cp_init, 0, sizeof(cp_init));

        cp_init.p_conn_params = NULL;
        cp_init.first_conn_params_update_delay = APP_TIMER_TICKS(5000); // Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds)
        cp_init.next_conn_params_update_delay = APP_TIMER_TICKS(30000); // Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds)
        cp_init.max_conn_params_update_count = 3;
        cp_init.disconnect_on_fail = true;
        cp_init.evt_handler = NULL;
        cp_init.error_handler = conn_params_error_handler;

        err_code = ble_conn_params_init(&cp_init);
        APP_ERROR_CHECK(err_code);
    }

    // Create 1ms state machine timer for booting the FPGA and pulling data
    APP_ERROR_CHECK(app_timer_create(&bluetooth_app_task,
                                     APP_TIMER_MODE_REPEATED,
                                     bluetooth_app_timer_handler));

    // APP_ERROR_CHECK(app_timer_start(bluetooth_app_task,
    //                                 APP_TIMER_TICKS(5000),
    //                                 NULL));

    // Start advertising
    APP_ERROR_CHECK(ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST));

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
 * @brief Timer based state machine for flashing the FPGA
 *        image and booting the FPGA. As some of the flash
 *        operations take a lot of time, using a timer based
 *        state machine avoids the main thread hanging while
 *        waiting for flash operations to complete.
 */
static void fpga_flasher_timer_handler(void *p_context)
{
    // We don't need the context pointer
    (void)p_context;

    switch (fpga_flasher_state)
    {
    // Configure power and erase the flash
    case STARTED:
        LOG("Erasing flash.");
        s1_flash_erase_all();
        fpga_flasher_state = ERASING;
        break;

    // Wait for erase to complete
    case ERASING:
        if (!s1_flash_is_busy())
        {
            flash_pages_remaining = (uint32_t)ceil((float)fpga_binfile_bin_len / 256.0f);
            fpga_flasher_state = FLASHING;
            LOG("Flashing %d pages.", flash_pages_remaining);
        }
        break;

    // Flash every page until done
    case FLASHING:
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

    // Stop the timer. We are done.
    case DONE:
        app_timer_stop(fpga_flasher_task);
        break;
        // // Wait for 1 second before reading back data§
        // case WAIT_FOR_DATA:

        //     start_image_rx_bus();

        //     NRFX_DELAY_US(100000);

        //     fpga_boot_state = DUMP_SPI;
        //     break;

        // // Dump data from FPGA over SPI
        // case DUMP_SPI:
        //     get_image();
        //     for (uint32_t i = 0; i < 1536; i++)
        //     {
        //         LOG_RAW("0x%x, ", image_buffer[i]);
        //     }
        //     NRFX_DELAY_US(100000);

        //     break;
    }
}

/**
 * @brief Main application portion of the FPGA flasher. This is optimised out
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
 * @brief Main application entry for the fpga-blinky demo.
 */
int main(void)
{
    // Log some stuff about this project
    LOG_CLEAR();
    LOG("S1 Thermal camera demo – Built: %s %s – SDK Version: %s.",
        __DATE__,
        __TIME__,
        __S1_SDK_VERSION__);

    // Initialise the S1 base settings
    APP_ERROR_CHECK(s1_init());

    // Initialise power and configuration settings
    s1_pimc_fpga_vcore(true);
    s1_pmic_set_vio(2.8f);
    s1_pmic_set_vaux(3.3f);
    s1_fpga_hold_reset();
    s1_flash_wakeup();

    // Init the timer module
    APP_ERROR_CHECK(app_timer_init());

    // Initialise the scheduler
    APP_SCHED_INIT(APP_TIMER_SCHED_EVENT_DATA_SIZE, 10);

#ifdef BLUETOOTH_ENABLED
    // If we're in normal operating mode, run the bluetooth application main()
    main_bluetooth_app();
#else
    // Otherwise run the FPGA flasher
    main_fpga_flasher_app();
#endif
}

// Softdevice error handler. Set flag and reset
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
    LOG("[ERROR] BLE stack error at %s:%d", p_file_name, line_num);
    nrf_delay_ms(500);
    NRF_BREAKPOINT_COND;
    NVIC_SystemReset();
}

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    switch (id)
    {
    case NRF_FAULT_ID_SDK_ASSERT:
    {
        assert_info_t *p_info = (assert_info_t *)info;
        LOG("ASSERTION FAILED at %s:%u",
            p_info->p_file_name,
            p_info->line_num);
        break;
    }
    case NRF_FAULT_ID_SDK_ERROR:
    {
        error_info_t *p_info = (error_info_t *)info;
        LOG("ERROR %u [%s] at %s:%u\r\nPC at: 0x%08x",
            p_info->err_code,
            nrf_strerror_get((ret_code_t)p_info->err_code),
            p_info->p_file_name,
            p_info->line_num,
            pc);
        break;
    }
    default:
        LOG("Other error at 0x%08X", pc);
        break;
    }

    NRF_BREAKPOINT_COND;
}

// Hardfault handler. Set flag and reset
void HardFault_Handler(void)
{
    LOG("[ERROR] CPU hardfault");
    nrf_delay_ms(500);
    NRF_BREAKPOINT_COND;
    NVIC_SystemReset();
}