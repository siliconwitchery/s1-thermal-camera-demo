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

/**
 * @brief The Bluetooth base UUID for the camera data service
 */
#define SILICONWITCHERY_BASE_UUID                                                                      \
    {                                                                                                  \
        0xab, 0x5a, 0x11, 0x37, 0xc5, 0xbf, 0x4d, 0xe8, 0x9e, 0xfe, 0x4d, 0x18, 0x81, 0x04, 0xfd, 0x48 \
    }

/**
 * @brief This value is appended to the Bluetooth base UUID and becomes the 
 * camera service UUID.
 */
#define SERVICE_UUID 0x1001

/**
 * @brief This value is appended to the Bluetooth base UUID and becomes the
 * camera characteristic UUID.
 */
#define CHARACTERISTIC_UUID 0x1002

/**
 * @brief The payload size is negotiated by the receiving device. This variable
 *        keeps track of that size so we can break the camera data into chunks
 *        according to this size.
 */
uint16_t max_negotiated_buffer_size = 512 - 3;

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
 * @brief Declare a global ble service variable here.
 */
ble_service_t ble_service;

/**
 * @brief We need a forward declaration of the ble event handler before 
 *        registering the observer.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context);

/**
* @brief This macro registers the Bluetooth service event handler to the stack.
*/
NRF_SDH_BLE_OBSERVER(ble_service_obs, 2, ble_evt_handler, &ble_service);

/**
 * @brief This large buffer stores the latest camera frame data
 */
static uint8_t image_buffer[1536] = "hello there";

/**
 * @brief Clock event callback. Not used but we need to have it.
 */
void clock_event_handler(nrfx_clock_evt_type_t event)
{
    // (void)event;
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
 * @brief Bluetooth event handler which informs us of connection status, new
 *        data, PHY update, as well as other things.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{

    LOG("BLE Event: %d", p_ble_evt->header.evt_id);

    ble_service_t *p_cus = (ble_service_t *)p_context;

    if (p_cus == NULL || p_ble_evt == NULL)
    {
        LOG("NULL");
        return;
    }

    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        LOG("Connected");
        p_cus->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        LOG("Disconnected");
        p_cus->conn_handle = BLE_CONN_HANDLE_INVALID;
        break;

    case BLE_GATTS_EVT_WRITE:
        LOG("Write event");
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

    case BLE_GATTC_EVT_TIMEOUT:
        // Disconnect on GATT Client timeout event.
        LOG("GATT Client Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GATTS_EVT_TIMEOUT:
        // Disconnect on GATT Server timeout event.
        LOG("GATT Server Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        // No system attributes have been stored.
        err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.gatts_evt.conn_handle, NULL, 0, 0);
        APP_ERROR_CHECK(err_code);
        break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

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

    default:
        // No implementation needed.
        break;
    }
}

/**
 * @brief Pushes chunk of a camera frame over bluetooth when called
 */
void send_camera_data(void)
{

    // TODO only send data if the connection is valid. Some reason this crashes

    ble_gatts_hvx_params_t hvx_params = {0};

    if (ble_service.conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return;
    }

    // TODO if this is too big
    uint16_t length = 10; // sizeof(image_buffer);

    hvx_params.handle = ble_service.char_handles.value_handle;
    hvx_params.p_data = (uint8_t *)&image_buffer;
    hvx_params.p_len = &length;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

    APP_ERROR_CHECK(sd_ble_gatts_hvx(ble_service.conn_handle, &hvx_params));
}

static void bluetooth_app_timer_handler(void *p_context)
{
    // We don't need the context pointer
    // (void)p_context;

    LOG("Hi!");
    send_camera_data();
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
    LOG("Started Bluetooth application.");

    // BLE stack init
    {
        APP_ERROR_CHECK(nrf_sdh_enable_request());

        // Configure the BLE stack using the default settings.
        // Fetch the start address of the application RAM.
        uint32_t ram_start = 0;
        APP_ERROR_CHECK(nrf_sdh_ble_default_cfg_set(1, &ram_start));

        // Enable BLE stack.
        APP_ERROR_CHECK(nrf_sdh_ble_enable(&ram_start));

        // Register a handler for BLE events.
        // TODO determine if we need this here
        // NRF_SDH_BLE_OBSERVER(m_ble_observer, 3, ble_evt_handler, NULL);
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
                                                   strlen((const char *)device_name)));

        ble_gap_conn_params_t gap_conn_params = {0};
        gap_conn_params.min_conn_interval = MSEC_TO_UNITS(30, UNIT_1_25_MS);
        gap_conn_params.max_conn_interval = MSEC_TO_UNITS(45, UNIT_1_25_MS);
        gap_conn_params.slave_latency = 0;
        gap_conn_params.conn_sup_timeout = MSEC_TO_UNITS(4000, UNIT_10_MS);

        APP_ERROR_CHECK(sd_ble_gap_ppcp_set(&gap_conn_params));
    }

    // GATT init
    {
        APP_ERROR_CHECK(nrf_ble_gatt_init(&m_gatt, gatt_evt_handler));
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
        init.evt_handler = on_adv_evt_handler;

        APP_ERROR_CHECK(ble_advertising_init(&m_advertising, &init));

        // // Start advertising
        ble_advertising_conn_cfg_tag_set(&m_advertising, 1);
        APP_ERROR_CHECK(ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST));
    }

    // Connection parameters init
    {
        ble_conn_params_init_t cp_init = {0};
        cp_init.p_conn_params = NULL;
        cp_init.first_conn_params_update_delay = APP_TIMER_TICKS(5000); // Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds)
        cp_init.next_conn_params_update_delay = APP_TIMER_TICKS(30000); // Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds)
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
                                    APP_TIMER_TICKS(5000),
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
    APP_SCHED_INIT(APP_TIMER_SCHED_EVENT_DATA_SIZE, 20);

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