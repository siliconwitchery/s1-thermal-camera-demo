/**
 * @file  examples/fpga-blinky/main.c
 * @brief Simple FPGA blinky Application running on S1
 *        
 *        Includes basic configuration of the S1 module, and
 *        operations required to boot the FPGA. The FPGA 
 *        verilog project can be built by running:
 *           "make build-verilog" 
 *        from this folder.
 * 
 * @attention Copyright 2021 Silicon Witchery AB
 *
 * Permission to use, copy, modify, and/or distribute this 
 * software for any purpose with or without fee is hereby
 * granted, provided that the above copyright notice and this
 * permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS
 * ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO 
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, 
 * INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER 
 * RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN 
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, 
 * ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE 
 * OF THIS SOFTWARE.
 */

#include <math.h>
#include "app_scheduler.h"
#include "app_timer.h"
#include "fpga_binfile.h"
#include "nrf_gpio.h"
#include "nrf52811.h"
#include "nrfx_clock.h"
#include "nrfx_saadc.h"
#include "nrfx_spim.h"
#include "nrfx_twim.h"
#include "s1.h"

APP_TIMER_DEF(fpga_boot_task_id);

typedef enum
{
    STARTED,
    ERASING,
    FLASHING,
    WAIT_FOR_DATA,
    DUMP_SPI
} fpga_boot_state_t;

static fpga_boot_state_t fpga_boot_state = STARTED;
static uint32_t pages_remaining;
static uint32_t page_address = 0x000000;

static uint8_t image_buffer[1536];

/**
 * @brief Clock event callback. Not used but required to have.
 */
void clock_event_handler(nrfx_clock_evt_type_t event) {}

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

void get_image(void)
{
    nrfx_spim_xfer_desc_t spi_xfer = NRFX_SPIM_XFER_RX(&image_buffer, 1536);

    nrfx_spim_t spi = NRFX_SPIM_INSTANCE(0);

    APP_ERROR_CHECK(nrfx_spim_xfer(&spi, &spi_xfer, 0));
}

/**
 * @brief Timer based state machine for flashing the FPGA
 *        image and booting the FPGA. As some of the flash
 *        operations take a lot of time, using a timer based
 *        state machine avoids the main thread hanging while
 *        waiting for flash operations to complete.
 */
static void fpga_boot_task(void *p_context)
{
    UNUSED_PARAMETER(p_context);
    switch (fpga_boot_state)
    {
    // Configure power and erase the flash
    case STARTED:
        s1_pimc_fpga_vcore(true);
        s1_pmic_set_vio(2.8);
        s1_pmic_set_vaux(3.3);
        s1_fpga_hold_reset();
        s1_flash_wakeup();
        s1_flash_erase_all();
        fpga_boot_state = ERASING;
        LOG("Erasing flash. Takes up to 80 seconds.");
        break;

    // Wait for erase to complete
    case ERASING:
        if (!s1_flash_is_busy())
        {
            pages_remaining = (uint32_t)ceil((float)fpga_binfile_bin_len / 256.0);
            fpga_boot_state = FLASHING;
            LOG("Flashing %d pages.", pages_remaining);
        }
        break;

    // Flash every page until done
    case FLASHING:
        if (!s1_flash_is_busy())
        {
            s1_flash_page_from_image(page_address, &fpga_binfile_bin);
            pages_remaining--;
            page_address += 0x100;
        }

        if (pages_remaining == 0)
        {
            fpga_boot_state = WAIT_FOR_DATA;
            s1_fpga_boot();
            LOG("Flashing done.");
            break;
        }
        break;

    // Wait for 1 second before reading back data§
    case WAIT_FOR_DATA:

        start_image_rx_bus();

        NRFX_DELAY_US(100000);

        fpga_boot_state = DUMP_SPI;
        break;

    // Dump data from FPGA over SPI
    case DUMP_SPI:
        get_image();
        for (uint32_t i = 0; i < 1536; i++)
        {
            LOG_RAW("0x%x, ", image_buffer[i]);
        }
        NRFX_DELAY_US(100000);

        break;
    }
}

/**
 * @brief Main application entry for the fpga-blinky demo.
 */
int main(void)
{
    // Log some stuff about this project
    LOG_CLEAR();
    LOG("S1 FPGA Blinky Demo – Built: %s %s – SDK Version: %s.",
        __DATE__,
        __TIME__,
        __S1_SDK_VERSION__);

    // Initialise S1 base setting
    APP_ERROR_CHECK(s1_init());

    // Initialise LFXO required by the App Timer
    APP_ERROR_CHECK(nrfx_clock_init(clock_event_handler));
    nrfx_clock_lfclk_start();

    // Initialise the App Timer
    APP_ERROR_CHECK(app_timer_init());
    APP_SCHED_INIT(sizeof(uint32_t), 5);

    // Create and start a timer for the FPGA flash/boot task
    APP_ERROR_CHECK(app_timer_create(&fpga_boot_task_id,
                                     APP_TIMER_MODE_REPEATED,
                                     fpga_boot_task));

    APP_ERROR_CHECK(app_timer_start(fpga_boot_task_id,
                                    APP_TIMER_TICKS(1),
                                    NULL));

    // The CPU is free to do nothing in the meanwhile
    for (;;)
    {
        __WFI();
    }
}