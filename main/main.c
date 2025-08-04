#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// —— adjust these to your wiring ——
#define PIN_R0    GPIO_NUM_2
#define PIN_G0    GPIO_NUM_4
#define PIN_B0    GPIO_NUM_5
#define PIN_R1    GPIO_NUM_18
#define PIN_G1    GPIO_NUM_19
#define PIN_B1    GPIO_NUM_21

#define PIN_CLK   GPIO_NUM_13
#define PIN_LE   GPIO_NUM_12
#define PIN_OE    GPIO_NUM_14

#define PIN_A     GPIO_NUM_15
#define PIN_B     GPIO_NUM_22
#define PIN_C     GPIO_NUM_23
// no D/E on this 32-row module



#define WIDTH    40
#define ROW_DELAY_MS  100

static void init_pins(void)
{
    uint64_t mask = (1ULL<<PIN_R0)|(1ULL<<PIN_G0)|(1ULL<<PIN_B0)
                  | (1ULL<<PIN_CLK)|(1ULL<<PIN_LE)|(1ULL<<PIN_OE)
                  | (1ULL<<PIN_A ) |(1ULL<<PIN_B ) |(1ULL<<PIN_C );
    gpio_config_t io = {
      .pin_bit_mask = mask,
      .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&io);
    // safe defaults
    gpio_set_level(PIN_CLK, 0);
    gpio_set_level(PIN_LE,  0);
    gpio_set_level(PIN_OE,  1);  // blank
    gpio_set_level(PIN_A,   0);
    gpio_set_level(PIN_B,   0);
    gpio_set_level(PIN_C,   0);
    gpio_set_level(PIN_R0,  0);
    gpio_set_level(PIN_G0,  0);
    gpio_set_level(PIN_B0,  0);
}

static inline void pulse_clk(void)
{
    gpio_set_level(PIN_CLK, 1);
    gpio_set_level(PIN_CLK, 0);
}

static void shift_all_ones(void)
{
    // drive R/G/B all high
    for (int i = 0; i < WIDTH; ++i) {
        gpio_set_level(PIN_R0, 1);
        gpio_set_level(PIN_G0, 1);
        gpio_set_level(PIN_B0, 1);
        pulse_clk();
    }
}

// this is the LE=3-CLK latch sequence from the datasheet
static void latch_3clk(void)
{
    // (1) blank outputs
    gpio_set_level(PIN_OE, 1);
    // (2) bring LE high
    gpio_set_level(PIN_LE, 1);
    // (3) pulse CLK exactly 3 times while LE remains high
    for (int i = 0; i < 3; ++i) {
        esp_rom_delay_us(1);  // small settle
        pulse_clk();
    }
    // (4) bring LE low
    gpio_set_level(PIN_LE, 0);
    // (5) leave OE=1 until we set address
}

void app_main(void)
{
    init_pins();

    // preload shift-register with WIDTH×(1,1,1)
    shift_all_ones();

    while (1) {
        // for each logical row 0..7 (1/8 scan)
        for (int row = 0; row < 8; ++row) {
            latch_3clk();

            // apply row address
            gpio_set_level(PIN_A, (row >> 0) & 1);
            gpio_set_level(PIN_B, (row >> 1) & 1);
            gpio_set_level(PIN_C, (row >> 2) & 1);

            // display it
            gpio_set_level(PIN_OE, 0);
            vTaskDelay(pdMS_TO_TICKS(ROW_DELAY_MS));

            // blank before next
            gpio_set_level(PIN_OE, 1);
        }
    }
}
