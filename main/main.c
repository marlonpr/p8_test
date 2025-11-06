#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define PIN_R0   GPIO_NUM_2
#define PIN_G0   GPIO_NUM_4
#define PIN_B0   GPIO_NUM_5
#define PIN_R1   GPIO_NUM_18
#define PIN_G1   GPIO_NUM_19
#define PIN_B1   GPIO_NUM_25
#define PIN_CLK  GPIO_NUM_13
#define PIN_LE   GPIO_NUM_12
#define PIN_OE   GPIO_NUM_14
#define PIN_A    GPIO_NUM_15
#define PIN_B    GPIO_NUM_26
#define PIN_C    GPIO_NUM_23


#define WIDTH        40     // columns per row
#define ROW_HOLD_US  800    // microseconds per row

// Initialize GPIOs as outputs, set safe defaults
static void init_pins(void) {
    uint64_t mask =
        (1ULL<<PIN_R0)|(1ULL<<PIN_G0)|(1ULL<<PIN_B0) |
		(1ULL<<PIN_R1)|(1ULL<<PIN_G1)|(1ULL<<PIN_B1)|
        (1ULL<<PIN_CLK)|(1ULL<<PIN_LE)|(1ULL<<PIN_OE) |
        (1ULL<<PIN_A)|(1ULL<<PIN_B)|(1ULL<<PIN_C);
    gpio_config_t io = { .pin_bit_mask = mask, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&io);


}

static inline void pulse_clk(void) {
    gpio_set_level(PIN_CLK, 1);
    //__asm__ __volatile__("nop\nnop");  // small settle
    gpio_set_level(PIN_CLK, 0);
}

// Perform LE=3CLK latch sequence
static void latch_3clk(void) {
    //gpio_set_level(PIN_OE, 1);  // blank
    //gpio_set_level(PIN_LE, 1);
    for (int i = 0; i < 3; ++i) {
        //__asm__ __volatile__("nop\nnop");
        //pulse_clk();
    }
    gpio_set_level(PIN_LE, 0);
}

// Shift one row’s bits: only col 0 is “on” for row 0
static void shift_row_bits(int row) {
	
    for (int col = 0; col < 80; col++) {
        



	//bool bit_on = (row == 0 && (col >= 3 && col <= 77));


	bool bit_on  = col >= 3 && col <= 76;



		
        gpio_set_level(PIN_R0, bit_on);
        //gpio_set_level(PIN_G0, bit_on);
        //gpio_set_level(PIN_B0, bit_on);
        gpio_set_level(PIN_R1, bit_on);
        //gpio_set_level(PIN_G1, bit_on);
        //gpio_set_level(PIN_B1, bit_on);
        pulse_clk();
    }
}

void app_main(void) {
    init_pins();
	esp_rom_delay_us(ROW_HOLD_US*3);
    while (true) {
        for (int row = 0; row < 5; row++) {  


		            gpio_set_level(PIN_OE, 1);


            // set row address
            gpio_set_level(PIN_A, (row >> 0) & 1);
            gpio_set_level(PIN_B, (row >> 1) & 1);
            gpio_set_level(PIN_C, (row >> 2) & 1);


        	gpio_set_level(PIN_LE, 1);
			
			shift_row_bits(row);


			gpio_set_level(PIN_LE, 0);
		

			//pulse_clk();
            //pulse_clk();
            //pulse_clk();
            //latch_3clk();

            //latch_3clk();
            //latch_3clk();



            // unblank, hold then blank
            gpio_set_level(PIN_OE, 0);
            esp_rom_delay_us(ROW_HOLD_US);

        }
    }
}




/*

int bit_on = 0;
		if (row == 4 && (col >= 3 && col <= 78))
		{
			bit_on = 1;		
		}
		else {
			bit_on = 0;
		}


*/












/*

// main.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include <stdint.h>

static const char *TAG = "p8_all_white";

#define PIN_R0   GPIO_NUM_2
#define PIN_G0   GPIO_NUM_4
#define PIN_B0   GPIO_NUM_5
#define PIN_R1   GPIO_NUM_18
#define PIN_G1   GPIO_NUM_19
#define PIN_B1   GPIO_NUM_25
#define PIN_CLK  GPIO_NUM_13
#define PIN_LE   GPIO_NUM_12
#define PIN_OE   GPIO_NUM_14
#define PIN_A    GPIO_NUM_15
#define PIN_B    GPIO_NUM_26
#define PIN_C    GPIO_NUM_23

#define PANEL_WIDTH 40

static inline void set_rgb_row_levels(bool r0,bool g0,bool b0,bool r1,bool g1,bool b1)
{
    gpio_set_level(PIN_R0, r0);
    gpio_set_level(PIN_G0, g0);
    gpio_set_level(PIN_B0, b0);
    gpio_set_level(PIN_R1, r1);
    gpio_set_level(PIN_G1, g1);
    gpio_set_level(PIN_B1, b1);
}

static inline void pulse_clk(void)
{
    gpio_set_level(PIN_CLK, 1);
    asm volatile("nop"); asm volatile("nop");
    gpio_set_level(PIN_CLK, 0);
}

static inline void pulse_le(void)
{
    gpio_set_level(PIN_LE, 1);
    pulse_clk();
    pulse_clk();
    pulse_clk();
    esp_rom_delay_us(1);
    gpio_set_level(PIN_LE, 0);
}

static inline void set_address(uint8_t addr)
{
    gpio_set_level(PIN_A, (addr >> 0) & 1);
    gpio_set_level(PIN_B, (addr >> 1) & 1);
    gpio_set_level(PIN_C, (addr >> 2) & 1);
}

static inline void set_oe(bool blank)
{
    // If your panel uses OE active-low, invert this logic.
    gpio_set_level(PIN_OE, blank ? 1 : 0);
}

static void init_pins(void)
{
    gpio_config_t io_conf = {0};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask =
        (1ULL<<PIN_R0) | (1ULL<<PIN_G0) | (1ULL<<PIN_B0) |
        (1ULL<<PIN_R1) | (1ULL<<PIN_G1) | (1ULL<<PIN_B1) |
        (1ULL<<PIN_CLK) | (1ULL<<PIN_LE) | (1ULL<<PIN_OE) |
        (1ULL<<PIN_A) | (1ULL<<PIN_B) | (1ULL<<PIN_C);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_level(PIN_CLK, 0);
    gpio_set_level(PIN_LE, 0);
    set_oe(true);
}

static void panel_task(void *arg)
{
    ESP_LOGI(TAG, "making panel all white (single-bit)");
    while (1) {
        for (uint8_t addr = 0; addr < 5; ++addr) {
            set_address(addr);
            set_oe(true); // blank while shifting

            // For full white: set all color lines high for both halves
            for (uint32_t col = 0; col < PANEL_WIDTH; ++col) {
                set_rgb_row_levels(1,1,1, 1,1,1);
                pulse_clk();
            }

            pulse_le();
            set_oe(false); // enable output to show the latched row
            // display time for this row group; adjust for brightness/refresh
            esp_rom_delay_us(1500);
            set_oe(true);  // blank before updating next address
        }
        // small frame delay to keep CPU friendly; can be reduced
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main(void)
{
    init_pins();
    xTaskCreate(panel_task, "panel_task", 4096, NULL, 5, NULL);
}

*/