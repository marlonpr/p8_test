#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define PIN_R0   GPIO_NUM_2
#define PIN_G0   GPIO_NUM_4
#define PIN_B0   GPIO_NUM_5
#define PIN_R1   GPIO_NUM_18
#define PIN_G1   GPIO_NUM_19
#define PIN_B1   GPIO_NUM_21
#define PIN_CLK  GPIO_NUM_13
#define PIN_LE  GPIO_NUM_12
#define PIN_OE   GPIO_NUM_14
#define PIN_A    GPIO_NUM_15
#define PIN_B    GPIO_NUM_22
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
    gpio_set_level(PIN_LE, 1);
    for (int i = 0; i < 3; ++i) {
        //__asm__ __volatile__("nop\nnop");
        pulse_clk();
    }
    gpio_set_level(PIN_LE, 0);
}

// Shift one row’s bits: only col 0 is “on” for row 0
static void shift_row_bits(int row) {
	
    for (int col = 0; col < 80; col++) {
        



	//bool bit_on = (row == 0 && (col >= 3 && col <= 77));


	bool bit_on  = col >= 0 && col <= 77;



		
        gpio_set_level(PIN_R0, bit_on);
        gpio_set_level(PIN_G0, bit_on);
        gpio_set_level(PIN_B0, bit_on);
        gpio_set_level(PIN_R1, bit_on);
        gpio_set_level(PIN_G1, bit_on);
        gpio_set_level(PIN_B1, bit_on);
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


        
			
			shift_row_bits(row);


            latch_3clk();



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
