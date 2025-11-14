#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "soc/gpio_struct.h"
#include <stdbool.h>
#include <string.h>
#include <stdint.h>

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

#define COLUMNS 80
#define SCAN     5
#define PANEL_W 40
#define PANEL_H 20
#define BIT_DEPTH 6

typedef struct { uint8_t x; uint8_t y; } PixelMap;

// ==== YOUR ORIGINAL MAP PRESERVED AS-IS ====
static const PixelMap panel_map[10][40] = {
  // y = 0
  {
    {0,4},{1,4},{2,4},{3,4},{12,4},{13,4},{14,4},{15,4},
    {16,4},{17,4},{18,4},{19,4},{28,4},{29,4},{30,4},{31,4},
    {32,4},{33,4},{34,4},{35,4},{44,4},{45,4},{46,4},{47,4},
    {48,4},{49,4},{50,4},{51,4},{60,4},{61,4},{62,4},{63,4},
    {64,4},{65,4},{66,4},{67,4},{76,4},{77,4},{78,4},{79,4}
  },
  // y = 1
  {
    {0,0},{1,0},{2,0},{3,0},{12,0},{13,0},{14,0},{15,0},
    {16,0},{17,0},{18,0},{19,0},{28,0},{29,0},{30,0},{31,0},
    {32,0},{33,0},{34,0},{35,0},{44,0},{45,0},{46,0},{47,0},
    {48,0},{49,0},{50,0},{51,0},{60,0},{61,0},{62,0},{63,0},
    {64,0},{65,0},{66,0},{67,0},{76,0},{77,0},{78,0},{79,0}
  },
  // y = 2
  {
    {0,1},{1,1},{2,1},{3,1},{12,1},{13,1},{14,1},{15,1},
    {16,1},{17,1},{18,1},{19,1},{28,1},{29,1},{30,1},{31,1},
    {32,1},{33,1},{34,1},{35,1},{44,1},{45,1},{46,1},{47,1},
    {48,1},{49,1},{50,1},{51,1},{60,1},{61,1},{62,1},{63,1},
    {64,1},{65,1},{66,1},{67,1},{76,1},{77,1},{78,1},{79,1}
  },
  // y = 3
  {
    {0,2},{1,2},{2,2},{3,2},{12,2},{13,2},{14,2},{15,2},
    {16,2},{17,2},{18,2},{19,2},{28,2},{29,2},{30,2},{31,2},
    {32,2},{33,2},{34,2},{35,2},{44,2},{45,2},{46,2},{47,2},
    {48,2},{49,2},{50,2},{51,2},{60,2},{61,2},{62,2},{63,2},
    {64,2},{65,2},{66,2},{67,2},{76,2},{77,2},{78,2},{79,2}
  },
  // y = 4
  {
    {0,3},{1,3},{2,3},{3,3},{12,3},{13,3},{14,3},{15,3},
    {16,3},{17,3},{18,3},{19,3},{28,3},{29,3},{30,3},{31,3},
    {32,3},{33,3},{34,3},{35,3},{44,3},{45,3},{46,3},{47,3},
    {48,3},{49,3},{50,3},{51,3},{60,3},{61,3},{62,3},{63,3},
    {64,3},{65,3},{66,3},{67,3},{76,3},{77,3},{78,3},{79,3}
  },
  // y = 5
  {
    {4,4},{5,4},{6,4},{7,4},{8,4},{9,4},{10,4},{11,4},
    {20,4},{21,4},{22,4},{23,4},{24,4},{25,4},{26,4},{27,4},
    {36,4},{37,4},{38,4},{39,4},{40,4},{41,4},{42,4},{43,4},
    {52,4},{53,4},{54,4},{55,4},{56,4},{57,4},{58,4},{59,4},
    {68,4},{69,4},{70,4},{71,4},{72,4},{73,4},{74,4},{75,4}
  },
  // y = 6
  {
    {4,0},{5,0},{6,0},{7,0},{8,0},{9,0},{10,0},{11,0},
    {20,0},{21,0},{22,0},{23,0},{24,0},{25,0},{26,0},{27,0},
    {36,0},{37,0},{38,0},{39,0},{40,0},{41,0},{42,0},{43,0},
    {52,0},{53,0},{54,0},{55,0},{56,0},{57,0},{58,0},{59,0},
    {68,0},{69,0},{70,0},{71,0},{72,0},{73,0},{74,0},{75,0}
  },
  // y = 7
  {
    {4,1},{5,1},{6,1},{7,1},{8,1},{9,1},{10,1},{11,1},
    {20,1},{21,1},{22,1},{23,1},{24,1},{25,1},{26,1},{27,1},
    {36,1},{37,1},{38,1},{39,1},{40,1},{41,1},{42,1},{43,1},
    {52,1},{53,1},{54,1},{55,1},{56,1},{57,1},{58,1},{59,1},
    {68,1},{69,1},{70,1},{71,1},{72,1},{73,1},{74,1},{75,1}
  },
  // y = 8
  {
    {4,2},{5,2},{6,2},{7,2},{8,2},{9,2},{10,2},{11,2},
    {20,2},{21,2},{22,2},{23,2},{24,2},{25,2},{26,2},{27,2},
    {36,2},{37,2},{38,2},{39,2},{40,2},{41,2},{42,2},{43,2},
    {52,2},{53,2},{54,2},{55,2},{56,2},{57,2},{58,2},{59,2},
    {68,2},{69,2},{70,2},{71,2},{72,2},{73,2},{74,2},{75,2}
  },
  // y = 9
  {
    {4,3},{5,3},{6,3},{7,3},{8,3},{9,3},{10,3},{11,3},
    {20,3},{21,3},{22,3},{23,3},{24,3},{25,3},{26,3},{27,3},
    {36,3},{37,3},{38,3},{39,3},{40,3},{41,3},{42,3},{43,3},
    {52,3},{53,3},{54,3},{55,3},{56,3},{57,3},{58,3},{59,3},
    {68,3},{69,3},{70,3},{71,3},{72,3},{73,3},{74,3},{75,3}
  }
};


// ===== FRAMEBUFFER (6-bit RGB) =====
typedef struct { uint8_t r,g,b; } Pixel;
static Pixel frame[PANEL_H][PANEL_W] = {0};

// ===== PATTERN BUFFER =====
static uint8_t pattern[SCAN][COLUMNS];

// ===== PIN MASKS =====
#define BITPIN(pin) (1U << (pin))
static const uint32_t MASK_R0 = BITPIN(PIN_R0);
static const uint32_t MASK_G0 = BITPIN(PIN_G0);
static const uint32_t MASK_B0 = BITPIN(PIN_B0);
static const uint32_t MASK_R1 = BITPIN(PIN_R1);
static const uint32_t MASK_G1 = BITPIN(PIN_G1);
static const uint32_t MASK_B1 = BITPIN(PIN_B1);
static const uint32_t MASK_RGB =
    MASK_R0|MASK_G0|MASK_B0|MASK_R1|MASK_G1|MASK_B1;
static const uint32_t MASK_CLK = BITPIN(PIN_CLK);
static const uint32_t MASK_LE  = BITPIN(PIN_LE);
static const uint32_t MASK_OE  = BITPIN(PIN_OE);
static const uint32_t MASK_A   = BITPIN(PIN_A);
static const uint32_t MASK_B   = BITPIN(PIN_B);
static const uint32_t MASK_C   = BITPIN(PIN_C);

// ===== DIRECT REGISTER WRITES =====
static inline void gpio_out_set_levels(uint8_t patt)
{
    uint32_t out = 0;
    if (patt & 0x01) out |= MASK_R0;
    if (patt & 0x02) out |= MASK_G0;
    if (patt & 0x04) out |= MASK_B0;
    if (patt & 0x08) out |= MASK_R1;
    if (patt & 0x10) out |= MASK_G1;
    if (patt & 0x20) out |= MASK_B1;

    GPIO.out_w1tc = MASK_RGB;
    GPIO.out_w1ts = out;
}

static inline void pulse_clk(void)
{
    GPIO.out_w1ts = MASK_CLK;
    GPIO.out_w1tc = MASK_CLK;
}

static inline void set_pin(uint32_t mask,bool v)
{
    if(v) GPIO.out_w1ts = mask; else GPIO.out_w1tc = mask;
}

// ===== PIN INIT =====
static void init_pins(void)
{
    uint64_t mask =
        (1ULL<<PIN_R0)|(1ULL<<PIN_G0)|(1ULL<<PIN_B0)|
        (1ULL<<PIN_R1)|(1ULL<<PIN_G1)|(1ULL<<PIN_B1)|
        (1ULL<<PIN_CLK)|(1ULL<<PIN_LE)|(1ULL<<PIN_OE)|
        (1ULL<<PIN_A)|(1ULL<<PIN_B)|(1ULL<<PIN_C);

    gpio_config_t io = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);

    GPIO.out_w1tc = mask;    // clear all
    GPIO.out_w1ts = MASK_OE; // OE high
}

// ===== BUILD BIT-PLANE PATTERN =====
static void build_bitplane_pattern(int row,int bit)
{
    memset(pattern[row],0,sizeof(pattern[row]));
    for(int sy=0;sy<PANEL_H;sy++)
    {
        PixelMap *map_row = (sy<10)?panel_map[sy]:panel_map[sy-10];
        for(int sx=0;sx<PANEL_W;sx++)
        {
            Pixel px = frame[sy][sx];
            PixelMap p = map_row[sx];
            if(p.y>=SCAN) continue;
            int col = p.x;
            int rbit = (px.r>>bit)&1;
            int gbit = (px.g>>bit)&1;
            int bbit = (px.b>>bit)&1;

            if(sy<10)
                pattern[p.y][col] |= (rbit<<0)|(gbit<<1)|(bbit<<2);
            else
                pattern[p.y][col] |= (rbit<<3)|(gbit<<4)|(bbit<<5);
        }
    }
}

// ===== REFRESH TASK (MAX OE BRIGHTNESS + 6-BIT PWM) =====
static void refresh_task(void *arg)
{
    const int bit_durations[BIT_DEPTH]={2,3,6,12,25,50}; // brightness optimized
    while(1)
    {
        for(int row=0;row<SCAN;row++)
        {
            set_pin(MASK_OE,0);
            set_pin(MASK_A,row&1);
            set_pin(MASK_B,(row>>1)&1);
            set_pin(MASK_C,(row>>2)&1);

            for(int bit=0;bit<BIT_DEPTH;bit++)
            {
                build_bitplane_pattern(row,bit);
                int duration = bit_durations[bit];
                for(int d=0;d<duration;d++)
                {
                    for(int col=0;col<COLUMNS;col++)
                    {
                        gpio_out_set_levels(pattern[row][col]);
                        pulse_clk();
                        if(col==(COLUMNS-4)) set_pin(MASK_LE,1);
                    }
                    set_pin(MASK_LE,0); // OE stays low
                }
            }
            set_pin(MASK_OE,1); // enable OE only after full row
        }
    }
}

// ===== SET PIXEL (6-bit RGB) =====
void set_pixel(int sx,int sy,uint8_t r,uint8_t g,uint8_t b)
{
    if(sx<0||sx>=PANEL_W||sy<0||sy>=PANEL_H) return;
    frame[sy][sx].r = r&0x3F;
    frame[sy][sx].g = g&0x3F;
    frame[sy][sx].b = b&0x3F;
}

// ===== MAIN =====
void app_main(void)
{
    init_pins();
    memset(frame,0,sizeof(frame));

    xTaskCreatePinnedToCore(refresh_task,"refresh_task",4096,NULL,1,NULL,0);

    // Example: fill panel pixel by pixel with red
    for(int y=0;y<PANEL_H;y++)
        for(int x=0;x<PANEL_W;x++)
        {
            set_pixel(x,y,0,63,63); // max red
            vTaskDelay(pdMS_TO_TICKS(1));
        }

    while(1) vTaskDelay(pdMS_TO_TICKS(1));
}
















/*
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include <stdbool.h>
#include <string.h>

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

#define COLUMNS 80     // keep your logic (panel needs 80 shift clocks)
#define SCAN     5     // 1/5 scan
#define PANEL_W 40
#define PANEL_H 20

typedef struct { uint8_t x; uint8_t y; } PixelMap;

// ==== YOUR ORIGINAL MAP PRESERVED AS-IS ====
static const PixelMap panel_map[10][40] = {
  // y = 0
  {
    {0,4},{1,4},{2,4},{3,4},{12,4},{13,4},{14,4},{15,4},
    {16,4},{17,4},{18,4},{19,4},{28,4},{29,4},{30,4},{31,4},
    {32,4},{33,4},{34,4},{35,4},{44,4},{45,4},{46,4},{47,4},
    {48,4},{49,4},{50,4},{51,4},{60,4},{61,4},{62,4},{63,4},
    {64,4},{65,4},{66,4},{67,4},{76,4},{77,4},{78,4},{79,4}
  },
  // y = 1
  {
    {0,0},{1,0},{2,0},{3,0},{12,0},{13,0},{14,0},{15,0},
    {16,0},{17,0},{18,0},{19,0},{28,0},{29,0},{30,0},{31,0},
    {32,0},{33,0},{34,0},{35,0},{44,0},{45,0},{46,0},{47,0},
    {48,0},{49,0},{50,0},{51,0},{60,0},{61,0},{62,0},{63,0},
    {64,0},{65,0},{66,0},{67,0},{76,0},{77,0},{78,0},{79,0}
  },
  // y = 2
  {
    {0,1},{1,1},{2,1},{3,1},{12,1},{13,1},{14,1},{15,1},
    {16,1},{17,1},{18,1},{19,1},{28,1},{29,1},{30,1},{31,1},
    {32,1},{33,1},{34,1},{35,1},{44,1},{45,1},{46,1},{47,1},
    {48,1},{49,1},{50,1},{51,1},{60,1},{61,1},{62,1},{63,1},
    {64,1},{65,1},{66,1},{67,1},{76,1},{77,1},{78,1},{79,1}
  },
  // y = 3
  {
    {0,2},{1,2},{2,2},{3,2},{12,2},{13,2},{14,2},{15,2},
    {16,2},{17,2},{18,2},{19,2},{28,2},{29,2},{30,2},{31,2},
    {32,2},{33,2},{34,2},{35,2},{44,2},{45,2},{46,2},{47,2},
    {48,2},{49,2},{50,2},{51,2},{60,2},{61,2},{62,2},{63,2},
    {64,2},{65,2},{66,2},{67,2},{76,2},{77,2},{78,2},{79,2}
  },
  // y = 4
  {
    {0,3},{1,3},{2,3},{3,3},{12,3},{13,3},{14,3},{15,3},
    {16,3},{17,3},{18,3},{19,3},{28,3},{29,3},{30,3},{31,3},
    {32,3},{33,3},{34,3},{35,3},{44,3},{45,3},{46,3},{47,3},
    {48,3},{49,3},{50,3},{51,3},{60,3},{61,3},{62,3},{63,3},
    {64,3},{65,3},{66,3},{67,3},{76,3},{77,3},{78,3},{79,3}
  },
  // y = 5
  {
    {4,4},{5,4},{6,4},{7,4},{8,4},{9,4},{10,4},{11,4},
    {20,4},{21,4},{22,4},{23,4},{24,4},{25,4},{26,4},{27,4},
    {36,4},{37,4},{38,4},{39,4},{40,4},{41,4},{42,4},{43,4},
    {52,4},{53,4},{54,4},{55,4},{56,4},{57,4},{58,4},{59,4},
    {68,4},{69,4},{70,4},{71,4},{72,4},{73,4},{74,4},{75,4}
  },
  // y = 6
  {
    {4,0},{5,0},{6,0},{7,0},{8,0},{9,0},{10,0},{11,0},
    {20,0},{21,0},{22,0},{23,0},{24,0},{25,0},{26,0},{27,0},
    {36,0},{37,0},{38,0},{39,0},{40,0},{41,0},{42,0},{43,0},
    {52,0},{53,0},{54,0},{55,0},{56,0},{57,0},{58,0},{59,0},
    {68,0},{69,0},{70,0},{71,0},{72,0},{73,0},{74,0},{75,0}
  },
  // y = 7
  {
    {4,1},{5,1},{6,1},{7,1},{8,1},{9,1},{10,1},{11,1},
    {20,1},{21,1},{22,1},{23,1},{24,1},{25,1},{26,1},{27,1},
    {36,1},{37,1},{38,1},{39,1},{40,1},{41,1},{42,1},{43,1},
    {52,1},{53,1},{54,1},{55,1},{56,1},{57,1},{58,1},{59,1},
    {68,1},{69,1},{70,1},{71,1},{72,1},{73,1},{74,1},{75,1}
  },
  // y = 8
  {
    {4,2},{5,2},{6,2},{7,2},{8,2},{9,2},{10,2},{11,2},
    {20,2},{21,2},{22,2},{23,2},{24,2},{25,2},{26,2},{27,2},
    {36,2},{37,2},{38,2},{39,2},{40,2},{41,2},{42,2},{43,2},
    {52,2},{53,2},{54,2},{55,2},{56,2},{57,2},{58,2},{59,2},
    {68,2},{69,2},{70,2},{71,2},{72,2},{73,2},{74,2},{75,2}
  },
  // y = 9
  {
    {4,3},{5,3},{6,3},{7,3},{8,3},{9,3},{10,3},{11,3},
    {20,3},{21,3},{22,3},{23,3},{24,3},{25,3},{26,3},{27,3},
    {36,3},{37,3},{38,3},{39,3},{40,3},{41,3},{42,3},{43,3},
    {52,3},{53,3},{54,3},{55,3},{56,3},{57,3},{58,3},{59,3},
    {68,3},{69,3},{70,3},{71,3},{72,3},{73,3},{74,3},{75,3}
  }
};

// ===============================
// NEW: framebuffer for persistent pixels
// ===============================
static bool frame[PANEL_H][PANEL_W] = {0};

static bool y_bottom = false;
static int current_row = 0;
static int current_col = 0;


// =====================================
// GPIO INIT
// =====================================
static void init_pins(void)
{
    uint64_t mask =
        (1ULL<<PIN_R0)|(1ULL<<PIN_G0)|(1ULL<<PIN_B0) |
        (1ULL<<PIN_R1)|(1ULL<<PIN_G1)|(1ULL<<PIN_B1) |
        (1ULL<<PIN_CLK)|(1ULL<<PIN_LE)|(1ULL<<PIN_OE) |
        (1ULL<<PIN_A)|(1ULL<<PIN_B)|(1ULL<<PIN_C);

    gpio_config_t io = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);

    gpio_set_level(PIN_R0, 0);
    gpio_set_level(PIN_G0, 0);
    gpio_set_level(PIN_B0, 0);
    gpio_set_level(PIN_R1, 0);
    gpio_set_level(PIN_G1, 0);
    gpio_set_level(PIN_B1, 0);
    gpio_set_level(PIN_CLK, 0);
    gpio_set_level(PIN_LE, 0);
    gpio_set_level(PIN_OE, 1);
    gpio_set_level(PIN_A, 0);
    gpio_set_level(PIN_B, 0);
    gpio_set_level(PIN_C, 0);
}


// =====================================
// SHIFT
// =====================================
static inline void pulse_clk(void)
{
    gpio_set_level(PIN_CLK, 1);
    gpio_set_level(PIN_CLK, 0);
}

static inline void shift_bit(bool r0, bool g0, bool b0, bool r1, bool g1, bool b1, int col)
{
    gpio_set_level(PIN_R0, r0);
    gpio_set_level(PIN_G0, g0);
    gpio_set_level(PIN_B0, b0);
    gpio_set_level(PIN_R1, r1);
    gpio_set_level(PIN_G1, g1);
    gpio_set_level(PIN_B1, b1);

    pulse_clk();

    if (col == (COLUMNS - 4))   // keep your timing
        gpio_set_level(PIN_LE, 1);
}


// =====================================
// REFRESH TASK (reads framebuffer)
// =====================================
static void refresh_task(void *arg)
{
    while (1)
    {
        for (int row = 0; row < SCAN; row++)
        {
            gpio_set_level(PIN_OE, 0);

            gpio_set_level(PIN_A, (row >> 0) & 1);
            gpio_set_level(PIN_B, (row >> 1) & 1);
            gpio_set_level(PIN_C, (row >> 2) & 1);

            for (int col = 0; col < COLUMNS; col++)
            {
                bool top_on = false;
                bool bot_on = false;

                // top half (virtual y 0–9)
                for (int sy = 0; sy < 10; sy++)
                {
                    for (int sx = 0; sx < 40; sx++)
                    {
                        if (frame[sy][sx] == 0) continue;

                        PixelMap p = panel_map[sy][sx];
                        if (p.y == row && p.x == col)
                            top_on = true;
                    }
                }

                // bottom half (virtual y 10–19)
                for (int sy = 10; sy < 20; sy++)
                {
                    for (int sx = 0; sx < 40; sx++)
                    {
                        if (frame[sy][sx] == 0) continue;

                        PixelMap p = panel_map[sy-10][sx];
                        if (p.y == row && p.x == col)
                            bot_on = true;
                    }
                }

                shift_bit(top_on, top_on, top_on, bot_on, bot_on, bot_on, col);
            }

            gpio_set_level(PIN_LE, 0);
            gpio_set_level(PIN_OE, 1);
            //esp_rom_delay_us(1);
        }
    }
}


// =====================================
// SET PIXEL → Writes into framebuffer
// =====================================
void set_pixel(int sx, int sy)
{
    if (sx < 0 || sx >= 40) return;
    if (sy < 0 || sy >= 20) return;

    frame[sy][sx] = true;  // persistent

    int sy_map = sy;
    y_bottom = (sy >= 10);
    if (y_bottom) sy_map -= 10;

    PixelMap p = panel_map[sy_map][sx];
    current_col = p.x;
    current_row = p.y;
}


// =====================================
// MAIN: fill pixel-by-pixel, then hold
// =====================================
void app_main(void)
{
    init_pins();
    xTaskCreatePinnedToCore(refresh_task, "refresh_task", 4096, NULL, 1, NULL, 0);

    // fill panel pixel-by-pixel
    for (int y = 0; y < 20; y++)
    {
        for (int x = 0; x < 40; x++)
        {
            set_pixel(x, y);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }


}


*/        