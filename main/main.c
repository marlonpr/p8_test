// main_double_buffered.c
// HUB75 driver - Mode 1 (1/5 scan per half), 6-bit per color double-buffered bitplanes
// Improvements: precomputed gamma6 LUTs, IRAM_ATTR hot functions, snprintf usage,
// unified BITPIN masks, fixed gains and brightness defaults.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "soc/gpio_struct.h"
#include "esp_attr.h"    // for IRAM_ATTR
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "font6x9.h"
#include <assert.h>
#include <stdlib.h>

/* -------------------- PIN DEFINITIONS -------------------- */
#define PIN_R0   GPIO_NUM_5
#define PIN_G0   GPIO_NUM_4
#define PIN_B0   GPIO_NUM_2
#define PIN_R1   GPIO_NUM_18
#define PIN_G1   GPIO_NUM_19
#define PIN_B1   GPIO_NUM_25
#define PIN_CLK  GPIO_NUM_13
#define PIN_LE   GPIO_NUM_12
#define PIN_OE   GPIO_NUM_14
#define PIN_A    GPIO_NUM_15
#define PIN_B    GPIO_NUM_26
#define PIN_C    GPIO_NUM_23

// -------------------- PANEL / LAYOUT CONFIG --------------------
#define PHYSICAL_PANEL_WIDTH 40
#define PHYSICAL_PANEL_HEIGHT 20

#define NUM_IC_PER_ROW 5    // driver chips per row (16-bit words each)
#define USE_LSB_FIRST 0      // 0 = MSB-first, 1 = LSB-first

// virtual grid: N = panels across; M = panels down
#define VIRTUAL_NUM_PANEL_HORIZONTAL 2
#define VIRTUAL_NUM_PANEL_VERTICAL   1

#define PANELS_ACROSS (VIRTUAL_NUM_PANEL_HORIZONTAL)
#define PANELS_DOWN   (VIRTUAL_NUM_PANEL_VERTICAL)

// sizes: per-row width (panels across), and flattened total width (panels across * panels down)
#define PANEL_W_PER_ROW  (PHYSICAL_PANEL_WIDTH * PANELS_ACROSS)                 // 40 * N
#define PANEL_W_TOTAL    (PHYSICAL_PANEL_WIDTH * PANELS_ACROSS * PANELS_DOWN)   // 40 * N * M
#define PANEL_H          (PHYSICAL_PANEL_HEIGHT)                                // 20

// COLUMNS = number of shift register columns you clock per scan row; adjust if different
#define COLUMNS (NUM_IC_PER_ROW * 16 * PANELS_ACROSS * PANELS_DOWN)

#define SCAN 5
#define BIT_DEPTH 6
#define BITPLANES BIT_DEPTH

// Buffering configuration: two bitplane buffers (front/back)
#define BITPLANE_BUFFERS 2

/* ----------------- LOW-LEVEL GPIOS ----------------- */

#define BITPIN(pin) (1U << (pin))

#define GPIO_SET(pin)    (GPIO.out_w1ts = BITPIN(pin))
#define GPIO_CLR(pin)    (GPIO.out_w1tc = BITPIN(pin))

/* ----------- DATA MASKS (R0 G0 B0 R1 G1 B1) ---------- */
static const uint32_t MASK_R0 = BITPIN(PIN_R0);
static const uint32_t MASK_G0 = BITPIN(PIN_G0);
static const uint32_t MASK_B0 = BITPIN(PIN_B0);
static const uint32_t MASK_R1 = BITPIN(PIN_R1);
static const uint32_t MASK_G1 = BITPIN(PIN_G1);
static const uint32_t MASK_B1 = BITPIN(PIN_B1);

static const uint32_t MASK_RGB =
    MASK_R0 | MASK_G0 | MASK_B0 |
    MASK_R1 | MASK_G1 | MASK_B1;

/* Useful for clearing data lines in one op */
#define DATA_PINS_MASK   MASK_RGB

/* ----------- CONTROL MASKS ---------- */
static const uint32_t MASK_CLK = BITPIN(PIN_CLK);
static const uint32_t MASK_LE  = BITPIN(PIN_LE);
static const uint32_t MASK_OE  = BITPIN(PIN_OE);

/* ----------- ADDRESS LINES ---------- */
static const uint32_t MASK_A = BITPIN(PIN_A);
static const uint32_t MASK_B = BITPIN(PIN_B);
static const uint32_t MASK_C = BITPIN(PIN_C);

/* -------------------- GPIO HELPERS -------------------- */
/* Map 6-bit pattern → physical RGB0/RGB1 bus */
IRAM_ATTR static inline void gpio_out_set_levels(uint8_t patt)
{
    uint32_t out = 0;

    if (patt & 0x01) out |= MASK_R0;
    if (patt & 0x02) out |= MASK_G0;
    if (patt & 0x04) out |= MASK_B0;
    if (patt & 0x08) out |= MASK_R1;
    if (patt & 0x10) out |= MASK_G1;
    if (patt & 0x20) out |= MASK_B1;

    /* Clear all RGB, then apply pattern */
    GPIO.out_w1tc = MASK_RGB;
    GPIO.out_w1ts = out;
}

IRAM_ATTR static inline void pulse_clk(void)
{
    GPIO.out_w1ts = MASK_CLK;
    GPIO.out_w1tc = MASK_CLK;
}

IRAM_ATTR static inline void set_pin(uint32_t mask, bool value)
{
    if (value) GPIO.out_w1ts = mask;
    else       GPIO.out_w1tc = mask;
}

/* -------------------- INIT PINS -------------------- */
static void init_pins(void)
{
    printf("Init pins with Pull-Downs\n");

    /* CLK & LE require pulldown at boot */
    const uint64_t pull_down_mask =
        (1ULL << PIN_CLK) |
        (1ULL << PIN_LE);

    /* All other pins */
    const uint64_t standard_mask =
          (1ULL << PIN_R0) | (1ULL << PIN_G0) | (1ULL << PIN_B0)
        | (1ULL << PIN_R1) | (1ULL << PIN_G1) | (1ULL << PIN_B1)
        | (1ULL << PIN_A)  | (1ULL << PIN_B) | (1ULL << PIN_C)
        | (1ULL << PIN_OE);

    const uint64_t total_mask = pull_down_mask | standard_mask;

    /* CLK & LE with pulldown */
    gpio_config_t pull_down_io = {
        .pin_bit_mask = pull_down_mask,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&pull_down_io);

    /* All other outputs */
    gpio_config_t standard_io = {
        .pin_bit_mask = standard_mask,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&standard_io);

    /* Clear all pins, then OE high */
    GPIO.out_w1tc = total_mask;
    GPIO.out_w1ts = BITPIN(PIN_OE);
} 

IRAM_ATTR static inline void short_nop_delay(void)
{
    // ~30 nops ≈ 100-150ns at 240MHz. Adjust when needed.
    asm volatile(
        "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
        "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
        "nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
        ::: "memory");
}

// Data valid before rising edge; sample on rising edge
IRAM_ATTR static inline void rul_clock_pulse_with_data_broadcast(uint8_t bit)
{
    // The bit contains the 1/0 register value. We use it to set ALL data pins.
    uint32_t val_to_set = (bit) ? DATA_PINS_MASK : 0;
    uint32_t val_to_clear = (bit) ? 0 : DATA_PINS_MASK;

    // Clear all pins that need to be cleared
    GPIO.out_w1tc = val_to_clear;
    
    // Set all pins that need to be set
    GPIO.out_w1ts = val_to_set; 
    
    // Data valid before rising edge; sample on rising edge
    GPIO_CLR(PIN_CLK);
    asm volatile ("" ::: "memory");
    GPIO_SET(PIN_CLK);  // rising edge -> sampled by device
    short_nop_delay();  // hold high to satisfy hold spec
    GPIO_CLR(PIN_CLK);
}

IRAM_ATTR void rul_spi_send_bits(uint64_t data, int bit_count)
{
    for (int i = bit_count - 1; i >= 0; --i) {
        uint8_t bit = (data >> i) & 1;
        // Use the broadcast function to send 'bit' over all data lines
        rul_clock_pulse_with_data_broadcast(bit);
    }
}

/* ----------------- LE (LAT) command helper ----------------- */
IRAM_ATTR void rul_le_command(int clk_count)
{
    GPIO_SET(PIN_LE);
    short_nop_delay(); short_nop_delay(); // Setup time

    for (int i = 0; i < clk_count; ++i) {
        GPIO_CLR(PIN_CLK);
        short_nop_delay(); 
        GPIO_SET(PIN_CLK);
        short_nop_delay(); 
        GPIO_CLR(PIN_CLK);
    }
    
    short_nop_delay(); short_nop_delay(); // Hold time
    GPIO_CLR(PIN_LE);
    short_nop_delay();
}

/* Register write helpers */
static inline void IRAM_ATTR wr_reg1_cmd(void) { rul_le_command(11); }
static inline void IRAM_ATTR wr_reg2_cmd(void) { rul_le_command(12); }

/* Keep last written values for runtime inspection */
static volatile uint16_t last_reg1 = 0xFFFF;
static volatile uint16_t last_reg2 = 0xFFFF;

void write_reg1(uint16_t r_up, uint8_t r_igain, uint8_t r_v0p3, uint8_t r_v1p26)
{
    // 1. Pack the Register
    uint16_t reg1 = (uint16_t)((((r_up & 0x1F) << 11) |
                                ((r_igain & 0x0F) << 7) |
                                ((r_v0p3 & 0x07) << 4) |
                                (r_v1p26 & 0x0F)) & 0xFFFF);

    // 2. Double-Write Strategy
    for(int retry = 0; retry < 2; retry++) {
        // Shift Data to all chips
        for (int i = 0; i < NUM_IC_PER_ROW; ++i) { 
             rul_spi_send_bits((uint64_t)reg1, 16);
        }

        // Latch Command (11 clocks)
        wr_reg1_cmd();

        // Post-Latch Clock Pulse (reliable)
        short_nop_delay();
        GPIO_SET(PIN_CLK);
        short_nop_delay();
        GPIO_CLR(PIN_CLK);
        
        // Short delay between retries
        short_nop_delay(); short_nop_delay(); short_nop_delay();
    }

    last_reg1 = reg1;
}

void write_reg2(uint16_t reg2_value)
{
    for(int retry = 0; retry < 2; retry++) {
        for (int i = 0; i < NUM_IC_PER_ROW; ++i) {
            rul_spi_send_bits((uint64_t)reg2_value, 16);
        }

        wr_reg2_cmd();

        short_nop_delay();
        GPIO_SET(PIN_CLK);
        short_nop_delay();
        GPIO_CLR(PIN_CLK);

        short_nop_delay(); short_nop_delay(); short_nop_delay();
    }

    last_reg2 = reg2_value;
}


// RESET_OEN protocol: datasheet defines LE widths 1 then 2
void reset_oen_protocol(void)
{
    // LE width = 1
    GPIO_SET(PIN_LE);
    asm volatile("" ::: "memory");
    GPIO_CLR(PIN_R0);
    GPIO_CLR(PIN_CLK);
    asm volatile("" ::: "memory");
    GPIO_SET(PIN_CLK);
    short_nop_delay();
    GPIO_CLR(PIN_CLK);
    GPIO_CLR(PIN_LE);
    asm volatile("" ::: "memory");

    // LE width = 2
    GPIO_SET(PIN_LE);
    for (int i = 0; i < 2; ++i) {
        GPIO_CLR(PIN_R0);
        GPIO_CLR(PIN_CLK);
        asm volatile("" ::: "memory");
        GPIO_SET(PIN_CLK);
        short_nop_delay();
        GPIO_CLR(PIN_CLK);
    }
    GPIO_CLR(PIN_LE);
}

/* ----------------- Configure sensible defaults ----------------- */
void configure_min(void)
{
    uint16_t r_up = 0;
    uint8_t r_igain = 0x00;
    uint8_t r_v0p3 = 0x00;
    uint8_t r_v1p26 = 0x00;

    GPIO_SET(PIN_OE); // Force Display OFF (High)
    short_nop_delay();

    write_reg1(r_up, r_igain, r_v0p3, r_v1p26);
    write_reg2(0x0000);
    reset_oen_protocol();

    printf("BRIGHT MIN -> REG1=0x%04X REG2=0x%04X\n", last_reg1, last_reg2);
}

void configure_max(void)
{
    uint16_t r_up = 31;
    uint8_t r_igain = 0x0F;
    uint8_t r_v0p3 = 0x07;
    uint8_t r_v1p26 = 0x0F;

    GPIO_SET(PIN_OE); // Force Display OFF (High)
    short_nop_delay();

    write_reg1(r_up, r_igain, r_v0p3, r_v1p26);
    write_reg2(0x0000);
    reset_oen_protocol();

    printf("BRIGHT MAX -> REG1=0x%04X REG2=0x%04X\n", last_reg1, last_reg2);
}


// -------------------- PANEL MAP FAST --------------------
typedef struct {
    int panel_w, panel_h, N, M;
    uint32_t recip_pw, recip_ph, SHIFT;
    int total_src_w, total_src_h;
} PanelMapFast;
static PanelMapFast pm;

static inline void map_src_to_flat_row_init(PanelMapFast* pm_in, int panel_w, int panel_h, int N, int M) {
    if (!pm_in) return;
    assert(panel_w > 0 && panel_h > 0 && N > 0 && M > 0);
    pm_in->panel_w = panel_w;
    pm_in->panel_h = panel_h;
    pm_in->N = N;
    pm_in->M = M;
    pm_in->total_src_w = panel_w * N;
    pm_in->total_src_h = panel_h * M;
    pm_in->SHIFT = 24u;
    pm_in->recip_pw = (((uint64_t)1 << pm_in->SHIFT) + (uint64_t)panel_w/2) / (uint64_t)panel_w;
    pm_in->recip_ph = (((uint64_t)1 << pm_in->SHIFT) + (uint64_t)panel_h/2) / (uint64_t)panel_h;
}

static inline void map_src_to_flat_row_fast(const PanelMapFast* pm_in, int x, int y, int *dx, int *dy) {
    uint32_t panel_x = (uint32_t)((((uint64_t)x * pm_in->recip_pw) >> pm_in->SHIFT));
    uint32_t panel_y = (uint32_t)((((uint64_t)y * pm_in->recip_ph) >> pm_in->SHIFT));
    if (panel_x >= (uint32_t)pm_in->N) panel_x = pm_in->N - 1;
    if (panel_y >= (uint32_t)pm_in->M) panel_y = pm_in->M - 1;
    int local_x = x - (int)panel_x * pm_in->panel_w;
    int local_y = y - (int)panel_y * pm_in->panel_h;
    uint32_t panel_index = panel_y * pm_in->N + panel_x;
    *dx = local_x + (int)panel_index * pm_in->panel_w;
    *dy = local_y;
}

static inline void map_src_to_flat_row_clamped(const PanelMapFast* pm_in, int x, int y, int *dx, int *dy) {
    if (!pm_in) { *dx = 0; *dy = 0; return; }
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= pm_in->total_src_w) x = pm_in->total_src_w - 1;
    if (y >= pm_in->total_src_h) y = pm_in->total_src_h - 1;
    map_src_to_flat_row_fast(pm_in, x, y, dx, dy);
}

// -------------------- PANEL COORD MAPPER --------------------
typedef struct { uint8_t x, y; } PixelMap;
static inline __attribute__((always_inline)) IRAM_ATTR PixelMap get_panel_map_coord_fast(int local_row, int j) {
    // clamp j to expected intra-panel range
    if (j < 0) j = 0;
    if (j >= pm.panel_w) j = pm.panel_w - 1;

    static const uint8_t y_lut[5] = {0,1,2,3,4};
    int row = local_row % 10;            // 0..9 inside a half
    uint8_t y = y_lut[row % 5];

    unsigned r = (unsigned)j & 3u;
    unsigned G = (unsigned)j >> 2;
    unsigned H = G >> 1;
    unsigned t = G & 1u;
    unsigned base16 = H << 4;
    unsigned top_x = base16 + t * 12u;
    unsigned bottom_x = base16 + (t << 2) + 4u;
    unsigned bottom = (row >= 5) ? 1u : 0u;
    unsigned x = bottom ? (bottom_x + r) : (top_x + r);

    PixelMap p; p.x = (uint8_t)x; p.y = (uint8_t)y;
    return p;
}

// -------------------- FRAME / BITPLANE STORAGE (double-buffered) --------------------
typedef struct { uint8_t r,g,b; } Pixel;
// back/frame buffer we draw into (source coords). We'll keep a single "frame_back" for drawing
static Pixel frame_back[PANEL_H][PANEL_W_TOTAL];

// Two bitplane buffers: buffer 0 and buffer 1. Refresh task reads 'display_buf', drawing updates the other one.
static uint8_t bitplane_buf[BITPLANE_BUFFERS][BITPLANES][SCAN][COLUMNS];
static volatile int display_buf = 0; // index currently being scanned by refresh_task (0 or 1)

// Helper macro to access the back buffer index (the one drawing code should write to)
static inline int get_back_buf_index(void) {
    return 1 - display_buf;
}

// -------------------- PWM / TIMING --------------------
// Color correction is handled by R_gain/G_gain/B_gain BEFORE gamma.
static uint32_t base_unit_us = 500;    // multiplier to convert plane units -> microseconds (tunable)

// default gains and brightness (you asked for max values)
static float R_gain = 1.0f, G_gain = 1.0f, B_gain = 1.0f;
static uint8_t global_brightness = 255; // 0..255, lower for indoor viewing

/* -------------------- 256-value gamma table (gamma = 2.2) --------------- */
static const uint8_t gamma8[256] = {
    /* kept as before */
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,
    1,1,1,1,2,2,2,2,3,3,3,4,4,4,5,5,
    6,6,7,7,8,9,9,10,11,11,12,13,14,15,16,17,
    18,19,20,21,22,23,24,25,27,28,29,31,32,33,35,36,
    38,39,41,42,44,46,47,49,51,53,54,56,58,60,62,64,
    66,68,70,72,74,76,78,81,83,85,87,90,92,94,97,99,
    101,104,106,109,111,114,116,119,121,124,127,129,132,135,137,140,
    143,146,149,151,154,157,160,163,166,169,172,175,178,181,184,188,
    191,194,197,201,204,207,211,214,218,221,224,228,231,235,239,242,
    246,249,253,255,255,255,255,255,255,255,255,255,255,255,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
};

/* -------------------- precomputed 6-bit tables (built at init) --------------- */
static uint8_t gamma6_r[256];
static uint8_t gamma6_g[256];
static uint8_t gamma6_b[256];

/* Compute gamma6 tables from current R_gain/G_gain/B_gain and global_brightness.
   This uses floating math but only runs at init / when config changes. The runtime
   inner loop uses only table lookups (no floating math). */
static void compute_gamma6_tables(void)
{
    const float bright_scale = (float)global_brightness / 255.0f;
    for (int i = 0; i < 256; ++i) {
        float rv = (float)i * R_gain * bright_scale;
        float gv = (float)i * G_gain * bright_scale;
        float bv = (float)i * B_gain * bright_scale;

        if (rv > 255.0f) rv = 255.0f;
        if (gv > 255.0f) gv = 255.0f;
        if (bv > 255.0f) bv = 255.0f;

        uint8_t r8 = gamma8[(int)(rv + 0.5f)];
        uint8_t g8 = gamma8[(int)(gv + 0.5f)];
        uint8_t b8 = gamma8[(int)(bv + 0.5f)];

        gamma6_r[i] = (uint8_t)((((uint32_t)r8 * 63U) + 127U) / 255U);
        gamma6_g[i] = (uint8_t)((((uint32_t)g8 * 63U) + 127U) / 255U);
        gamma6_b[i] = (uint8_t)((((uint32_t)b8 * 63U) + 127U) / 255U);
    }
}

/* API to update gains/brightness at runtime (will recompute tables) */
void set_gains_and_brightness(float Rg, float Gg, float Bg, uint8_t brightness)
{
    if (Rg <= 0.0f) Rg = 1.0f;
    if (Gg <= 0.0f) Gg = 1.0f;
    if (Bg <= 0.0f) Bg = 1.0f;
    R_gain = Rg; G_gain = Gg; B_gain = Bg;
    global_brightness = brightness;
    compute_gamma6_tables();
}

/* -------------------- IRAM fast memzero (used for clearing back buffer) -------------------- */
IRAM_ATTR static inline void fast_memzero32(void *buf, size_t size_bytes)
{
    if (buf == NULL || size_bytes == 0) return;

    uint32_t *p = (uint32_t *)buf;
    uint32_t *end = (uint32_t *)((uint8_t *)buf + (size_bytes & ~(size_t)3));

    // Unrolled clear: 8 words per loop
    while (p + 8 <= end) {
        p[0] = 0; p[1] = 0; p[2] = 0; p[3] = 0;
        p[4] = 0; p[5] = 0; p[6] = 0; p[7] = 0;
        p += 8;
    }
    while (p < end) *p++ = 0;

    // handle remaining tail bytes (if size not multiple of 4)
    size_t rem = size_bytes & 3;
    if (rem) {
        uint8_t *tail = (uint8_t *)((uint8_t *)buf + (size_bytes - rem));
        for (size_t i = 0; i < rem; ++i) tail[i] = 0;
    }
}

/* Clear back buffers (frame_back and bitplane back buffer) */
static inline void clear_back_buffers_fast(void)
{
    // clear frame_back
    fast_memzero32((void*)frame_back, sizeof(frame_back));
    // clear back bitplane buffer
    int b = get_back_buf_index();
    fast_memzero32((void*)bitplane_buf[b], sizeof(bitplane_buf[b]));
}

/* -------------------- BITPLANE BUILD / UPDATE (write into back buffer) -------------------- */
IRAM_ATTR static inline void update_pixel_bitplanes_back(int dx, int dy) {
    if ((unsigned)dx >= (unsigned)PANEL_W_TOTAL || (unsigned)dy >= (unsigned)PANEL_H) return;

    int back = get_back_buf_index();

    // which panel along the flattened row (0 .. N*M-1)
    int panel_index = dx / pm.panel_w;
    int lx = dx - panel_index * pm.panel_w;    // local x inside panel (0..panel_w-1)

    // local row inside panel (0..panel_h-1)
    int local_row = dy % pm.panel_h;
    bool lower = (local_row < (pm.panel_h / 2));

    PixelMap p = get_panel_map_coord_fast(local_row, lx);
    if (p.y >= SCAN) return;

    // convert intra-panel column (p.x) into global shift-register column index
    int shift_cols_per_panel = COLUMNS / (pm.N * pm.M); // should be 80 in your setup
    int global_col = panel_index * shift_cols_per_panel + (int)p.x;

    int row = p.y;
    int col = global_col; // index into bitplane[plane][row][col]

    Pixel src = frame_back[dy][dx];

    // Use precomputed gamma6 tables (no FP math here)
    uint8_t R6 = gamma6_r[src.r];
    uint8_t G6 = gamma6_g[src.g];
    uint8_t B6 = gamma6_b[src.b];

    for (int plane = 0; plane < BITPLANES; ++plane) {
        uint8_t clear_mask = lower ? 0x07 : 0x38;
        bitplane_buf[back][plane][row][col] &= (uint8_t)~clear_mask;

        uint8_t rbit = (R6 >> plane) & 1;
        uint8_t gbit = (G6 >> plane) & 1;
        uint8_t bbit = (B6 >> plane) & 1;
        uint8_t patt = lower ? ((rbit<<0)|(gbit<<1)|(bbit<<2)) : ((rbit<<3)|(gbit<<4)|(bbit<<5));

        bitplane_buf[back][plane][row][col] |= patt;
    }
}

/* Rebuild all bitplanes into back buffer from frame_back */
static void rebuild_all_bitplanes_back(void) {
    int back = get_back_buf_index();

    // clear back bit plane 
    fast_memzero32((void*)bitplane_buf[back], sizeof(bitplane_buf[back]));

    // Build bitplanes
    for (int y = 0; y < PANEL_H; ++y) {
        for (int x = 0; x < PANEL_W_TOTAL; ++x) {
            update_pixel_bitplanes_back(x, y);
        }
    }
}

/* -------------------- SWAP / SYNCHRONIZATION -------------------- */
static  portMUX_TYPE swap_mux = portMUX_INITIALIZER_UNLOCKED;

void swap_bitplanes(void)
{
    taskENTER_CRITICAL(&swap_mux);
    display_buf = 1 - display_buf;
    taskEXIT_CRITICAL(&swap_mux);
}

/* -------------------- REFRESH TASK -------------------- */
static void refresh_task(void *arg){
    (void)arg;
    while (1) 
    {
        int buf = display_buf;
        for (int row = 0; row < SCAN; ++row) 
        {
            set_pin(MASK_A, row & 1);
            set_pin(MASK_B, (row >> 1) & 1);
            set_pin(MASK_C, (row >> 2) & 1);

            for (int plane = 0; plane < BITPLANES; ++plane) 
            {
                // shift out columns
                for (int col = 0; col < COLUMNS; ++col) {
                    gpio_out_set_levels(bitplane_buf[buf][plane][row][col]);
                    pulse_clk();
                    if (col == COLUMNS - 4) set_pin(MASK_LE, 1);
                }
                set_pin(MASK_LE, 0);

                // enable output for the computed time slice
                set_pin(MASK_OE, 0);

                // simple uniform plane timing; you can replace with weighted time per plane later
                esp_rom_delay_us(base_unit_us);

                set_pin(MASK_OE, 1);
                short_nop_delay();
            }
        }
    }
}

/* -------------------- API -------------------- */
void set_pixel_rgb_back(int sx, int sy, uint8_t r, uint8_t g, uint8_t b) {
    if (pm.total_src_w == 0 || pm.total_src_h == 0) return;
    int dx = 0, dy = 0;
    map_src_to_flat_row_clamped(&pm, sx, sy, &dx, &dy);
    if ((unsigned)dx >= (unsigned)PANEL_W_TOTAL || (unsigned)dy >= (unsigned)PANEL_H) return;
    frame_back[dy][dx].r = r;
    frame_back[dy][dx].g = g;
    frame_back[dy][dx].b = b;
    update_pixel_bitplanes_back(dx, dy);
}

/* Convenience API to prepare a new frame */
void prepare_frame_back(void) {
    clear_back_buffers_fast();
}

void present_frame_back(void) {
    // Rebuild bitplanes fully (ensures any pixels not individually updated are correct)
    rebuild_all_bitplanes_back();

    // atomically swap the back buffer into view
    swap_bitplanes();
}

void draw_char_back(int sx, int sy, char c, uint8_t r, uint8_t g, uint8_t b)
{
    if (c < 32 || c > 126) return;

    // Each glyph is [9][6]
    const uint8_t (*glyph)[6] = font6x9[c - 32];

    for (int row = 0; row < 9; row++) {
        for (int col = 0; col < 6; col++) {
            if (glyph[row][col]) {
                set_pixel_rgb_back(sx + col, sy + row, r, g, b);
            }
            // else: background transparent
        }
    }
}

void draw_text_back(int sx, int sy, const char *str, uint8_t r, uint8_t g, uint8_t b)
{
    int x = sx;
    while (*str) {
        draw_char_back(x, sy, *str, r, g, b);
        x += 7;  // 6px glyph + 1px space
        str++;
    }
}

/* clear a rectangular region in source coordinates in the back buffer */
void clear_region_back(int sx, int sy, int w, int h)
{
    if (pm.total_src_w == 0 || pm.total_src_h == 0) return;

    // clamp to source area
    if (sx < 0) { w += sx; sx = 0; }
    if (sy < 0) { h += sy; sy = 0; }
    if (sx >= pm.total_src_w || sy >= pm.total_src_h) return;
    if (sx + w > pm.total_src_w) w = pm.total_src_w - sx;
    if (sy + h > pm.total_src_h) h = pm.total_src_h - sy;
    if (w <= 0 || h <= 0) return;

    for (int y = sy; y < sy + h; ++y) {
        for (int x = sx; x < sx + w; ++x) {
            int dx = 0, dy = 0;
            map_src_to_flat_row_clamped(&pm, x, y, &dx, &dy);
            // clear frame pixel in back buffer
            frame_back[dy][dx].r = 0;
            frame_back[dy][dx].g = 0;
            frame_back[dy][dx].b = 0;
            // update back bitplanes for this pixel
            update_pixel_bitplanes_back(dx, dy);
        }
    }
}

/* -------------------- DRAWING TASK (example counter) -------------------- */
void drawing_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(50));   // allow refresh_task to start

    int counter = 0;
    char buf[50];   // buffer size used for snprintf
    char buf2[50];

    while (1) {
        // Clear + rebuild back framebuffer
        prepare_frame_back();

        // Format counter 0–999 (safely)
        snprintf(buf, sizeof(buf), "%011d", counter);
        snprintf(buf2, sizeof(buf2), "HELLO WORLD");

        // Draw text
        draw_text_back(2, 10, buf, 255, 255, 255);  // white

        int idx = counter % 3;
        switch (idx)
        {
            case 0:
                draw_text_back(2, 0, buf2, 255, 0, 0); // red
                break;
            case 1:
                draw_text_back(2, 0, buf2, 0, 255, 0); // green
                break;
            case 2:
                draw_text_back(2, 0, buf2, 0, 0, 255); // blue
                break;
        }

        // Swap buffers safely
        present_frame_back();

        // Increment & wrap
        counter++;
        if (counter >= 1234567899) counter = 0;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void init_max_brightness()
{
    // Rapidly toggle the Latch (LE) pin to force chips to exit random power-on sequences.
    for (int i = 0; i < 10; i++) {
        GPIO_SET(PIN_LE);
        GPIO_CLR(PIN_LE);
        for(int j=0; j<10; j++) short_nop_delay(); 
    }

    // Aggressive POR Cleanup: Run configure_min 3 times
    for (int i = 0; i < 3; i++) {
        configure_min(); 
        for(int j=0; j<5000; j++) short_nop_delay(); 
    }
    
    // Load Final Configuration
    configure_max();
}

/* -------------------- MAIN -------------------- */
void app_main(void)
{
    init_pins();

    // initialize mapping: N = panels across, M = panels down
    map_src_to_flat_row_init(&pm, PHYSICAL_PANEL_WIDTH, PHYSICAL_PANEL_HEIGHT, PANELS_ACROSS, PANELS_DOWN);

    // sanity asserts
    assert(pm.total_src_w == PHYSICAL_PANEL_WIDTH * pm.N);
    assert(pm.total_src_h == PHYSICAL_PANEL_HEIGHT * pm.M);
    assert(PANEL_W_TOTAL == pm.panel_w * pm.N * pm.M);

    // compute gamma6 tables with current gains/brightness
    compute_gamma6_tables();

    // clear both bitplane buffers and frame_back
    fast_memzero32((void*)bitplane_buf, sizeof(bitplane_buf));
    fast_memzero32((void*)frame_back, sizeof(frame_back));

    //general brightness/time base
    base_unit_us = 500;    
    init_max_brightness();    
    
    // start refresh task on core 0
    xTaskCreatePinnedToCore(refresh_task, "refresh_task", 4096, NULL, 1, NULL, 0);   
    // start drawing task on core 1	
    xTaskCreatePinnedToCore(drawing_task, "drawing_task", 8192, NULL, 1, NULL, 1);
    
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}


