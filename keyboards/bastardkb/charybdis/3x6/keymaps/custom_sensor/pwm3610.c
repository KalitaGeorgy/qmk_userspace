#include "quantum.h"
#include "pointing_device.h"
#include "gpio.h"
#include "wait.h"

#ifndef PMW3610_SCLK_PIN
#    error "PMW3610_SCLK_PIN is not defined"
#endif
#ifndef PMW3610_SDIO_PIN
#    error "PMW3610_SDIO_PIN is not defined"
#endif
#ifndef PMW3610_CS_PIN
#    error "PMW3610_CS_PIN is not defined"
#endif

#ifndef PMW3610_CPI_DEFAULT
#    define PMW3610_CPI_DEFAULT 1200
#endif
#ifndef PMW3610_CPI_MIN
#    define PMW3610_CPI_MIN 200
#endif
#ifndef PMW3610_CPI_MAX
#    define PMW3610_CPI_MAX 3200
#endif

// Page 0
#define REG_PROD_ID            0x00
#define REG_MOTION             0x02
#define REG_DELTA_X_L          0x03
#define REG_DELTA_Y_L          0x04
#define REG_DELTA_XY_H         0x05
#define REG_PERFORMANCE        0x11
#define REG_BURST_READ         0x12
#define REG_RUN_DOWNSHIFT      0x1B
#define REG_REST1_RATE         0x1C
#define REG_REST1_DOWNSHIFT    0x1D
#define REG_OBSERVATION1       0x2D
#define REG_POWER_UP_RESET     0x3A
#define REG_SPI_CLK_ON_REQ     0x41
#define REG_SPI_PAGE           0x7F

// Page 1
#define REG_RES_STEP           0x05

#define PMW3610_PRODUCT_ID     0x3E
#define PMW3610_SPI_WRITE      0x80
#define PMW3610_SPI_CLK_ON     0xBA
#define PMW3610_SPI_CLK_OFF    0xB5

static uint16_t pmw3610_cpi = PMW3610_CPI_DEFAULT;

static inline void sclk_hi(void) { gpio_write_pin_high(PMW3610_SCLK_PIN); }
static inline void sclk_lo(void) { gpio_write_pin_low(PMW3610_SCLK_PIN);  }
static inline void cs_hi(void)   { gpio_write_pin_high(PMW3610_CS_PIN);   }
static inline void cs_lo(void)   { gpio_write_pin_low(PMW3610_CS_PIN);    }

static inline void sdio_out(void)       { gpio_set_pin_output(PMW3610_SDIO_PIN); }
static inline void sdio_in(void)        { gpio_set_pin_input(PMW3610_SDIO_PIN);   }
static inline void sdio_hi(void)        { gpio_write_pin_high(PMW3610_SDIO_PIN);  }
static inline void sdio_lo(void)        { gpio_write_pin_low(PMW3610_SDIO_PIN);   }
static inline bool sdio_read(void)      { return gpio_read_pin(PMW3610_SDIO_PIN); }

static inline void spi_delay(void) {
    wait_us(1);
}

// Порядок фронтов тут сделан как стартовая реализация.
// Если PROD_ID не читается при гарантированно верной пайке и питании,
// первым делом меняй именно фазу в этих двух функциях.
static void pmw3610_write_u8(uint8_t data) {
    sdio_out();
    for (int8_t i = 7; i >= 0; --i) {
        sclk_lo();
        if (data & (1 << i)) {
            sdio_hi();
        } else {
            sdio_lo();
        }
        spi_delay();
        sclk_hi();
        spi_delay();
    }
}

static uint8_t pmw3610_read_u8(void) {
    uint8_t data = 0;
    sdio_in();

    for (int8_t i = 7; i >= 0; --i) {
        sclk_lo();
        spi_delay();
        sclk_hi();
        if (sdio_read()) {
            data |= (1 << i);
        }
        spi_delay();
    }

    return data;
}

static void pmw3610_write_reg(uint8_t reg, uint8_t val) {
    cs_lo();
    spi_delay();
    pmw3610_write_u8(reg | PMW3610_SPI_WRITE);
    pmw3610_write_u8(val);
    spi_delay();
    cs_hi();
    wait_us(20);
}

static uint8_t pmw3610_read_reg(uint8_t reg) {
    uint8_t val;
    cs_lo();
    spi_delay();
    pmw3610_write_u8(reg & 0x7F);
    val = pmw3610_read_u8();
    spi_delay();
    cs_hi();
    wait_us(20);
    return val;
}

static void pmw3610_spi_clk_on(void) {
    pmw3610_write_reg(REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLK_ON);
    wait_us(300);
}

static void pmw3610_spi_clk_off(void) {
    pmw3610_write_reg(REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLK_OFF);
}

static int16_t sign_extend_12(uint16_t v) {
    if (v & 0x0800) {
        v |= 0xF000;
    }
    return (int16_t)v;
}

static bool pmw3610_set_cpi_internal(uint16_t cpi) {
    if (cpi < PMW3610_CPI_MIN) cpi = PMW3610_CPI_MIN;
    if (cpi > PMW3610_CPI_MAX) cpi = PMW3610_CPI_MAX;

    // шаг 200 CPI
    uint8_t res = (uint8_t)(cpi / 200);
    if (res > 0x1F) {
        res = 0x1F;
    }

    pmw3610_spi_clk_on();
    pmw3610_write_reg(REG_SPI_PAGE, 0xFF);    // page 1

    uint8_t val = pmw3610_read_reg(REG_RES_STEP);
    val &= ~0x1F;
    val |= res;

    pmw3610_write_reg(REG_RES_STEP, val);
    pmw3610_write_reg(REG_SPI_PAGE, 0x00);    // back to page 0
    pmw3610_spi_clk_off();

    pmw3610_cpi = cpi;
    return true;
}

static bool pmw3610_read_motion(int16_t *dx, int16_t *dy) {
    uint8_t motion = pmw3610_read_reg(REG_MOTION);
    if ((motion & 0x80) == 0) {
        *dx = 0;
        *dy = 0;
        return false;
    }

    // Даташит требует читать 0x02..0x05 последовательно хотя бы в init,
    // и именно эти регистры содержат motion/x/y/high bits.
    uint8_t x_l  = pmw3610_read_reg(REG_DELTA_X_L);
    uint8_t y_l  = pmw3610_read_reg(REG_DELTA_Y_L);
    uint8_t xy_h = pmw3610_read_reg(REG_DELTA_XY_H);

    uint16_t raw_x = (((uint16_t)(xy_h >> 4)) << 8) | x_l;
    uint16_t raw_y = (((uint16_t)(xy_h & 0x0F)) << 8) | y_l;

    *dx = sign_extend_12(raw_x);
    *dy = sign_extend_12(raw_y);
    return true;
}

bool pointing_device_driver_init(void) {
    gpio_set_pin_output(PMW3610_SCLK_PIN);
    gpio_set_pin_output(PMW3610_CS_PIN);
    sdio_out();

    cs_hi();
    sclk_hi();
    sdio_hi();
    wait_ms(1);

    // Сброс/подготовка SPI-порта по даташиту
    cs_hi();
    wait_us(10);
    cs_lo();
    wait_us(160);
    cs_hi();
    wait_us(10);

    // power-up reset
    pmw3610_write_reg(REG_POWER_UP_RESET, 0x5A);
    wait_ms(10);

    uint8_t prod = pmw3610_read_reg(REG_PROD_ID);
    if (prod != PMW3610_PRODUCT_ID) {
        return false;
    }

    pmw3610_spi_clk_on();

    pmw3610_write_reg(REG_OBSERVATION1, 0x00);
    wait_ms(10);

    uint8_t obs = pmw3610_read_reg(REG_OBSERVATION1);
    if ((obs & 0x0F) != 0x0F) {
        pmw3610_spi_clk_off();
        return false;
    }

    // обязательные reads после power-up
    (void)pmw3610_read_reg(REG_MOTION);
    (void)pmw3610_read_reg(REG_DELTA_X_L);
    (void)pmw3610_read_reg(REG_DELTA_Y_L);
    (void)pmw3610_read_reg(REG_DELTA_XY_H);

    pmw3610_write_reg(REG_PERFORMANCE,     0x0D);
    pmw3610_write_reg(REG_RUN_DOWNSHIFT,   0x04);
    pmw3610_write_reg(REG_REST1_RATE,      0x04);
    pmw3610_write_reg(REG_REST1_DOWNSHIFT, 0x0F);

    pmw3610_spi_clk_off();

    return pmw3610_set_cpi_internal(PMW3610_CPI_DEFAULT);
}

report_mouse_t pointing_device_driver_get_report(report_mouse_t mouse_report) {
    int16_t dx = 0;
    int16_t dy = 0;

    if (pmw3610_read_motion(&dx, &dy)) {
        mouse_report.x = dx;
        mouse_report.y = dy;
    }

    return mouse_report;
}

uint16_t pointing_device_driver_get_cpi(void) {
    return pmw3610_cpi;
}

void pointing_device_driver_set_cpi(uint16_t cpi) {
    pmw3610_set_cpi_internal(cpi);
}
