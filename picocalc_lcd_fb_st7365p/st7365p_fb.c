// SPDX-License-Identifier: GPL-2.0
/*
 * Framebuffer driver for Sitronix ST7365P SPI panels as fitted to the PicoCalc
 *
 * Copyright 2025 Calculinux
 *
 * Adapted from the PicoCalc ILI9488 framebuffer driver
 * (drivers/staging/picocalc/picocalc_lcd_fb/ili9488_fb.c)
 * Copyright embeddedboys <writeforever@foxmail.com>, mod by hiro
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>

#include <linux/wait.h>
#include <linux/spinlock.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/regmap.h>

#include <linux/uaccess.h>

#include <linux/fb.h>
#include <linux/fbcon.h>
#include <video/mipi_display.h>

#define DRV_NAME "st7365p_drv"

/*
 * ST7365P register set. The ST7365P shares the ILI9488 register map, so the
 * command numbers match the ones used by the ILI9488 driver; the names follow
 * the ST7365P datasheet.
 */
#define ST7365P_SLPOUT      0x11    /* Sleep out */
#define ST7365P_INVON       0x21    /* Display inversion on */
#define ST7365P_DISPON      0x29    /* Display on */
#define ST7365P_CASET       0x2A    /* Column address set */
#define ST7365P_RASET       0x2B    /* Row address set */
#define ST7365P_RAMWR       0x2C    /* Memory write */
#define ST7365P_MADCTL      0x36    /* Memory data access control */
#define ST7365P_COLMOD      0x3A    /* Interface pixel format */
#define ST7365P_IFMODE      0xB0    /* Interface mode control */
#define ST7365P_FRMCTR1     0xB1    /* Frame rate control, normal mode */
#define ST7365P_INVTR       0xB4    /* Display inversion control */
#define ST7365P_DISCTRL     0xB6    /* Display function control */
#define ST7365P_ETMOD       0xB7    /* Entry mode set */
#define ST7365P_PWCTRL1     0xC0    /* Power control 1 */
#define ST7365P_PWCTRL2     0xC1    /* Power control 2 */
#define ST7365P_VMCTRL      0xC5    /* VCOM control */
#define ST7365P_PGAMCTRL    0xE0    /* Positive gamma correction */
#define ST7365P_NGAMCTRL    0xE1    /* Negative gamma correction */
#define ST7365P_IMGFUNC     0xE9    /* Set image function */
#define ST7365P_ADJCTRL3    0xF7    /* Adjust control 3 */

/* COLMOD pixel formats, same value for the DBI and DPI nibbles */
#define ST7365P_COLMOD_16BIT    0x55
#define ST7365P_COLMOD_3BIT     0x22

/* MADCTL bits, the PicoCalc panel is wired for MX | BGR */
#define ST7365P_MADCTL_BGR  BIT(3)
#define ST7365P_MADCTL_MV   BIT(5)
#define ST7365P_MADCTL_MX   BIT(6)
#define ST7365P_MADCTL_MY   BIT(7)

static int p_3bit_mode = 0;
module_param(p_3bit_mode, int, 0660);

static int p_dither = 0;
module_param(p_dither, int, 0660);

/*
 * Pixel data and register writes are clocked separately. The init sequence and
 * the per-refresh window setup are short bursts where a glitch corrupts the
 * panel state, so they stay at a conservative rate while the bulk pixel
 * transfer runs at whatever the bus can actually sustain.
 */
static unsigned int p_spi_speed_hz;
module_param(p_spi_speed_hz, uint, 0444);
MODULE_PARM_DESC(p_spi_speed_hz,
                 "Override the SPI clock used for pixel data, in Hz (0: use spi-max-frequency)");

static unsigned int p_cmd_speed_hz = 10000000;
module_param(p_cmd_speed_hz, uint, 0444);
MODULE_PARM_DESC(p_cmd_speed_hz,
                 "SPI clock used for register writes, in Hz");

static bool p_bpw16 = true;
module_param(p_bpw16, bool, 0444);
MODULE_PARM_DESC(p_bpw16,
                 "Push RGB565 as 16-bit SPI words straight from the framebuffer when the controller supports it");

struct st7365p_par;

struct st7365p_operations {
    int (*reset)(struct st7365p_par *par);
    int (*clear)(struct st7365p_par *par);
    int (*idle)(struct st7365p_par *par, bool on);
    int (*blank)(struct st7365p_par *par, bool on);
    int (*sleep)(struct st7365p_par *par, bool on);
    int (*set_addr_win)(struct st7365p_par *par, int xs, int ys, int xe, int ye);
};

struct st7365p_display {
    u32                     xres;
    u32                     yres;
    u32                     bpp;
    u32                     fps;
    u32                     rotate;
    u32                     xs_off;
    u32                     xe_off;
    u32                     ys_off;
    u32                     ye_off;

    char *gamma;
    int gamma_num;
    int gamma_len;
};

struct st7365p_par {

    struct device           *dev;
    struct spi_device       *spi;
    struct spi_transfer     *spi_3_xfers;

    u8                      *buf;
    struct {
        void *buf;
        size_t len;
    } txbuf;
    struct {
        struct gpio_desc *rst;
        struct gpio_desc *dc;
        struct gpio_desc *cs;
    } gpio;


    spinlock_t              dirty_lock;
    struct completion       complete;

    /* device specific */
    u32                     refr_mode;
    u32                     wait;
    u32                     busy;

    /* SPI clocking: registers are written slowly, pixels as fast as we can */
    u32                     px_speed_hz;
    u32                     cmd_speed_hz;
    size_t                  max_xfer;
    bool                    bpw16;

    const struct st7365p_operations        *tftops;
    const struct st7365p_display           *display;

    struct fb_info          *fbinfo;
    struct fb_ops           *fbops;

    u32             pseudo_palette[16];

    u32             dirty_lines_start;
    u32             dirty_lines_end;
};

#define gpio_put(d, v) gpiod_set_raw_value(d, v)

/*
 * A single transfer at an explicit clock rate. spi_write() would run everything
 * at spi->max_speed_hz, which is exactly what we do not want once the bus is
 * clocked past the rate the panel accepts for register writes.
 */
static int st7365p_spi_xfer(struct st7365p_par *par, const void *buf, size_t len,
                            u32 speed_hz, u8 bits_per_word)
{
    struct spi_transfer xfer = {
        .tx_buf = buf,
        .len = len,
        .speed_hz = speed_hz,
        .bits_per_word = bits_per_word,
    };
    struct spi_message msg;

    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);

    return spi_sync(par->spi, &msg);
}

/* Register writes and their parameters, kept at the slow clock */
static int st7365p_write_cmd(struct st7365p_par *par, const void *buf, size_t len)
{
    return st7365p_spi_xfer(par, buf, len, par->cmd_speed_hz, 8);
}

/*
 * Bulk pixel data at the fast clock, split into whatever the controller will
 * take in one go. A 320x320 RGB565 full-screen update is 200KiB, which is more
 * than some SPI controllers accept as a single transfer.
 */
static int st7365p_write_pixels(struct st7365p_par *par, const void *buf, size_t len,
                                u8 bits_per_word)
{
    const u8 *pos = buf;
    size_t chunk;
    int rc;

    while (len) {
        chunk = min(len, par->max_xfer);

        /* never split in the middle of a 16-bit word */
        if (bits_per_word == 16)
            chunk &= ~(size_t)1;

        rc = st7365p_spi_xfer(par, pos, chunk, par->px_speed_hz, bits_per_word);
        if (rc)
            return rc;

        pos += chunk;
        len -= chunk;
    }

    return 0;
}

static inline void fbtft_write_buf_dc(struct st7365p_par *par, void *buf, size_t len, int dc)
{
    gpio_put(par->gpio.dc, dc);
    st7365p_write_cmd(par, buf, len);
}

#define NUMARGS(...)  (sizeof((int[]){__VA_ARGS__}) / sizeof(int))
static int st7365p_write_reg(struct st7365p_par *par, int len, ...)
{
    u8 *buf = (u8 *)par->buf;
    va_list args;
    int i;

    va_start(args, len);

    *buf = (u8)va_arg(args, unsigned int);
    fbtft_write_buf_dc(par, buf, sizeof(u8), 0);
    len--;

    /* if there no params */
    if (len == 0)
        goto exit_no_param;

    for (i = 0; i < len; i++)
        *buf++ = (u8)va_arg(args, unsigned int);

    fbtft_write_buf_dc(par, par->buf, len, 1);
    va_end(args);

exit_no_param:
    va_end(args);
    return 0;
}
#define write_reg(par, ...) \
    st7365p_write_reg(par, NUMARGS(__VA_ARGS__), __VA_ARGS__)

static int st7365p_reset(struct st7365p_par *par)
{
    gpio_put(par->gpio.rst, 1);
    mdelay(10);
    gpio_put(par->gpio.rst, 0);
    mdelay(10);
    gpio_put(par->gpio.rst, 1);
    mdelay(10);
    return 0;
}

static int st7365p_init_display(struct st7365p_par *priv)
{
    st7365p_reset(priv);

    gpio_put(priv->gpio.cs, 0);
    write_reg(priv, ST7365P_PGAMCTRL,
              0x00, 0x03, 0x09, 0x08, 0x16, 0x0A, 0x3F, 0x78,
              0x4C, 0x09, 0x0A, 0x08, 0x16, 0x1A, 0x0F);
    write_reg(priv, ST7365P_NGAMCTRL,
              0x00, 0x16, 0x19, 0x03, 0x0F, 0x05, 0x32, 0x45,
              0x46, 0x04, 0x0E, 0x0D, 0x35, 0x37, 0x0F);

    write_reg(priv, ST7365P_PWCTRL1, 0x17, 0x15);
    write_reg(priv, ST7365P_PWCTRL2, 0x41);
    write_reg(priv, ST7365P_VMCTRL, 0x00, 0x12, 0x80);
    write_reg(priv, ST7365P_MADCTL, ST7365P_MADCTL_MX | ST7365P_MADCTL_BGR);

    /*
     * The ST7365P takes RGB565 straight over SPI, so 16 bit is the normal
     * mode here. 3 bit mode stays available as a bandwidth escape hatch.
     */
    if (p_3bit_mode)
        write_reg(priv, ST7365P_COLMOD, ST7365P_COLMOD_3BIT);
    else
        write_reg(priv, ST7365P_COLMOD, ST7365P_COLMOD_16BIT);

    write_reg(priv, ST7365P_IFMODE, 0x00);

    /* fosc/2 with 17 clocks per line */
    write_reg(priv, ST7365P_FRMCTR1, 0xD0, 0x11);

    write_reg(priv, ST7365P_INVON);
    write_reg(priv, ST7365P_INVTR, 0x02);
    write_reg(priv, ST7365P_DISCTRL, 0x02, 0x02, 0x3B);
    write_reg(priv, ST7365P_ETMOD, 0xC6);
    write_reg(priv, ST7365P_IMGFUNC, 0x00);
    write_reg(priv, ST7365P_ADJCTRL3, 0xA9, 0x51, 0x2C, 0x82);
    write_reg(priv, ST7365P_SLPOUT);
    mdelay(120);
    write_reg(priv, ST7365P_DISPON);
    mdelay(120);
    gpio_put(priv->gpio.cs, 1);

    return 0;
}

static int st7365p_blank(struct st7365p_par *par, bool on)
{
    gpio_put(par->gpio.cs, 0);
    if (on)
        write_reg(par, MIPI_DCS_SET_DISPLAY_OFF);
    else
        write_reg(par, MIPI_DCS_SET_DISPLAY_ON);
    gpio_put(par->gpio.cs, 1);
    return 0;
}

static int st7365p_set_addr_win(struct st7365p_par *par, int xs, int ys, int xe,
                                int ye)
{
    dev_dbg(par->dev, "xs = %d, xe = %d, ys = %d, ye = %d\n", xs, xe, ys, ye);

    write_reg(par, ST7365P_CASET,
              ((xs >> BITS_PER_BYTE)), (xs & 0xFF),
              ((xe >> BITS_PER_BYTE)), (xe & 0xFF));

    write_reg(par, ST7365P_RASET,
              ((ys >> BITS_PER_BYTE)), (ys & 0xFF),
              ((ye >> BITS_PER_BYTE)), (ye & 0xFF));

    write_reg(par, ST7365P_RAMWR);

    return 0;
}

// static int st7365p_idle(struct st7365p_par *par, bool on)
// {
//     if (on)
//         write_reg(par, MIPI_DCS_EXIT_IDLE_MODE);
//     else
//         write_reg(par, MIPI_DCS_EXIT_IDLE_MODE);

//     return 0;
// }

// static int st7365p_sleep(struct st7365p_par *par, bool on)
// {
//     if (on) {
//         write_reg(par, MIPI_DCS_SET_DISPLAY_OFF);
//         write_reg(par, MIPI_DCS_ENTER_SLEEP_MODE);
//     } else {
//         write_reg(par, MIPI_DCS_EXIT_SLEEP_MODE);
//         write_reg(par, MIPI_DCS_SET_DISPLAY_ON);
//     }

//     return 0;
// }

static int st7365p_clear(struct st7365p_par *priv)
{
    u32 width = priv->display->xres;
    u32 height = priv->display->yres;
    size_t len;

    /* 3 bit mode packs two pixels into a byte, 16 bit mode takes two per pixel */
    len = p_3bit_mode ? (width * height) / 2 : width * height * 2;
    if (len > priv->txbuf.len)
        len = priv->txbuf.len;

    memset(priv->txbuf.buf, 0, len);

    gpio_put(priv->gpio.cs, 0);
    st7365p_set_addr_win(priv, 0, 0, width, height);

    gpio_put(priv->gpio.dc, 1);
    st7365p_write_pixels(priv, priv->txbuf.buf, len, 8);
    gpio_put(priv->gpio.cs, 1);

    return 0;
}

static const struct st7365p_operations default_st7365p_ops = {
    // .idle  = st7365p_idle,
    .clear = st7365p_clear,
    // .blank = st7365p_blank,
    .reset = st7365p_reset,
    // .sleep = st7365p_sleep,
    .set_addr_win = st7365p_set_addr_win,
};

static int st7365p_request_one_gpio(struct st7365p_par *par,
                                    const char *name, int index,
                                    struct gpio_desc **gpiop)
{
    struct device *dev = par->dev;
    struct device_node *np = dev->of_node;
    int gpio, flags, rc = 0;
    enum of_gpio_flags of_flags;

    if (of_find_property(np, name, NULL)) {
        gpio = of_get_named_gpio_flags(np, name, index, &of_flags);
        if (gpio == -ENOENT)
            return 0;
        if (gpio == -EPROBE_DEFER)
            return gpio;
        if (gpio < 0) {
            dev_err(dev,
                    "failed to get '%s' from DT\n", name);
            return gpio;
        }

        flags = (of_flags & OF_GPIO_ACTIVE_LOW) ? GPIOF_OUT_INIT_LOW :
                GPIOF_OUT_INIT_HIGH;
        rc = devm_gpio_request_one(dev, gpio, flags,
                                   dev->driver->name);
        if (rc) {
            dev_err(dev,
                    "gpio_request_one('%s'=%d) failed with %d\n",
                    name, gpio, rc);
            return rc;
        }
        if (gpiop)
            *gpiop = gpio_to_desc(gpio);
        pr_debug("%s : '%s' = GPIO%d\n",
                 __func__, name, gpio);
    }

    return rc;
}

static int st7365p_request_gpios(struct st7365p_par *par)
{
    int rc;
    rc = st7365p_request_one_gpio(par, "rst", 0, &par->gpio.rst);
    if (rc)
        return rc;
    rc = st7365p_request_one_gpio(par, "dc", 0, &par->gpio.dc);
    if (rc)
        return rc;
    rc = st7365p_request_one_gpio(par, "cs", 0, &par->gpio.cs);
    if (rc)
        return rc;

    return 0;
}

/* returns 0 if the property is not present */
static u32 __maybe_unused fbtft_property_value(struct device *dev, const char *propname)
{
    int ret;
    u32 val = 0;

    ret = device_property_read_u32(dev, propname, &val);
    if (ret == 0)
        dev_info(dev, "%s: %s = %u\n", __func__, propname, val);

    return val;
}

static int st7365p_of_config(struct st7365p_par *par)
{
    int rc;

    rc = st7365p_request_gpios(par);
    if (rc) {
        dev_err(par->dev, "Request gpios failed!\n");
        return rc;
    }
    return 0;

    /* request xres and yres from dt */
}

// #define MADCTL_BGR BIT(3) /* bitmask for RGB/BGR order */
// #define MADCTL_MV BIT(5) /* bitmask for page/column order */
// #define MADCTL_MX BIT(6) /* bitmask for column address order */
// #define MADCTL_MY BIT(7) /* bitmask for page address order */
// static int st7365p_set_var(struct st7365p_par *par)
// {
//     u8 madctl_par = 0;

//     switch (par->fbinfo->var.rotate) {
//     case 0:
//         break;
//     case 90:
//         madctl_par |= (MADCTL_MV | MADCTL_MY);
//         break;
//     case 180:
//         madctl_par |= (MADCTL_MX | MADCTL_MY);
//         break;
//     case 270:
//         madctl_par |= (MADCTL_MV | MADCTL_MX);
//         break;
//     default:
//         return -EINVAL;

//     }

//     write_reg(par, MIPI_DCS_SET_ADDRESS_MODE, madctl_par);
//     return 0;
// }

static int st7365p_hw_init(struct st7365p_par *par)
{
    st7365p_init_display(par);

    // st7365p_set_var(par);
    // st7365p_set_gamma(par, default_curves);
    
    /*
    st7365p_clear(par);
    msleep(1000);
    st7365p_blank(par, true);
    msleep(1000);
    st7365p_blank(par, false);
    msleep(1000);
    st7365p_clear(par);
    */

    return 0;
}

#define RED(a)      ((((a) & 0xf800) >> 11) << 3)
#define GREEN(a)    ((((a) & 0x07e0) >> 5) << 2)
#define BLUE(a)     (((a) & 0x001f) << 3)

#define to_rgb565(r,g,b) ((r) << 11 | (g) << 5 | (b))

static inline u16 rgb565_to_grayscale(u16 rgb565)
{
    int r,g,b;
    u16 gray;

    r = RED(rgb565);
    g = GREEN(rgb565);
    b = BLUE(rgb565);

    gray = ((r + g + b) / 3);

    /* map to rgb565 format */
    r = b = gray * 31 / 255;  // 0 ~ 31
    g = gray * 63 / 255;

    return cpu_to_be16(to_rgb565(r, g, b));
}

static inline u16 rgb565_to_grayscale_byweight(u16 rgb565)
{
    int r,g,b;
    u16 gray;

    /* get each channel and expand them to 8 bit */
    r = RED(rgb565);
    g = GREEN(rgb565);
    b = BLUE(rgb565);

    /* convert rgb888 to grayscale */
    gray = ((r * 77 + g * 151 + b * 28) >> 8); // 0 ~ 255

    /* map to rgb565 format */
    r = b = gray * 31 / 255;  // 0 ~ 31
    g = gray * 63 / 255;

    return cpu_to_be16(to_rgb565(r, g, b));
}

const uint8_t dither_4x4[4][4] =
{
    { 0,  8,  2, 10},
    {12,  4, 14,  6},
    { 3, 11,  1,  9},
    {15,  7, 13,  5},
};

const uint8_t dither_8x8[8][8] =
{
  {   0, 32,  8, 40,  2, 34, 10, 42 },
  {  48, 16, 56, 24, 50, 18, 58, 26 },
  {  12, 44,  4, 36, 14, 46,  6, 38 },
  {  60, 28, 52, 20, 62, 30, 54, 22 },
  {   3, 35, 11, 43,  1, 33,  9, 41 },
  {  51, 19, 59, 27, 49, 17, 57, 25 },
  {  15, 47,  7, 39, 13, 45,  5, 37 },
  {  63, 31, 55, 23, 61, 29, 53, 21 },
};

const uint8_t dither_16x16[16][16] =
{
{     0, 191,  48, 239,  12, 203,  60, 251,   3, 194,  51, 242,  15, 206,  63, 254  }, 
{   127,  64, 175, 112, 139,  76, 187, 124, 130,  67, 178, 115, 142,  79, 190, 127  },
{    32, 223,  16, 207,  44, 235,  28, 219,  35, 226,  19, 210,  47, 238,  31, 222  },
{   159,  96, 143,  80, 171, 108, 155,  92, 162,  99, 146,  83, 174, 111, 158,  95  },
{     8, 199,  56, 247,   4, 195,  52, 243,  11, 202,  59, 250,   7, 198,  55, 246  },
{   135,  72, 183, 120, 131,  68, 179, 116, 138,  75, 186, 123, 134,  71, 182, 119  },
{    40, 231,  24, 215,  36, 227,  20, 211,  43, 234,  27, 218,  39, 230,  23, 214  },
{   167, 104, 151,  88, 163, 100, 147,  84, 170, 107, 154,  91, 166, 103, 150,  87  },
{     2, 193,  50, 241,  14, 205,  62, 253,   1, 192,  49, 240,  13, 204,  61, 252  },
{   129,  66, 177, 114, 141,  78, 189, 126, 128,  65, 176, 113, 140,  77, 188, 125  },
{    34, 225,  18, 209,  46, 237,  30, 221,  33, 224,  17, 208,  45, 236,  29, 220  },
{   161,  98, 145,  82, 173, 110, 157,  94, 160,  97, 144,  81, 172, 109, 156,  93  },
{    10, 201,  58, 249,   6, 197,  54, 245,   9, 200,  57, 248,   5, 196,  53, 244  },
{   137,  74, 185, 122, 133,  70, 181, 118, 136,  73, 184, 121, 132,  69, 180, 117  },
{    42, 233,  26, 217,  38, 229,  22, 213,  41, 232,  25, 216,  37, 228,  21, 212  },
{   169, 106, 153,  90, 165, 102, 149,  86, 168, 105, 152,  89, 164, 101, 148,  85  }
};

const size_t matrix_size = 8;

static int write_vmem_3bit_dither(struct st7365p_par *par, size_t offset, size_t len)
{
    u16 *vmem16;
    u8 *txbuf = par->txbuf.buf;
    size_t remain;
    size_t to_copy;
    size_t tx_array_size;
    int i;
    int k;
    size_t cur_offset = offset;
    const size_t width = par->display->xres;
    const uint8_t color_red = 1 << 2;
    const uint8_t color_green = 1 << 1;
    const uint8_t color_blue = 1 << 3;

    dev_dbg(par->dev, "%s, offset = %zu, len = %zu\n", __func__, offset, len);

    remain = len / 2;
    vmem16 = (u16 *)(par->fbinfo->screen_buffer + offset);

    gpio_put(par->gpio.dc, 1);

    /* non-buffered spi write */
    if (!par->txbuf.buf)
        return st7365p_write_pixels(par, vmem16, len, 8);

    tx_array_size = par->txbuf.len / 3;

    while (remain) {
        to_copy = min(tx_array_size, remain);
        dev_dbg(par->fbinfo->device, "to_copy=%zu, remain=%zu\n",
                to_copy, remain - to_copy);

        for (i = 0, k = 0; i < to_copy; i += 2)
        {
            uint32_t col = cur_offset % width;
            uint32_t row = cur_offset / width;

            uint8_t thr = dither_16x16[row % matrix_size][col % matrix_size];

            txbuf[k] = 0;
            if ((((vmem16[i] & 0xF800) >> 11) & 0xFF) > (thr + 4) / 8)
            {
                txbuf[k] |= color_red << 3;
            }

            if ((((vmem16[i] & 0x07E0) >> 5) & 0xFF) > (thr + 2) / 4)
            {
                txbuf[k] |= color_green << 3;
            }

            if ((((vmem16[i] & 0x001F)) & 0xFF) > (thr + 4) / 8)
            {
                txbuf[k] |= color_blue << 3;
            }

            cur_offset++;

            col = cur_offset % width;
            row = cur_offset / width;
            thr = dither_16x16[row % matrix_size][col % matrix_size];

            if ((((vmem16[i + 1] & 0xF800) >> 11) & 0xFF) > (thr + 4) / 8)
            {
                txbuf[k] |= color_red;
            }

            if ((((vmem16[i + 1] & 0x07E0) >> 5) & 0xFF) > (thr + 2) / 4)
            {
                txbuf[k] |= color_green;
            }

            if ((((vmem16[i + 1] & 0x001F)) & 0xFF) > (thr + 4) / 8)
            {
                txbuf[k] |= color_blue;
            }

            cur_offset++;
            k++;
        } 

        /* send batch to device */
        st7365p_write_pixels(par, txbuf, to_copy / 2, 8);

        vmem16 = vmem16 + to_copy;
        remain -= to_copy;
    }
    return 0;
}

static int write_vmem_3bit(struct st7365p_par *par, size_t offset, size_t len)
{
    u16 *vmem16;
    u8 *txbuf = par->txbuf.buf;
    size_t remain;
    size_t to_copy;
    size_t tx_array_size;
    int i;
    int k;
    size_t cur_offset = offset;
    const size_t width = par->display->xres;
    const uint8_t color_red = 1 << 2;
    const uint8_t color_green = 1 << 1;
    const uint8_t color_blue = 1 << 3;

    dev_dbg(par->dev, "%s, offset = %zu, len = %zu\n", __func__, offset, len);

    remain = len / 2;
    vmem16 = (u16 *)(par->fbinfo->screen_buffer + offset);

    gpio_put(par->gpio.dc, 1);

    /* non-buffered spi write */
    if (!par->txbuf.buf)
        return st7365p_write_pixels(par, vmem16, len, 8);

    tx_array_size = par->txbuf.len / 3;

    while (remain) {
        to_copy = min(tx_array_size, remain);
        dev_dbg(par->fbinfo->device, "to_copy=%zu, remain=%zu\n",
                to_copy, remain - to_copy);

        for (i = 0, k = 0; i < to_copy; i += 2)
        {
            uint32_t col = cur_offset % width;
            uint32_t row = cur_offset / width;

            uint8_t thr = 8;

            txbuf[k] = 0;
            if ((((vmem16[i] & 0xF800) >> 11) & 0xFF) >= thr * 2)
            {
                txbuf[k] |= color_red << 3;
            }

            if ((((vmem16[i] & 0x07E0) >> 5) & 0xFF) >= thr * 4)
            {
                txbuf[k] |= color_green << 3;
            }

            if ((((vmem16[i] & 0x001F)) & 0xFF) >= thr * 2)
            {
                txbuf[k] |= color_blue << 3;
            }

            cur_offset++;

            col = cur_offset % width;
            row = cur_offset / width;

            if ((((vmem16[i + 1] & 0xF800) >> 11) & 0xFF) >= thr * 2)
            {
                txbuf[k] |= color_red;
            }

            if ((((vmem16[i + 1] & 0x07E0) >> 5) & 0xFF) >= thr * 4)
            {
                txbuf[k] |= color_green;
            }

            if ((((vmem16[i + 1] & 0x001F)) & 0xFF) >= thr * 2)
            {
                txbuf[k] |= color_blue;
            }

            cur_offset++;
            k++;
        } 

        /* send batch to device */
        st7365p_write_pixels(par, txbuf, to_copy / 2, 8);

        vmem16 = vmem16 + to_copy;
        remain -= to_copy;
    }
    return 0;
}

static int write_vmem(struct st7365p_par *par, size_t offset, size_t len)
{
    u16 *vmem16;
    u8 *txbuf = par->txbuf.buf;
    size_t remain;
    size_t to_copy;
    size_t tx_array_size;
    int i;
    int k;

    dev_dbg(par->dev, "%s, offset = %zu, len = %zu\n", __func__, offset, len);

    remain = len / 2;
    vmem16 = (u16 *)(par->fbinfo->screen_buffer + offset);

    gpio_put(par->gpio.dc, 1);

    /*
     * With 16 bit words the controller shifts each RGB565 pixel out MSB first
     * by itself, so the framebuffer goes to the SPI core untouched: no byte
     * swap loop, no bounce buffer, and the SPI core can DMA straight out of
     * the vmalloc'd framebuffer. That copy is what caps the frame rate once
     * the bus is clocked up, so it matters more than the clock itself.
     */
    if (par->bpw16)
        return st7365p_write_pixels(par, vmem16, len, 16);

    /* non-buffered spi write */
    if (!par->txbuf.buf)
        return st7365p_write_pixels(par, vmem16, len, 8);

    tx_array_size = par->txbuf.len / 3;

    while (remain) {
        to_copy = min(tx_array_size, remain);
        dev_dbg(par->fbinfo->device, "to_copy=%zu, remain=%zu\n",
                to_copy, remain - to_copy);

        for (i = 0, k = 0; i < to_copy; i++)
        {
            txbuf[k++] = ((vmem16[i] & 0xFF00) >> 8) & 0xFF;
            txbuf[k++] = (vmem16[i] & 0x00FF);
        }

        /* send batch to device */
        st7365p_write_pixels(par, txbuf, to_copy * 2, 8);

        vmem16 = vmem16 + to_copy;
        remain -= to_copy;
    }
    return 0;
}

static void update_display(struct st7365p_par *par, unsigned int start_line,
                           unsigned int end_line)
{
    size_t offset, len;

    dev_dbg(par->dev, "%s, start_line : %d, end_line : %d\n", __func__, start_line, end_line);

    // par->tftops->idle(par, false);
    /* write vmem to display then call refresh routine */
    /*
     * when this was called, driver should wait for busy pin comes low
     * until next frame refreshed
     */
    if (start_line > end_line) {
        dev_dbg(par->dev, "start line never should bigger than end line !!!!!\n");
        start_line = 0;
        end_line = par->fbinfo->var.yres - 1;
    }

    if (start_line > par->fbinfo->var.yres - 1 ||
        end_line > par->fbinfo->var.yres - 1) {
        dev_dbg(par->dev, "invaild start line or end line !!!!!\n");
        start_line = 0;
        end_line = par->fbinfo->var.yres - 1;
    }

    /* for each column, refresh dirty rows */

    gpio_put(par->gpio.cs, 0);
    par->tftops->set_addr_win(par, 0, start_line, par->fbinfo->var.xres - 1, end_line);

    offset = start_line * par->fbinfo->fix.line_length;
    len = (end_line - start_line + 1) * par->fbinfo->fix.line_length;

    if (p_3bit_mode)
    {
        if (p_dither)
            write_vmem_3bit_dither(par, offset, len);
	else
            write_vmem_3bit(par, offset, len);
    }
    else
    {
        write_vmem(par, offset, len);
    }

    gpio_put(par->gpio.cs, 1);

    // par->tftops->idle(par, true);
}

static void st7365p_mkdirty(struct fb_info *info, int y, int height)
{
    struct st7365p_par *par = info->par;
    struct fb_deferred_io *fbdefio = info->fbdefio;

    dev_dbg(info->dev, "%s, y : %d, height : %d\n", __func__, y, height);

    if (y == -1) {
        y = 0;
        height = info->var.yres;
    }

    /* mark dirty lines here, but update all for now */
    spin_lock(&par->dirty_lock);
    if (y < par->dirty_lines_start)
        par->dirty_lines_start = y;
    if (y + height - 1 > par->dirty_lines_end)
        par->dirty_lines_end = y + height - 1;
    spin_unlock(&par->dirty_lock);

    schedule_delayed_work(&info->deferred_work, fbdefio->delay);
}

static void st7365p_deferred_io(struct fb_info *info, struct list_head *pagelist)
{
    struct st7365p_par *par = info->par;
    unsigned int dirty_lines_start, dirty_lines_end;
    struct fb_deferred_io_pageref *pageref;
    unsigned int y_low = 0, y_high = 0;
    int count = 0;

    spin_lock(&par->dirty_lock);
    dirty_lines_start = par->dirty_lines_start;
    dirty_lines_end = par->dirty_lines_end;

    /* clean dirty markers */
    par->dirty_lines_start = par->fbinfo->var.yres - 1;
    par->dirty_lines_end = 0;
    spin_unlock(&par->dirty_lock);

    list_for_each_entry(pageref, pagelist, list) {
        count++;
        y_low = pageref->offset / info->fix.line_length;
        y_high = (pageref->offset + PAGE_SIZE - 1) / info->fix.line_length;
        dev_dbg(info->device,
                "page->index=%lu y_low=%d y_high=%d\n",
                pageref->page->index, y_low, y_high);

        if (y_high > info->var.yres - 1)
            y_high = info->var.yres - 1;
        if (y_low < dirty_lines_start)
            dirty_lines_start = y_low;
        if (y_high > dirty_lines_end)
            dirty_lines_end = y_high;
    }

    dev_dbg(info->device,
            "%s, count %d dirty_line  start : %d, end : %d\n",
            __func__, count, dirty_lines_start, dirty_lines_end);
    update_display(par, dirty_lines_start, dirty_lines_end);
}

static void st7365p_fb_fillrect(struct fb_info *info,
                                const struct fb_fillrect *rect)
{
    dev_dbg(info->dev,
            "%s: dx=%d, dy=%d, width=%d, height=%d\n",
            __func__, rect->dx, rect->dy, rect->width, rect->height);

    sys_fillrect(info, rect);
    st7365p_mkdirty(info, rect->dy, rect->height);
}

static void st7365p_fb_copyarea(struct fb_info *info,
                                const struct fb_copyarea *area)
{
    dev_dbg(info->dev,
            "%s: dx=%d, dy=%d, width=%d, height=%d\n",
            __func__,  area->dx, area->dy, area->width, area->height);

    sys_copyarea(info, area);
    st7365p_mkdirty(info, area->dy, area->height);
}

static void st7365p_fb_imageblit(struct fb_info *info,
                                 const struct fb_image *image)
{
    dev_dbg(info->dev,
            "%s: dx=%d, dy=%d, width=%d, height=%d\n",
            __func__,  image->dx, image->dy, image->width, image->height);
    sys_imageblit(info, image);

    st7365p_mkdirty(info, image->dy, image->height);
}

static ssize_t st7365p_fb_write(struct fb_info *info, const char __user *buf,
                                size_t count, loff_t *ppos)
{
    ssize_t res;
    dev_dbg(info->dev,
            "%s: count=%zd, ppos=%llu\n", __func__,  count, *ppos);

    res = fb_sys_write(info, buf, count, ppos);

    st7365p_mkdirty(info, -1, 0);
    return 0;
}

/* from pxafb.c */
static unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
    chan &= 0xffff;
    chan >>= 16 - bf->length;
    return chan << bf->offset;
}

static int st7365p_fb_setcolreg(unsigned int regno, unsigned int red,
                                unsigned int green, unsigned int blue,
                                unsigned int transp, struct fb_info *info)
{
    unsigned int val;
    int ret = 1;

    /* printk("%s(regno=%u, red=0x%X, green=0x%X, blue=0x%X, trans=0x%X)\n",
           __func__, regno, red, green, blue, transp); */

    if (regno >= 256)   /* no. of hw registers */
        return 1;
    /*
    * Program hardware... do anything you want with transp
    */

    switch (info->fix.visual) {
    case FB_VISUAL_TRUECOLOR:
        if (regno < 16) {
            val  = chan_to_field(red, &info->var.red);
            val |= chan_to_field(green, &info->var.green);
            val |= chan_to_field(blue, &info->var.blue);

            ((u32 *)(info->pseudo_palette))[regno] = val;
            ret = 0;
        }
        break;
    case FB_VISUAL_MONO01:
        ((u32 *)(info->pseudo_palette))[regno] =
                    (red << info->var.red.offset) |
                    (green << info->var.green.offset) |
                    (blue << info->var.blue.offset) |
                    (transp << info->var.transp.offset);
        ret = 0;
        break;
    }

    return ret;
}

static int st7365p_fb_blank(int blank, struct fb_info *info)
{
    struct st7365p_par *par = info->par;
    int ret = -EINVAL;

    switch (blank) {
    case FB_BLANK_POWERDOWN:
    case FB_BLANK_VSYNC_SUSPEND:
    case FB_BLANK_HSYNC_SUSPEND:
    case FB_BLANK_NORMAL:
        ret = st7365p_blank(par, true);
        break;
    case FB_BLANK_UNBLANK:
        ret = st7365p_blank(par, false);
        break;
    }
    return ret;
}

static const struct st7365p_display display = {
    .xres = 320,
    .yres = 320,
    .bpp = 16,
    .fps = 30,
};

static const struct st7365p_display display_3bit = {
    .xres = 320,
    .yres = 320,
    .bpp = 16,
    .fps = 60,
};

static int st7365p_probe(struct spi_device *spi)
{
    struct device *dev = &spi->dev;
    struct st7365p_par *par;
    struct fb_deferred_io *fbdefio;
    struct fb_event event;
    int width, height, bpp, rotate;
    struct fb_info *info;
    struct fb_ops *fbops;
    u8 *vmem = NULL;
    int vmem_size;
    int spi_tx_buf_size;
    int rc;

    /* memory resource alloc */
    if (p_3bit_mode)
    {
        rotate = display_3bit.rotate;
        bpp = display_3bit.bpp;
        switch (rotate) {
        case 90:
        case 270:
            width = display_3bit.yres;
            height = display_3bit.xres;
            break;
        default:
            width = display_3bit.xres;
            height = display_3bit.yres;
            break;
        }
    }
    else
    {
        rotate = display.rotate;
        bpp = display.bpp;
        switch (rotate) {
        case 90:
        case 270:
            width = display.yres;
            height = display.xres;
            break;
        default:
            width = display.xres;
            height = display.yres;
            break;
        }
    }

    vmem_size = (width * height * bpp) / BITS_PER_BYTE;
    vmem = vzalloc(vmem_size);
    if (!vmem)
        goto alloc_fail;

    fbops = devm_kzalloc(dev, sizeof(struct fb_ops), GFP_KERNEL);
    if (!fbops)
        goto alloc_fail;

    fbdefio = devm_kzalloc(dev, sizeof(struct fb_deferred_io), GFP_KERNEL);
    if (!fbdefio)
        goto alloc_fail;

    /* framebuffer info setup */
    info = framebuffer_alloc(sizeof(struct st7365p_par), dev);
    if (!info) {
        dev_err(dev, "failed to alloc framebuffer!\n");
        return -ENOMEM;
    }

    info->screen_buffer = vmem;
    info->fbops = fbops;
    info->fbdefio = fbdefio;

    fbops->owner        = dev->driver->owner;
    fbops->fb_read      = fb_sys_read;
    fbops->fb_write     = st7365p_fb_write;
    fbops->fb_fillrect  = st7365p_fb_fillrect;
    fbops->fb_copyarea  = st7365p_fb_copyarea;
    fbops->fb_imageblit = st7365p_fb_imageblit;
    fbops->fb_setcolreg = st7365p_fb_setcolreg;
    fbops->fb_blank     = st7365p_fb_blank;
    fbops->fb_mmap      = fb_deferred_io_mmap;

    snprintf(info->fix.id, sizeof(info->fix.id), "%s", dev->driver->name);
    info->fix.type            =       FB_TYPE_PACKED_PIXELS;
    info->fix.visual          =       FB_VISUAL_TRUECOLOR;
    info->fix.xpanstep        =       0;
    info->fix.ypanstep        =       0;
    info->fix.ywrapstep       =       0;
    info->fix.line_length     =       width * bpp / BITS_PER_BYTE;
    info->fix.accel           =       FB_ACCEL_NONE;
    info->fix.smem_len        =       vmem_size;

    info->var.rotate          =       rotate;
    info->var.xres            =       width;
    info->var.yres            =       height;
    info->var.xres_virtual    =       info->var.xres;
    info->var.yres_virtual    =       info->var.yres;

    info->var.bits_per_pixel  =       bpp;
    info->var.nonstd          =       1;
    info->var.grayscale       =       0;

    switch (info->var.bits_per_pixel) {
    case 1:
    case 2:
    case 4:
    case 8:
        info->var.red.offset = info->var.green.offset = info->var.blue.offset = 0;
        info->var.red.length = info->var.green.length = info->var.blue.length = 8;
        break;

    case 16:
        info->var.red.offset      =       11;
        info->var.red.length      =       5;
        info->var.green.offset    =       5;
        info->var.green.length    =       6;
        info->var.blue.offset     =       0;
        info->var.blue.length     =       5;
        info->var.transp.offset   =       0;
        info->var.transp.length   =       0;
        break;
    default:
        dev_err(dev, "color depth %d not supported\n",
                info->var.bits_per_pixel);
        break;
    }

    info->flags = FBINFO_FLAG_DEFAULT | FBINFO_VIRTFB;

    if (p_3bit_mode)
    {
        fbdefio->delay = HZ / display_3bit.fps;
    }
    else
    {
        fbdefio->delay = HZ / display.fps;
    }
    fbdefio->deferred_io = st7365p_deferred_io;
    fb_deferred_io_init(info);

    /* st7365p self setup */
    par = info->par;
    info->pseudo_palette = &par->pseudo_palette;

    par->fbinfo = info;
    par->dev = dev;
    par->spi = spi;
    par->spi_3_xfers = devm_kzalloc(dev, sizeof(struct spi_transfer) * 3, GFP_KERNEL);

    /*
     * Transfers below carry an explicit speed_hz, but keep spi->max_speed_hz
     * in step so anything that falls back to the device default agrees. The
     * SPI core still clamps to the controller ceiling, which spi-rockchip
     * pins at 50MHz stock and 100MHz with kernel-patches/0001 applied.
     */
    if (p_spi_speed_hz)
        spi->max_speed_hz = p_spi_speed_hz;

    par->px_speed_hz = spi->max_speed_hz;
    par->cmd_speed_hz = min_t(u32, p_cmd_speed_hz, par->px_speed_hz);
    /* at least one 16 bit word, so the chunk loop below always makes progress */
    par->max_xfer = max_t(size_t, spi_max_transfer_size(spi), 2);

    /* 3 bit mode packs bytes, so there is nothing for 16 bit words to do */
    par->bpw16 = p_bpw16 && !p_3bit_mode && spi_is_bpw_supported(spi, 16);

    dev_info(dev, "SPI: %u Hz pixels (%s), %u Hz registers, %zu byte transfers\n",
             par->px_speed_hz, par->bpw16 ? "16 bpw, zero copy" : "8 bpw, byte swapped",
             par->cmd_speed_hz, par->max_xfer);

    par->buf = devm_kzalloc(dev, 128, GFP_KERNEL);
    if (!par->buf) {
        dev_err(dev, "failed to alloc buf memory!\n");
        return -ENOMEM;
    }

    spi_tx_buf_size = width * height * 3;
    par->txbuf.buf = devm_kzalloc(dev, spi_tx_buf_size, GFP_KERNEL);
    if (!par->txbuf.buf) {
         dev_err(dev, "failed to alloc txbuf!\n");
         return -ENOMEM;
    }
    par->txbuf.len = spi_tx_buf_size;

    par->tftops = &default_st7365p_ops;
    if (p_3bit_mode)
    {
        par->display = &display_3bit;
    }
    else
    {
        par->display = &display;
    }

    dev_set_drvdata(dev, par);
    spi_set_drvdata(spi, par);

    spin_lock_init(&par->dirty_lock);
    init_completion(&par->complete);
    st7365p_of_config(par);

    st7365p_hw_init(par);

    update_display(par, 0, par->fbinfo->var.yres - 1);
    /* framebuffer register */
    rc = register_framebuffer(info);
    if (rc < 0) {
        dev_err(dev, "framebuffer register failed with %d!\n", rc);
        goto alloc_fail;
    }

    /* Notify backlight that display is starting in unblank state */
    event.info = info;
    int blank = FB_BLANK_UNBLANK;
    event.data = &blank;
    fb_notifier_call_chain(FB_EVENT_BLANK, &event);

    /* Buffer and video memory initialized */

    return 0;

alloc_fail:
    vfree(vmem);
    return 0;
}

static void st7365p_remove(struct spi_device *spi)
{
    struct st7365p_par *par = spi_get_drvdata(spi);

    fb_deferred_io_cleanup(par->fbinfo);
    unregister_framebuffer(par->fbinfo);
    framebuffer_release(par->fbinfo);
}

static int __maybe_unused st7365p_runtime_suspend(struct device *dev)
{
    // struct st7365p_par *par = dev_get_drvdata(dev);

    // par->tftops->sleep(par, true);

    return 0;
}

static int __maybe_unused st7365p_runtime_resume(struct device *dev)
{
    // struct st7365p_par *par = dev_get_drvdata(dev);

    // par->tftops->sleep(par, false);

    return 0;
}

static int __maybe_unused st7365p_runtime_idle(struct device *dev)
{
    // struct st7365p_par *par = dev_get_drvdata(dev);

    // par->tftops->idle(par, true);

    return 0;
}

static const struct of_device_id st7365p_dt_ids[] = {
    { .compatible = "sitronix,st7365p" },
    { .compatible = "picocalc,st7365p" },
    { /* KEEP THIS */ },
};
MODULE_DEVICE_TABLE(of, st7365p_dt_ids);

#if CONFIG_PM
static const struct dev_pm_ops st7365p_pm_ops = {
    SET_RUNTIME_PM_OPS(st7365p_runtime_suspend,
                       st7365p_runtime_resume,
                       st7365p_runtime_idle)
};
#else
static const struct dev_pm_ops st7365p_pm_ops = {
    SET_RUNTIME_PM_OPS(NULL, NULL, NULL)
};
#endif

static struct spi_driver st7365p_spi_drv = {
    .probe    = st7365p_probe,
    .remove   = st7365p_remove,
    .driver   = {
        .name           = DRV_NAME,
        .of_match_table = of_match_ptr(st7365p_dt_ids),
        // .pm             = &st7365p_pm_ops
    },
};

module_spi_driver(st7365p_spi_drv);

MODULE_AUTHOR("Calculinux");
MODULE_DESCRIPTION("Sitronix ST7365P based SPI LCD-TFT display framebuffer driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:st7365p");
