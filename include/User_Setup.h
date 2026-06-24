// User_Setup.h - Custom configuration for TFT_eSPI
// GC9A01 240x240 round display on ESP32-S3-DevKitM-1

#ifndef USER_SETUP_LOADED
#define USER_SETUP_LOADED

#define GC9A01_DRIVER

#define TFT_WIDTH  240
#define TFT_HEIGHT 240

// ESP32-S3 SPI pins (typical configuration)
#define TFT_MISO  -1       // Not connected
#define TFT_MOSI  11
#define TFT_SCLK  12
#define TFT_CS    10
#define TFT_DC    14
#define TFT_RST   17
#define TFT_BL    9        // Backlight pin

#define TOUCH_CS  -1

#define SPI_FREQUENCY        80000000
#define SPI_READ_FREQUENCY   20000000

// Optional: use SPI DMA for faster updates
#define USE_SPI_DMA 1

// Fonts
#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF

#define SMOOTH_FONT

#endif // USER_SETUP_LOADED