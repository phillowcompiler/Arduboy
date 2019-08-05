#ifndef ArduboyCore_h
#define ArduboyCore_h

#include <Arduino.h>
#include <SPI.h>
#include "soc/spi_reg.h"
#include "In_eSPI_Setup.h"

#define SPI_PORT VSPI
#define TFT_SPI_MODE SPI_MODE0

// For the M5Stack module use these #define lines
#define TFT_MISO 19
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   14  // Chip select control pin
#define TFT_DC   27  // Data Command control pin
#define TFT_RST  33  // Reset pin (could connect to Arduino RESET pin)
#define TFT_BL   32  // LED back-light (required for M5Stack)

// New color definitions use for all my libraries
#define TFT_BLACK       0x0000      /*   0,   0,   0 */
#define TFT_NAVY        0x000F      /*   0,   0, 128 */
#define TFT_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define TFT_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define TFT_MAROON      0x7800      /* 128,   0,   0 */
#define TFT_PURPLE      0x780F      /* 128,   0, 128 */
#define TFT_OLIVE       0x7BE0      /* 128, 128,   0 */
#define TFT_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define TFT_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define TFT_BLUE        0x001F      /*   0,   0, 255 */
#define TFT_GREEN       0x07E0      /*   0, 255,   0 */
#define TFT_CYAN        0x07FF      /*   0, 255, 255 */
#define TFT_RED         0xF800      /* 255,   0,   0 */
#define TFT_MAGENTA     0xF81F      /* 255,   0, 255 */
#define TFT_YELLOW      0xFFE0      /* 255, 255,   0 */
#define TFT_WHITE       0xFFFF      /* 255, 255, 255 */
#define TFT_ORANGE      0xFDA0      /* 255, 180,   0 */
#define TFT_GREENYELLOW 0xB7E0      /* 180, 255,   0 */
#define TFT_PINK        0xFC9F

#define WIDTH 128
#define HEIGHT 64

#define INVERT 2 //< lit/unlit pixel
#define WHITE 1 //< lit pixel
#define BLACK 0 //< unlit pixel

// Macro
#define CS_L GPIO.out_w1tc = (1 << TFT_CS);GPIO.out_w1tc = (1 << TFT_CS)
#define CS_H GPIO.out_w1ts = (1 << TFT_CS)//;GPIO.out_w1ts = (1 << TFT_CS)

#define DC_C GPIO.out_w1tc = (1 << TFT_DC)//;GPIO.out_w1tc = (1 << TFT_DC)
#define DC_D GPIO.out_w1ts = (1 << TFT_DC)//;GPIO.out_w1ts = (1 << TFT_DC)

#define WR_L GPIO.out_w1tc = (1 << TFT_WR)
#define WR_H GPIO.out_w1ts = (1 << TFT_WR)

// Swap byte order for concatenated 16 bit colors
// AB CD -> DCBA for 32 bit register write
#define COL_32(H,L) ( ((H)<<8 | (H)>>8) | (((L)<<8 | (L)>>8)<<16 ) )
#define SPI_32(H,L) ( ((H)<<8 | (H)>>8) | (((L)<<8 | (L)>>8)<<16 ) )

// ESP32 using SPI with 16 bit color display

// ESP32 low level SPI writes for 8, 16 and 32 bit values
// to avoid the function call overhead
  
  // Write 8 bits
  #define tft_Write_8(C) \
  WRITE_PERI_REG(SPI_MOSI_DLEN_REG(SPI_PORT), 8-1); \
  WRITE_PERI_REG(SPI_W0_REG(SPI_PORT), C); \
  SET_PERI_REG_MASK(SPI_CMD_REG(SPI_PORT), SPI_USR); \
  while (READ_PERI_REG(SPI_CMD_REG(SPI_PORT))&SPI_USR);

  // Write 16 bits with corrected endianess for 16 bit colours
  #define tft_Write_16(C) \
  WRITE_PERI_REG(SPI_MOSI_DLEN_REG(SPI_PORT), 16-1); \
  WRITE_PERI_REG(SPI_W0_REG(SPI_PORT), C<<8 | C>>8); \
  SET_PERI_REG_MASK(SPI_CMD_REG(SPI_PORT), SPI_USR); \
  while (READ_PERI_REG(SPI_CMD_REG(SPI_PORT))&SPI_USR);

  // Write 16 bits
  #define tft_Write_16S(C) \
  WRITE_PERI_REG(SPI_MOSI_DLEN_REG(SPI_PORT), 16-1); \
  WRITE_PERI_REG(SPI_W0_REG(SPI_PORT), C); \
  SET_PERI_REG_MASK(SPI_CMD_REG(SPI_PORT), SPI_USR); \
  while (READ_PERI_REG(SPI_CMD_REG(SPI_PORT))&SPI_USR);

  // Write 32 bits
  #define tft_Write_32(C) \
  WRITE_PERI_REG(SPI_MOSI_DLEN_REG(SPI_PORT), 32-1); \
  WRITE_PERI_REG(SPI_W0_REG(SPI_PORT), C); \
  SET_PERI_REG_MASK(SPI_CMD_REG(SPI_PORT), SPI_USR); \
  while (READ_PERI_REG(SPI_CMD_REG(SPI_PORT))&SPI_USR);

class ArduboyCore
{
public:
    ArduboyCore();
    uint8_t getInput();
    uint8_t buttonsState();
    void paintScreen(u_char *image);
    void setRGBled(uint8_t red, uint8_t green, uint8_t blue);

protected:
    void boot();
    void bootLCD();

//For M5Stack
    void setRotation(uint8_t m);
    void setWindow(int32_t x0, int32_t y0, int32_t x1, int32_t y1);
    void fillRectM5(int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color);
    void spi_begin();
    void spi_end();
    void writecommand(uint8_t c);
    void writedata(uint8_t c);
    void writeBlock(uint16_t color, uint32_t repeat);

protected:
    int32_t  _init_width, _init_height; // Display w/h as input, used by setRotation()
    int32_t  _width, _height;           // Display w/h as modified by current rotation
    uint8_t  rotation;  // Display rotation (0-3)
    int32_t  addr_row, addr_col;
    bool     locked, inTransaction; // Transaction and mutex lock flags for ESP32
    uint8_t  btnsts;
};

#endif

