#include <Wire.h>
#include <EEPROM.h>
#include "ArduboyCore.h"


SPIClass& spi = SPI;


ArduboyCore::ArduboyCore()
{
// for M5Stack
  _init_width  = _width  = 240; // Set by specific xxxxx_Defines.h file or by users sketch
  _init_height = _height = 320; // Set by specific xxxxx_Defines.h file or by users sketch
  rotation  = 1;

  locked = true;
  inTransaction = false;
}

void ArduboyCore::spi_begin(void)
{
  if (locked) {locked = false; spi.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, TFT_SPI_MODE)); CS_L;}
}

void ArduboyCore::spi_end(void)
{
  if(!inTransaction) {
    if (!locked) {locked = true; CS_H; spi.endTransaction();}
  }
}

/***************************************************************************************
** Function name:           writecommand
** Description:             Send an 8 bit command to the TFT
***************************************************************************************/
void ArduboyCore::writecommand(uint8_t c)
{
  spi_begin(); // CS_L;

  DC_C;

  tft_Write_8(c);

  DC_D;

  spi_end();  // CS_H;
}


/***************************************************************************************
** Function name:           writedata
** Description:             Send a 8 bit data value to the TFT
***************************************************************************************/
void ArduboyCore::writedata(uint8_t d)
{
  spi_begin(); // CS_L;

  DC_D;        // Play safe, but should already be in data mode

  tft_Write_8(d);

  CS_L;        // Allow more hold time for low VDI rail

  spi_end();  // CS_H;
}


void ArduboyCore::writeBlock(uint16_t color, uint32_t repeat)
{
  uint32_t color32 = COL_32(color, color);

  if (repeat > 31) // Revert legacy toggle buffer change
  {
    WRITE_PERI_REG(SPI_MOSI_DLEN_REG(SPI_PORT), 511);
    while(repeat>31)
    {
      while (READ_PERI_REG(SPI_CMD_REG(SPI_PORT))&SPI_USR);
      WRITE_PERI_REG(SPI_W0_REG(SPI_PORT), color32);
      WRITE_PERI_REG(SPI_W1_REG(SPI_PORT), color32);
      WRITE_PERI_REG(SPI_W2_REG(SPI_PORT), color32);
      WRITE_PERI_REG(SPI_W3_REG(SPI_PORT), color32);
      WRITE_PERI_REG(SPI_W4_REG(SPI_PORT), color32);
      WRITE_PERI_REG(SPI_W5_REG(SPI_PORT), color32);
      WRITE_PERI_REG(SPI_W6_REG(SPI_PORT), color32);
      WRITE_PERI_REG(SPI_W7_REG(SPI_PORT), color32);
      WRITE_PERI_REG(SPI_W8_REG(SPI_PORT), color32);
      WRITE_PERI_REG(SPI_W9_REG(SPI_PORT), color32);
      WRITE_PERI_REG(SPI_W10_REG(SPI_PORT), color32);
      WRITE_PERI_REG(SPI_W11_REG(SPI_PORT), color32);
      WRITE_PERI_REG(SPI_W12_REG(SPI_PORT), color32);
      WRITE_PERI_REG(SPI_W13_REG(SPI_PORT), color32);
      WRITE_PERI_REG(SPI_W14_REG(SPI_PORT), color32);
      WRITE_PERI_REG(SPI_W15_REG(SPI_PORT), color32);
      SET_PERI_REG_MASK(SPI_CMD_REG(SPI_PORT), SPI_USR);
      repeat -= 32;
    }
    while (READ_PERI_REG(SPI_CMD_REG(SPI_PORT))&SPI_USR);
  }

  if (repeat)
  {
    // Revert toggle buffer change
    WRITE_PERI_REG(SPI_MOSI_DLEN_REG(SPI_PORT), (repeat << 4) - 1);
    for (uint32_t i=0; i <= (repeat>>1); i++) WRITE_PERI_REG((SPI_W0_REG(SPI_PORT) + (i << 2)), color32);
    SET_PERI_REG_MASK(SPI_CMD_REG(SPI_PORT), SPI_USR);
    while (READ_PERI_REG(SPI_CMD_REG(SPI_PORT))&SPI_USR);
  }
}



void ArduboyCore::boot()
{
// The control pins are deliberately set to the inactive state (CS high) as setup()
// might call and initialise other SPI peripherals which would could cause conflicts
// if CS is floating or undefined.
#ifdef TFT_CS
  digitalWrite(TFT_CS, HIGH); // Chip select high (inactive)
  pinMode(TFT_CS, OUTPUT);
#endif

#ifdef TFT_DC
  digitalWrite(TFT_DC, HIGH); // Data/Command high = data mode
  pinMode(TFT_DC, OUTPUT);
#endif

#ifdef TFT_RST
  if (TFT_RST >= 0) {
    digitalWrite(TFT_RST, HIGH); // Set high, do not share pin with another SPI device
    pinMode(TFT_RST, OUTPUT);
  }
#endif

  Serial.begin(115200);
  Serial.flush();
  Serial.print("Arduboy booting...\n");

  Wire.begin();
  pinMode(5, INPUT_PULLUP);     // add Faces
  btnsts = 0;
  
  EEPROM.begin(1024);

  spi.begin();
  bootLCD();
}


#define BLK_PWM_CHANNEL 7 // LEDC_CHANNEL_7

void ArduboyCore::bootLCD()
{
	if (TFT_RST >= 0) {
    	digitalWrite(TFT_RST, HIGH);
    	delay(5);
    	digitalWrite(TFT_RST, LOW);
    	delay(20);
    	digitalWrite(TFT_RST, HIGH);
  }
	delay(150);

	spi_begin();
	
  #include "ILI9341_Init.h"
	
#ifdef TFT_INVERSION_ON
  writecommand(TFT_INVON);
#endif

#ifdef TFT_INVERSION_OFF
  writecommand(TFT_INVOFF);
#endif
	
	spi_end();

  setRotation(1);
  fillRectM5(0,0,320,240,TFT_DARKGREY);
  
  // Init the back-light LED PWM
  ledcSetup(BLK_PWM_CHANNEL, 44100, 8);
  ledcAttachPin(TFT_BL, BLK_PWM_CHANNEL);
  ledcWrite(BLK_PWM_CHANNEL, 80);
}

void ArduboyCore::setRotation(uint8_t m)
{
  spi_begin();
  #include "ILI9341_Rotation.h"
  spi_end();
  addr_row = 0xFFFF;
  addr_col = 0xFFFF;
}


void ArduboyCore::setWindow(int32_t x0, int32_t y0, int32_t x1, int32_t y1)
{
  //spi_begin(); // Must be called before setWimdow

  addr_col = 0xFFFF;
  addr_row = 0xFFFF;

#ifdef CGRAM_OFFSET
  x0+=colstart;
  x1+=colstart;
  y0+=rowstart;
  y1+=rowstart;
#endif

  DC_C;
  tft_Write_8(TFT_CASET);
  DC_D;
  tft_Write_32(SPI_32(x0, x1));
  DC_C;

  // Row addr set
  tft_Write_8(TFT_PASET);
  DC_D;
  tft_Write_32(SPI_32(y0, y1));
  DC_C;

  // write to RAM
  tft_Write_8(TFT_RAMWR);
  DC_D;

  //spi_end();
}


void ArduboyCore::fillRectM5(int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color)
{
  // Clipping
  if ((x >= _width) || (y >= _height)) return;
  
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }

  if ((x + w) > _width)  w = _width  - x;
  if ((y + h) > _height) h = _height - y;

  if ((w < 1) || (h < 1)) return;

  spi_begin();
  inTransaction = true;

  setWindow(x, y, x + w - 1, y + h - 1);

  uint32_t n = (uint32_t)w * (uint32_t)h;

  writeBlock(color, n);

  inTransaction = false;
  spi_end();
}


void ArduboyCore::paintScreen(u_char *image)
{
  u_char *p;
  int x = 32;
  int y = 56;
  int w = 128;
  int h = 64;

  if ((x >= _width) || (y >= (int32_t)_height)) return;

  int32_t dx = 0;
  int32_t dy = 0;
  int32_t dw = w<<1;
  int32_t dh = h<<1;

  if (x < 0) { dw += x; dx = -x; x = 0; }
  if (y < 0) { dh += y; dy = -y; y = 0; }
  
  if ((x + w) > _width ) dw = _width  - x;
  if ((y + h) > _height) dh = _height - y;

  if (dw < 1 || dh < 1) return;

  spi_begin();
  inTransaction = true;

  setWindow(x, y, x + dw - 1, y + dh - 1); // Sets CS low and sent RAMWR

  // Line buffer makes plotting faster
  uint16_t  lineBuf[dw];
  dx = 0; dy = 0;
  
  while ( dy < h ){
    p = image + (dx + (dy/8)*w);
    uint16_t col = ( (*p & (1 << (dy%8))) ? TFT_WHITE : TFT_BLACK );
    lineBuf[dx*2] = col;
    lineBuf[dx*2 + 1] = col;
    if (++dx > w){
      spi.writeBytes((uint8_t*)lineBuf, dw<<1);
      spi.writeBytes((uint8_t*)lineBuf, dw<<1);
      dx = 0;
      dy++;
    }
  }
  
  inTransaction = false;
  spi_end();
}

/* Buttons */
#define FACES_KEYBOARD_I2C_ADDR 0x08


uint8_t ArduboyCore::getInput()
{
    return buttonsState(); 
}


uint8_t ArduboyCore::buttonsState()
{
   Wire.requestFrom(FACES_KEYBOARD_I2C_ADDR, 1);
  if (Wire.available()){
    uint8_t key = Wire.read();
    //Serial.printf("btnsts=%d\n",key);
    btnsts = ~key;
  }
  return btnsts;
}




void ArduboyCore::setRGBled(uint8_t red, uint8_t green, uint8_t blue)
{
}
