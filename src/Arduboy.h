#ifndef Arduboy_h
#define Arduboy_h

#include <Arduino.h>
#include <Print.h>
#include <limits.h>
#include <EEPROM.h>
#include "Config.h"
#include "ArduboyCore.h"
#include "ArduboyAudio.h"

// defines
#define M5STACK

// EEPROM settings
#define EEPROM_VERSION 0
#define EEPROM_BRIGHTNESS 1
#define EEPROM_AUDIO_ON_OFF 2

// values for button states
#define LEFT_BUTTON 4
#define RIGHT_BUTTON 8
#define UP_BUTTON 1
#define DOWN_BUTTON 2
#define A_BUTTON 0x10
#define B_BUTTON 0x20


class Arduboy : public Print, public ArduboyCore{
public:
    /* Constructor */    
    Arduboy();

    /* Audio */
    ArduboyTunes tunes;
    ArduboyAudio audio;

    /// Returns true if the button mask passed in is pressed.
    /**
    * if (pressed(LEFT_BUTTON + A_BUTTON))
    */
    boolean pressed(uint8_t buttons);

    /// Returns true if the button mask passed in not pressed.
    /**
    * if (notPressed(LEFT_BUTTON))
    */
    boolean notPressed(uint8_t buttons);

    /// Initializes the hardware (but with no boot logo)
    void begin();
    void beginNoLogo();
    void bootLogo();
    void bootUtils();

    void clear(){fillScreen(BLACK);}

    /// Copies the contents of the screen buffer to the screen.
    /**
    * X and Y positions on the display are from the top left corner, thus a Y of 64
    * is the bottom of the screen and an X of 128 is the right side of the screen.
    * "Color" or "value" means choosing whether a pixel is lit or not - if color is
    * 0, the pixel is off (black), if color is 1, the pixel is on (white).
    */
    void display();

    /// Sets a single pixel on the screen buffer to white or black.
    void drawPixel(int x, int y, uint8_t color);

    uint8_t getPixel(uint8_t x, uint8_t y);

    /// Draw a circle of a defined radius.
    /**
    * Draws a circle in white or black. X and Y are the center point of the circle.
    */
    void drawCircle(int16_t x0, int16_t y0, uint8_t r, uint8_t color);

    /// Draws one or more "corners" of a circle.
    void drawCircleHelper(int16_t x0, int16_t y0, uint8_t r, uint8_t cornername, uint8_t color);

    /// Draws a filled-in circle.
    void fillCircle(int16_t x0, int16_t y0, uint8_t r, uint8_t color);

   /// Draws one or both vertical halves of a filled-in circle.
    void fillCircleHelper(int16_t x0, int16_t y0, uint8_t r, uint8_t cornername, int16_t delta, uint8_t color);

    /// Draws a line between two points.
    /**
    * Uses Bresenham's algorithm.
    */
    void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t color);

    /// Draws a rectangle of a width and height.
    void drawRect(int16_t x, int16_t y, uint8_t w, uint8_t h, uint8_t color);

    /// Draws vertical line.
    void drawFastVLine(int16_t x, int16_t y, uint8_t h, uint8_t color);

    /// Draws a horizontal line.
    void drawFastHLine(int16_t x, int16_t y, uint8_t w, uint8_t color);

    /// Draws a filled-in rectangle.
    void fillRect(int16_t x, int16_t y, uint8_t w, uint8_t h, uint8_t color);

    /// Fills the screen buffer with white or black.
    void fillScreen(uint8_t color);
    
    void drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, uint8_t w, uint8_t h, 
 uint8_t color);

    /// Draws images that are bit-oriented horizontally.
  /**
   * This requires a lot of additional CPU power and will draw images slower
   * than drawBitmap, where the images are stored in a format that
   * allows them to be directly written to the screen. It is
   * recommended you use drawBitmap when possible.
   */
    void drawSlowXYBitmap(int16_t x, int16_t y, const uint8_t *bitmap, uint8_t w, uint8_t h, uint8_t color);

    /// Draws an ASCII character at a point.
    void drawChar(int16_t x, int16_t y, unsigned char c, uint8_t color, uint8_t bg, uint8_t size);

    /// Sets the location of the screen cursor.
    void setCursor(int16_t x, int16_t y);

    /// Set text size
    /**
    * As mentioned in drawChar(), individual ASCII characters are 6x8 pixels
    * (5x7 with spacing on two edges). The size is a pixel multiplier,
    * so a size of 2 means each character will be 12x16, etc.
    */
    void setTextSize(uint8_t s);

    /// Sets whether text will wrap at screen edges.
    void setTextWrap(boolean w);

    unsigned char* getBuffer();

    /// Writes a single ASCII character to the screen.
    virtual size_t write(uint8_t);

    

    /// Seeds the random number generator with entropy from the temperature, voltage reading, and microseconds since boot.
    /**
    * This method is still most effective when called semi-randomly such
    * as after a user hits a button to start a game or other semi-random
    * events
    */
    void initRandomSeed(){} /* dummy */

    /// Swap the references of two pointers.
    void swap(int16_t& a, int16_t& b);


    void setFrameRate(uint8_t rate);
    bool nextFrame();

    uint8_t frameRate;
    uint16_t frameCount;
    uint8_t eachFrameMillis;
    long lastFrameStart;
    long nextFrameStart;
    bool post_render;
    uint8_t lastFrameDurationMs;

    // Adafruit stuff

protected:
    unsigned char sBuffer[(HEIGHT*WIDTH)/8];

    int16_t cursor_x;
    int16_t cursor_y;
    uint8_t textsize;
    boolean wrap; // If set, 'wrap' text at right edge of display
};

#endif

