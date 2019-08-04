#ifndef ArduboyAudio_h
#define ArduboyAudio_h

#include <Arduino.h>
#include <EEPROM.h>
#include "Config.h" 

// for M5Stack
#define PIN_SPEAKER_1 26     /* GPIO26 */
#define PIN_SPEAKER_2 2     /* GPIO2 */
#define SPEAKER1_CHANNEL 0
#define SPEAKER2_CHANNEL 1

#define TUNE_OP_PLAYNOTE  0x90  /* play a note: low nibble is generator #, note is next byte */
#define TUNE_OP_STOPNOTE  0x80  /* stop a note: low nibble is generator # */
#define TUNE_OP_RESTART   0xe0  /* restart the score from the beginning */
#define TUNE_OP_STOP      0xf0  /* stop playing */

class ArduboyAudio
{
public:
  ArduboyAudio();
  void begin();
  void on();
  void off();
  void saveOnOff();
  bool enabled();

//protected:
  //bool audio_enabled;
};

typedef struct{
  bool busy;
  int32_t toggle_count;
}SPEAKERTIMER;

class ArduboyTunes
{
public:
  // Playtune Functions

  /// Assign a timer to an output pin.
  void initChannel(byte pin);

  /// Start playing a polyphonic score.
  void playScore(const byte *score);

  /// Stop playing the score.
  void stopScore();

  /// Delay in milliseconds.
  void delay(unsigned msec){}

  /// Stop all timers.
  void closeChannels(){}

  bool playing(){}
  void tone(unsigned int frequency, unsigned long duration);

  // called via interrupt
  void step();
  void soundOutput();

  // for M5Stack
  void speakerUpdate();

private:
  void playNote (byte chan, byte note);
  void stopNote (byte chan);

private:
  byte *score_start;
  byte *score_cursor;
  SPEAKERTIMER speakerTimer[2];
  boolean tune_playing; // is the score still playing?
  unsigned int wait_timer_frequency2;
};

#endif

