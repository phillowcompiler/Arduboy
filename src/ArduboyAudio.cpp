#include "Arduboy.h"
#include "ArduboyAudio.h"


volatile bool audio_enabled = false;

// Table of midi note frequencies * 2
//   They are times 2 for greater accuracy, yet still fits in a word.
//   Generated from Excel by =ROUND(2*440/32*(2^((x-9)/12)),0) for 0<x<128
// The lowest notes might not work, depending on the Arduino clock frequency
// Ref: http://www.phy.mtu.edu/~suits/notefreqs.html
const uint8_t PROGMEM _midi_byte_note_frequencies[48] = {
16,17,18,19,21,22,23,24,26,28,29,31,33,35,37,39,41,44,46,49,52,55,58,62,65,
69,73,78,82,87,92,98,104,110,117,123,131,139,147,156,165,175,185,196,208,220,
233,247
};
const unsigned int PROGMEM _midi_word_note_frequencies[80] = {
262,277,294,311,330,349,370,392,415,440,466,494,523,554,587,622,659,
698,740,784,831,880,932,988,1047,1109,1175,1245,1319,1397,1480,1568,1661,1760,
1865,1976,2093,2217,2349,2489,2637,2794,2960,3136,3322,3520,3729,3951,4186,
4435,4699,4978,5274,5588,5920,6272,6645,7040,7459,7902,8372,8870,9397,9956,
10548,11175,11840,12544,13290,14080,14917,15804,16744,17740,18795,19912,21096,
22351,23680,25088 };


/* AUDIO */

ArduboyAudio::ArduboyAudio()
{
  audio_enabled = false;
}

void ArduboyAudio::on()
{
  //power_timer1_enable();
  //power_timer3_enable();
  audio_enabled = true;
}

bool ArduboyAudio::enabled()
{
  return audio_enabled;
}

void ArduboyAudio::off()
{
  audio_enabled = false;
  //power_timer1_disable();
  //power_timer3_disable();
}

void ArduboyAudio::saveOnOff()
{
  EEPROM.write(EEPROM_AUDIO_ON_OFF, audio_enabled);
  EEPROM.commit();
  int enb = (audio_enabled ? 1:0);
  delay(10);
  int sts = (EEPROM.read(EEPROM_AUDIO_ON_OFF) ? 1:0);
  Serial.printf("saved sound status(%dbyte)=%d:%d\n", sizeof(bool), enb, sts);
}

void ArduboyAudio::begin()
{
  //tune_playing = false;
  int sts = (EEPROM.read(EEPROM_AUDIO_ON_OFF) ? 1:0);
  Serial.printf("read sound status=%d", sts);
  if (EEPROM.read(EEPROM_AUDIO_ON_OFF)){on();}
}





/* TUNES */
void ArduboyTunes::initChannel(byte pin)
{
  uint8_t ch = (pin == PIN_SPEAKER_1 ? SPEAKER1_CHANNEL:SPEAKER2_CHANNEL);
  //pinMode(pin, OUTPUT);
  ledcSetup(ch, 0, 13);
  ledcAttachPin(pin, ch);
  ledcWriteTone(ch, 0);
  digitalWrite(pin, 0); 
}


void ArduboyTunes::playNote(byte chan, byte note)
{
  unsigned int frequency2; /* frequency times 2 */

  // we can't plan on a channel that does not exist
  if (chan > SPEAKER2_CHANNEL)
    return;

  // we only have frequencies for 128 notes
  if (note > 127) {
    return;
  }

  if (note < 48) {
    frequency2 = pgm_read_byte(_midi_byte_note_frequencies + note);
  } else {
    frequency2 = pgm_read_word(_midi_word_note_frequencies + note - 48);
  }
  ledcWriteTone(chan, frequency2);
  if(chan == SPEAKER1_CHANNEL){
    wait_timer_frequency2 = frequency2;  // for "tune_delay" function
  }
}

void ArduboyTunes::stopNote(byte chan)
{
  ledcWriteTone(chan, 0);
  digitalWrite(PIN_SPEAKER_2, 0);
  if( chan == SPEAKER1_CHANNEL ){
    speakerTimer[SPEAKER1_CHANNEL].busy = false;
  }
}

void ArduboyTunes::playScore(const byte *score)
{
  if(!audio_enabled){return;}
  //Serial.print("playscore...\n");
  score_start = (byte *)score;
  score_cursor = score_start;
  step();  /* execute initial commands */
  tune_playing = true;  /* release the interrupt routine */
}

void ArduboyTunes::stopScore (void)
{
  stopNote(SPEAKER1_CHANNEL);
  stopNote(SPEAKER2_CHANNEL);
  tune_playing = false;
}

/* Do score commands until a "wait" is found, or the score is stopped.
This is called initially from tune_playcore, but then is called
from the interrupt routine when waits expire.
*/
/* if CMD < 0x80, then the other 7 bits and the next byte are a 15-bit big-endian number of msec to wait */
void ArduboyTunes::step()
{
  byte command, opcode, chan;
  unsigned duration;

  while (1) {
    command = pgm_read_byte(score_cursor++);
    opcode = command & 0xf0;
    chan = command & 0x0f;
    if (opcode == TUNE_OP_STOPNOTE) { /* stop note */
      stopNote(chan);
    }
    else if (opcode == TUNE_OP_PLAYNOTE) { /* play note */
      playNote(chan, pgm_read_byte(score_cursor++));
    }
    else if (opcode == TUNE_OP_RESTART) { /* restart score */
      score_cursor = score_start;
    }
    else if (opcode == TUNE_OP_STOP) { /* stop score */
      stopScore();
      break;
    }
    else if (opcode < 0x80) { /* wait count in msec. */
      duration = ((unsigned)command << 8) | (pgm_read_byte(score_cursor++));
      speakerTimer[SPEAKER1_CHANNEL].toggle_count = millis() + ((unsigned long) wait_timer_frequency2 * duration + 500) / 1000;
      speakerTimer[SPEAKER1_CHANNEL].busy = true;
      break;
    }
  }
}


void ArduboyTunes::tone(unsigned int frequency, unsigned long duration)
{
  if(!audio_enabled){return;}
  // Using 1ch(Speaker2) for Tone
  ledcWriteTone(SPEAKER2_CHANNEL, frequency);
  speakerTimer[SPEAKER2_CHANNEL].toggle_count = millis() + duration;
  speakerTimer[SPEAKER2_CHANNEL].busy = true;
}

void ArduboyTunes::soundOutput()
{
  if (tune_playing && speakerTimer[SPEAKER1_CHANNEL].busy &&
      millis() > speakerTimer[SPEAKER1_CHANNEL].toggle_count) {
    // end of a score wait, so execute more score commands
    ArduboyTunes::step();  // execute commands
  }
}

//
// for M5Stack
//
void ArduboyTunes::speakerUpdate() {
  
  soundOutput();

  if(speakerTimer[SPEAKER2_CHANNEL].busy && 
      millis() > speakerTimer[SPEAKER2_CHANNEL].toggle_count) {
      ledcWriteTone(SPEAKER2_CHANNEL, 0);
      digitalWrite(PIN_SPEAKER_2, 0);
      speakerTimer[SPEAKER2_CHANNEL].busy = false;
  }
}
