/**
 * @file ArduboyTones.cpp
 * \brief An Arduino library for playing tones and tone sequences, 
 * intended for the Arduboy game system.
 */

/*****************************************************************************
  ArduboyTones

An Arduino library to play tones and tone sequences.

Specifically written for use by the Arduboy miniature game system
https://www.arduboy.com/
but could work with other Arduino AVR boards that have 16 bit timer 3
available, by changing the port and bit definintions for the pin(s)
if necessary.

Copyright (c) 2017 Scott Allen

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*****************************************************************************/

#include "ArduboyTones.h"

// pointer to a function that indicates if sound is enabled
static bool (*outputEnabled)();

static volatile long durationToggleCount = 0;
static volatile bool tonesPlaying = false;
static volatile bool toneSilent;
#ifdef TONES_VOLUME_CONTROL
static volatile bool toneHighVol;
static volatile bool forceHighVol = false;
static volatile bool forceNormVol = false;
#endif

static volatile uint16_t *tonesStart;
static volatile uint16_t *tonesIndex;
static volatile uint16_t toneSequence[MAX_TONES * 2 + 1];
static volatile bool inProgmem;


ArduboyTones::ArduboyTones(boolean (*outEn)())
{
  outputEnabled = outEn;

  toneSequence[MAX_TONES * 2] = TONES_END;
#ifdef ARDUBOY_SAMD
  initSAMD21timer(); // set up the timer
  REG_PORT_OUTCLR0 = TONE_PIN; // Set the output of TONE_PIN to LOW
  REG_PORT_DIRSET0 = TONE_PIN; // Set the direction of TONE_PIN to an output
#ifdef TONES_2_SPEAKER_PINS 
  REG_PORT_OUTCLR0 = TONE_PIN2; // Set the output of TONE_PIN to LOW
  REG_PORT_DIRSET0 = TONE_PIN2; // Set the direction of TONE_PIN to an output
#endif // def TONES_2_SPEAKER_PINS
#else
  bitClear(TONE_PIN_PORT, TONE_PIN); // set the pin low
  bitSet(TONE_PIN_DDR, TONE_PIN); // set the pin to output mode
#ifdef TONES_2_SPEAKER_PINS
  bitClear(TONE_PIN2_PORT, TONE_PIN2); // set pin 2 low
  bitSet(TONE_PIN2_DDR, TONE_PIN2); // set pin 2 to output mode
#endif 
#endif // def ARDUBOY_SAMD
}

void ArduboyTones::tone(uint16_t freq, uint16_t dur)
{
#ifdef ARDUBOY_SAMD
  /*ArduboyTones::*/pauseSAMD21timer(); // disable the output compare match interrupt
#else
  bitWrite(TIMSK3, OCIE3A, 0); // disable the output compare match interrupt
#endif  // ARDUBOY_SAMD
  inProgmem = false;
  tonesStart = tonesIndex = toneSequence; // set to start of sequence array
  toneSequence[0] = freq;
  toneSequence[1] = dur;
  toneSequence[2] = TONES_END; // set end marker
  nextTone(); // start playing
}

void ArduboyTones::tone(uint16_t freq1, uint16_t dur1,
                        uint16_t freq2, uint16_t dur2)
{
#ifdef ARDUBOY_SAMD
  pauseSAMD21timer(); // disable the output compare match interrupt
#else
  bitWrite(TIMSK3, OCIE3A, 0); // disable the output compare match interrupt
#endif // def ARDUBOY_SAMD
  inProgmem = false;
  tonesStart = tonesIndex = toneSequence; // set to start of sequence array
  toneSequence[0] = freq1;
  toneSequence[1] = dur1;
  toneSequence[2] = freq2;
  toneSequence[3] = dur2;
  toneSequence[4] = TONES_END; // set end marker
  nextTone(); // start playing
}

void ArduboyTones::tone(uint16_t freq1, uint16_t dur1,
                        uint16_t freq2, uint16_t dur2,
                        uint16_t freq3, uint16_t dur3)
{
#ifdef ARDUBOY_SAMD
  pauseSAMD21timer(); // disable the output compare match interrupt
#else
  bitWrite(TIMSK3, OCIE3A, 0); // disable the output compare match interrupt
#endif // def ARDUBOY_SAMD
  inProgmem = false;
  tonesStart = tonesIndex = toneSequence; // set to start of sequence array
  toneSequence[0] = freq1;
  toneSequence[1] = dur1;
  toneSequence[2] = freq2;
  toneSequence[3] = dur2;
  toneSequence[4] = freq3;
  toneSequence[5] = dur3;
  // end marker was set in the constructor and will never change
  nextTone(); // start playing
}

void ArduboyTones::tones(const uint16_t *tones)
{
#ifdef ARDUBOY_SAMD
  pauseSAMD21timer(); // disable the output compare match interrupt
#else  
  bitWrite(TIMSK3, OCIE3A, 0); // disable the output compare match interrupt
#endif // def ARDUBOY_SAMD
  inProgmem = true;
  tonesStart = tonesIndex = (uint16_t *)tones; // set to start of sequence array
  nextTone(); // start playing
}

void ArduboyTones::tonesInRAM(uint16_t *tones)
{
#ifdef ARDUBOY_SAMD
  pauseSAMD21timer(); // disable the output compare match interrupt
#else	
  bitWrite(TIMSK3, OCIE3A, 0); // disable the output compare match interrupt
#endif // def ARDUBOY_SAMD
  inProgmem = false;
  tonesStart = tonesIndex = tones; // set to start of sequence array
  nextTone(); // start playing
}

void ArduboyTones::noTone()
{
#ifdef ARDUBOY_SAMD
  pauseSAMD21timer(); // disable the output compare match interrupt
  //PORT->Group[PORTA].OUTCLR.reg = TONE_PIN; // TODO: may need to |=, can't remember
#else	
  bitWrite(TIMSK3, OCIE3A, 0); // disable the output compare match interrupt
  TCCR3B = 0; // stop the counter
  bitClear(TONE_PIN_PORT, TONE_PIN); // set the pin low
#ifdef TONES_VOLUME_CONTROL
  bitClear(TONE_PIN2_PORT, TONE_PIN2); // set pin 2 low
#endif // TONES_VOLUME_CONTROL
#endif // ARDUBOY_SAMD
  tonesPlaying = false;
}

void ArduboyTones::volumeMode(uint8_t mode)
{
#ifdef TONES_VOLUME_CONTROL
  forceNormVol = false; // assume volume is tone controlled
  forceHighVol = false;

  if (mode == VOLUME_ALWAYS_NORMAL) {
    forceNormVol = true;
  }
  else if (mode == VOLUME_ALWAYS_HIGH) {
    forceHighVol = true;
  }
#endif
}

bool ArduboyTones::playing()
{
  return tonesPlaying;
}

void ArduboyTones::nextTone()
{
  uint16_t freq;
  uint16_t dur;
  long toggleCount;
  uint32_t ocrValue;
#ifdef TONES_ADJUST_PRESCALER
  uint8_t tccrxbValue;
#endif

  freq = getNext(); // get tone frequency

  if (freq == TONES_END) { // if freq is actually an "end of sequence" marker
    noTone(); // stop playing
    return;
  }

  tonesPlaying = true;

  if (freq == TONES_REPEAT) { // if frequency is actually a "repeat" marker
    tonesIndex = tonesStart; // reset to start of sequence
    freq = getNext();
  }

#ifdef TONES_VOLUME_CONTROL
  if (((freq & TONE_HIGH_VOLUME) || forceHighVol) && !forceNormVol) {
    toneHighVol = true;
  }
  else {
    toneHighVol = false;
  }
#endif

  freq &= ~TONE_HIGH_VOLUME; // strip volume indicator from frequency
#ifdef ARDUBOY_SAMD
	ocrValue = F_CPU / 64 / freq / 2 - 1;
	  SerialUSB.println(ocrValue);
	      toneSilent = false;

#else  // ARDUBOY_SAMD
#ifdef TONES_ADJUST_PRESCALER
  if (freq >= MIN_NO_PRESCALE_FREQ) {
    tccrxbValue = _BV(WGM32) | _BV(CS30); // CTC mode, no prescaling
    ocrValue = F_CPU / freq / 2 - 1;
    toneSilent = false;
  }
  else {
    tccrxbValue = _BV(WGM32) | _BV(CS31); // CTC mode, prescaler /8
#endif // TONES_ADJUST_PRESCALER
    if (freq == 0) { // if tone is silent
      ocrValue = F_CPU / 8 / SILENT_FREQ / 2 - 1; // dummy tone for silence
      freq = SILENT_FREQ;
      toneSilent = true;
      bitClear(TONE_PIN_PORT, TONE_PIN); // set the pin low
    }
    else {
      ocrValue = F_CPU / 8 / freq / 2 - 1;
      toneSilent = false;
    }
#ifdef TONES_ADJUST_PRESCALER
  }
#endif // TONES_ADJUST_PRESCALER
#endif // ARDUBOY_SAMD

  if (!outputEnabled()) { // if sound has been muted
    toneSilent = true;
  }

#if !defined ARDUBOY_SAMD
#ifdef TONES_VOLUME_CONTROL
  if (toneHighVol && !toneSilent) {
    // set pin 2 to the compliment of pin 1
    if (bitRead(TONE_PIN_PORT, TONE_PIN)) {
      bitClear(TONE_PIN2_PORT, TONE_PIN2);
    }
    else {
      bitSet(TONE_PIN2_PORT, TONE_PIN2);
    }
  }
  else {
    bitClear(TONE_PIN2_PORT, TONE_PIN2); // set pin 2 low for normal volume
  }
#endif // TONES_VOLUME_CONTROL
#endif // ARDUBOY_SAMD

  dur = getNext(); // get tone duration
  if (dur != 0) {
    // A right shift is used to divide by 512 for efficency.
    // For durations in milliseconds it should actually be a divide by 500,
    // so durations will by shorter by 2.34% of what is specified.
    toggleCount = ((long)dur * freq) >> 9;
  }
  else {
    toggleCount = -1; // indicate infinite duration
  }
#ifdef ARDUBOY_SAMD
	// TODO: implement an equivalent to ocrValue 
  REG_TC3_COUNT16_CC0 = ocrValue;                   // Set the TC4 CC0 register as the TOP value in match frequency mode
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);       // Wait for synchronization
  REG_TC3_CTRLA |= TC_CTRLA_ENABLE; // enable TC4 TODO: TEST
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);       // Wait for synchronization
  //TODO: check if we need to bother adjusting prescaler
#else // def ARDUBOY_SAMD
  TCCR3A = 0;
#ifdef TONES_ADJUST_PRESCALER
  TCCR3B = tccrxbValue;
#else // TONES_ADJUST_PRESCALER
  TCCR3B = _BV(WGM32) | _BV(CS31); // CTC mode, prescaler /8
#endif // TONES_ADJUST_PRESCALER
  OCR3A = ocrValue;
  bitWrite(TIMSK3, OCIE3A, 1); // enable the output compare match interrupt
#endif // def ARDUBOY_SAMD
  durationToggleCount = toggleCount; // COMMIT DOCS: moved to end to simplify conditional #ifdef macro
}

uint16_t ArduboyTones::getNext()
{
  if (inProgmem) {
    return pgm_read_word(tonesIndex++);
  }
  return *tonesIndex++;
}

#ifdef ARDUBOY_SAMD
void TC3_Handler(){	
  if (TC3->COUNT16.INTFLAG.bit.OVF && TC3->COUNT16.INTENSET.bit.OVF){
    if (durationToggleCount != 0) {
      if (!toneSilent) {	
        REG_PORT_OUTTGL0 = TONE_PIN; // toggle the speaker pin
#ifdef TONES_VOLUME_CONTROL
        if (toneHighVol) {
		  // TODO: find out whether it needs to be out of phase with TONE_PIN (push-pull)	
          REG_PORT_OUTTGL0 = TONE_PIN2; // toggle the speaker pin
		}
#endif //def TONES_VOLUME_CONTROL		
	  }
	  else {
        REG_PORT_OUTCLR0 = TONE_PIN; // Set the output of TONE_PIN to LOW to prevent alternating noise on piezo activations
#ifdef TONES_VOLUME_CONTROL
        REG_PORT_OUTCLR0 = TONE_PIN2; // Set the output of TONE_PIN2 to LOW to prevent alternating noise on piezo activations
#endif //def TONES_VOLUME_CONTROL	   
      }
	  if (durationToggleCount > 0) {
        durationToggleCount--;
      }
	}
    else {
      ArduboyTones::nextTone();
    }	
    REG_TC3_INTFLAG = TC_INTFLAG_OVF;// Clear the OVF interrupt flag	
  }
}	
#else //ARDUBOY_SAMD
ISR(TIMER3_COMPA_vect)
{
  if (durationToggleCount != 0) {
    if (!toneSilent) {
      *(&TONE_PIN_PORT) ^= TONE_PIN_MASK; // toggle the pin
#ifdef TONES_VOLUME_CONTROL
      if (toneHighVol) {
        *(&TONE_PIN2_PORT) ^= TONE_PIN2_MASK; // toggle pin 2
      }
#endif //def TONES_VOLUME_CONTROL
    }
    if (durationToggleCount > 0) {
      durationToggleCount--;
    }
  }
  else {
    ArduboyTones::nextTone();
  }
}
#endif // def ARDUBOY_SAMD


void ArduboyTones::initSAMD21timer(){ // copyright Martin L
	  // Set up the generic clock (GCLK4) used to clock timers
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Feed GCLK4 to TC4 and TC5
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TC4 and TC5
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC2_TC3;     // Feed the GCLK4 to TC4 and TC5
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
 //TODO: change this to another variable, rather than a magic number
  REG_TC3_COUNT16_CC0 = /*0xB71A*/ 0x3ff;                   // Set the TC3 CC0 register as the TOP value in match frequency mode
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);       // Wait for synchronization

  NVIC_DisableIRQ(TC3_IRQn);
  NVIC_ClearPendingIRQ(TC3_IRQn);
  NVIC_SetPriority(TC3_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC3 to 0 (highest)
  NVIC_EnableIRQ(TC3_IRQn);         // Connect TC3 to Nested Vector Interrupt Controller (NVIC)

  REG_TC3_INTFLAG |= TC_INTFLAG_OVF;              // Clear the interrupt flags
  REG_TC3_INTENSET = TC_INTENSET_OVF;             // Enable TC3 interrupts
  // REG_TC4_INTENCLR = TC_INTENCLR_OVF;          // Disable TC3 interrupts
 
  REG_TC3_CTRLA |= TC_CTRLA_PRESCALER_DIV64 |     // Set prescaler to 64, 48MHz/64/2 = 375kHz base frequency
                   TC_CTRLA_WAVEGEN_MFRQ |        // Put the timer TC3 into match frequency (MFRQ) mode
                   TC_CTRLA_ENABLE;               // Enable TC3
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);       // Wait for synchronization
}
void ArduboyTones::pauseSAMD21timer()
{
  REG_TC3_CTRLA &= ~TC_CTRLA_ENABLE;               // Enable TC3
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
  REG_PORT_OUTCLR0 = TONE_PIN;                    // Set the output of TONE_PIN to LOW to prevent alternating noise on piezo activations
}