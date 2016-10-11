#include "Arduboy2.h"

#define PWMDAC_OUTPUT_PIN 13
#define PWMDAC_OUTPUT_PIN2 5

#include "PWMDAC_Synth.h"

Arduboy2 arduboy;

PWMDAC_CREATE_WAVETABLE(wave, PWMDAC_SQUARE_WAVE);

// {wavetableArray, {release, sustain, decay, attack}}
PROGMEM const Instrument instrument = {wave, {2, 128, 4, 0}};

PWMDAC_CREATE_INSTANCE(&instrument);

const uint8_t song1[] = {
88,12,88,12,89,12,91,12,91,12,89,12,88,12,86,12,
84,12,84,12,86,12,88,12,88,18,86,6,86,24,
88,12,88,12,89,12,91,12,91,12,89,12,88,12,86,12,
84,12,84,12,86,12,88,12,86,18,84,6,84,24,
86,12,86,12,88,12,84,12,86,12,88,6,89,6,88,12,84,12,
86,12,88,6,89,6,88,12,86,12,84,12,86,12,79,24,
88,12,88,12,89,12,91,12,91,12,89,12,88,12,86,12,
84,12,84,12,86,12,88,12,86,18,84,6,84,24,
-1  
};

const uint8_t song2[] = {
91,24,93,24,95,24,91,24,
88,24,89,24,91,24,89,24,
91,24,93,24,95,24,91,24,
88,24,86,24,91,24,88,24,
89,24,91,24,89,24,91,24,
89,24,91,24,88,24,83,12,91,12,
91,24,93,24,95,24,91,24,
88,24,89,24,91,24,88,24,
-1
};

struct player_t {
  uint8_t pos;
  int8_t note;
  int cnt;
  int8_t *song;
};

struct player_t p[4] = {
  { 0, 0, 1, song1},
  { 0, 0, 1, song2},
  { 0, 0, 1, song1},
  { 0, 0, 1, song2}
};

void play(struct player_t *p, uint8_t chan, uint8_t v, int8_t trans) {
  if(p->cnt-- == 1) {
    if (p->note != 0) {
      PWMDACSynth::noteOff(chan, p->note + trans, v);     
    }
    if((p->note = p->song[p->pos++]) < 0) {
      p->pos = 0;
      p->note = 0;
      p->cnt = 1;
    } else {
      PWMDACSynth::noteOn(chan, p->note + trans, v);
      p->cnt = p->song[p->pos++];
    }
  }  
}

void setup() {
  arduboy.begin();
  arduboy.setFrameRate(24);
  pinMode(PWMDAC_OUTPUT_PIN2,OUTPUT);
  PWMDACSynth::setup();
}

void loop() {
  if (!(arduboy.nextFrame()))
    return;

  play(&p[0], 1, 127, -12);
  play(&p[1], 2, 127, -36);
  play(&p[2], 3, 127, -24);
  play(&p[3], 4, 127, -48);
  
  PWMDACSynth::update();
  arduboy.display();
}
