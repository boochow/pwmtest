//
// PWM DAC Synthesizer ver.20151003
//  by Akiyoshi Kamide (Twitter: @akiyoshi_kamide)
//  http://kamide.b.osdn.me/pwmdac_synth_lib/
//  https://osdn.jp/users/kamide/pf/PWMDAC_Synth/
//
#include "PWMDAC_Synth.h"

byte PWMDACSynth::musicalMod7(char x) {
  while( x & 0xF8 ) x = (x >> 3) + (x & 7);
  if(x==7) return 0;
  return x;
}
byte PWMDACSynth::musicalMod12(char x) {
  char n = x >> 2;
  while( n & 0xFC ) n = (n >> 2) + (n & 3);
  x &= 3;
  if(n==3||n==0) return x;
  return x + (n << 2);
}
byte PWMDACSynth::log2(unsigned int x) {
  byte index = 0;
  if (x & 0xFF00) { index += 8; x &= 0xFF00; }
  if (x & 0xF0F0) { index += 4; x &= 0xF0F0; }
  if (x & 0xCCCC) { index += 2; x &= 0xCCCC; }
  if (x & 0xAAAA) { index += 1; x &= 0xAAAA; }
  return index;
}

