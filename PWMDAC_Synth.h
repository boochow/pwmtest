//
// PWM DAC Synthesizer ver.20160611
//  by Akiyoshi Kamide (Twitter: @akiyoshi_kamide)
//  http://kamide.b.osdn.me/pwmdac_synth_lib/
//  https://osdn.jp/users/kamide/pf/PWMDAC_Synth/
//
#pragma once

#include <Arduino.h>
#include <wiring_private.h>
#include <limits.h>

#define NumberOf(array) (sizeof(array)/sizeof((array)[0]))
#define BitSizeOf(type) (8 * sizeof(type))
#define cbi16(sfr, bit) (_SFR_WORD(sfr) &= ~_BV(bit))
#define sbi16(sfr, bit) (_SFR_WORD(sfr) |= _BV(bit))

// Function-to-array generator
#define FX2(f,x)    f(x), f(x + 1)
#define FX4(f,x)    FX2(f,x), FX2(f,x + 2)
#define FX8(f,x)    FX4(f,x), FX4(f,x + 4)
#define FX16(f,x)   FX8(f,x), FX8(f,x + 8)
#define FX32(f,x)   FX16(f,x),FX16(f,x + 16)
#define FX64(f,x)   FX32(f,x),FX32(f,x + 32)
#define FX128(f,x)  FX64(f,x),FX64(f,x + 64)
#define ARRAY128(f) {FX128(f,0)}
#define ARRAY256(f) {FX128(f,0),FX128(f,128)}

#ifndef PWMDAC_OUTPUT_PIN
#define PWMDAC_OUTPUT_PIN 3
#endif
#ifndef PWMDAC_NOTE_A_FREQUENCY
#define PWMDAC_NOTE_A_FREQUENCY 440 // [Hz]
#endif
#ifndef PWMDAC_POLYPHONY
#define PWMDAC_POLYPHONY 6
#endif


// Built-in wavetable generator
#define PWMDAC_SQUARE_WAVE(x)   (((x) < 128 ? 0 : 255) / PWMDAC_POLYPHONY)
#define PWMDAC_SAWTOOTH_WAVE(x) ((x) / PWMDAC_POLYPHONY)
#define PWMDAC_TRIANGLE_WAVE(x) (((x) < 128 ? (x) : 255 - (x)) * 2 / PWMDAC_POLYPHONY)

#define SINPI(x, t) sin(PI * (x) / (t))
#define PWMDAC_MAX_VOLUME_SINE_WAVE(x)  ((SINPI(x,128) + 1) * 128)
#define PWMDAC_SINE_WAVE(x)     (PWMDAC_MAX_VOLUME_SINE_WAVE(x) / PWMDAC_POLYPHONY)
#define PWMDAC_SHEPARD_TONE(x)  (( \
  SINPI(x,128) + SINPI(x,64) + SINPI(x,32) + SINPI(x,16) + \
  SINPI(x,8)   + SINPI(x,4)  + SINPI(x,2) + 7 ) * 18.22 / PWMDAC_POLYPHONY)

#define PWMDAC_CREATE_WAVETABLE(table, function) PROGMEM const byte table[] = ARRAY256(function)

enum AdsrStatus : byte {ADSR_OFF, ADSR_RELEASE, ADSR_SUSTAIN, ADSR_DECAY, ADSR_ATTACK};

typedef struct _Instrument {
  PROGMEM const byte *wavetable;
  PROGMEM const byte envelope[ADSR_ATTACK];
} Instrument;

class EnvelopeParam {
  public:
    EnvelopeParam() { }
    EnvelopeParam(PROGMEM const byte *envelope) {
      setParam(envelope);
    }
    EnvelopeParam(byte attack_time, byte decay_time, byte sustain_level, byte release_time) {
      *getParam(ADSR_ATTACK) = attack_time;     // 0..15
      *getParam(ADSR_DECAY) = decay_time;       // 0..15
      *getParam(ADSR_SUSTAIN) = sustain_level;  // 0..255
      *getParam(ADSR_RELEASE) = release_time;   // 0..15
    }
    byte *getParam(AdsrStatus adsr) {
      return param + (byte)adsr - 1;
    }
    void setParam(PROGMEM const byte *envelope) {
      memcpy_P(this->param, envelope, sizeof(this->param));
    }
  protected:
    byte param[ADSR_ATTACK];
};

class MidiChannel {
  protected:
    enum ByteSignificance {LSB, MSB}; // AVR is Little Endian
    static const byte RPN_Null = UCHAR_MAX;
    byte RPN[2]; // RPN (Registered Parameter Number) [LSB MSB]
    byte *data_entry_source; // RPN or NRPN(Non-Registered Parameter Number)
    byte pitch_bend_sensitivity; // +/- max semitones 0 .. 24(= 2 octaves)
    int pitch_bend; // Signed 14bit : -8192(lowest) .. 0(center) .. +8191(highest)
    double pitch_rate;
    void updatePitchRate() {
      //  98304 : 1 octave (== 12 semitones) high
      //   8192 : 1 semitone high
      //      0 : original
      //  -8192 : 1 semitone low
      // -98304 : 1 octave low
      long value8192_per_semitone = static_cast<long>(pitch_bend) * pitch_bend_sensitivity;
      // 2.0 : 1 octave high
      // 1.0 : original
      // 0.5 : 1 octave low
      pitch_rate = pow( 2, static_cast<double>(value8192_per_semitone) / 98304 );
    }
  public:
    byte modulation;  // 0 ... 127 (unsigned 7 bit - MSB only)
    PROGMEM const byte *wavetable;
    EnvelopeParam envelope;
    MidiChannel(PROGMEM const Instrument *instrument) { reset(instrument); }
    void reset(PROGMEM const Instrument *instrument) {
      resetAllControllers();
      RPN[LSB] = RPN[MSB] = RPN_Null;
      data_entry_source = NULL;
      programChange(instrument);
    }
    double getPitchRate() const { return pitch_rate; }
    int getPitchBend() const { return pitch_bend; }
    boolean pitchBendChange(int bend) {
      if( abs(bend - pitch_bend) < 16 ) return false;
      pitch_bend = bend;
      updatePitchRate();
      return true;
    }
    byte getPitchBendSensitivity() const { return pitch_bend_sensitivity; }
    boolean pitchBendSensitivityChange(byte value) {
      if ( pitch_bend_sensitivity == value ) return false;
      pitch_bend_sensitivity = value;
      updatePitchRate();
      return true;
    }
    void programChange(PROGMEM const Instrument *instrument) {
      this->wavetable = (PROGMEM const byte *)pgm_read_word(&(instrument->wavetable));
      this->envelope.setParam(instrument->envelope);
    }
    void resetAllControllers() {
      modulation = 0;
      pitch_bend = 0;
      pitch_rate = 1.0;
      pitch_bend_sensitivity = 2;
    }
    void controlChange(byte number, byte value) {
      switch(number) {
        case 1: modulation = value; break;
        case 6: // RPN/NRPN Data Entry
          if ( data_entry_source == NULL ) break;
          if ( data_entry_source[LSB] == 0 && data_entry_source[MSB] == 0 ) {
            pitchBendSensitivityChange(value);
          }
          data_entry_source = NULL;
          break;
        // case  96: break; // Data Increment
        // case  97: break; // Data Decrement
        case  98: // NRPN LSB
        case  99: data_entry_source = NULL; break; // NRPN MSB
        case 100: (data_entry_source = RPN)[LSB] = value; break;
        case 101: (data_entry_source = RPN)[MSB] = value; break;
        case 121: resetAllControllers(); break;
      }
    }
};

// [Phase-correct PWM dual-slope]
//    TCNTn =
//       00(BOTTOM) 01 02 03 ... FC FD FE
//       FF(TOP)    FE FD FC ... 03 02 01
//    -> 0xFF * 2 = 510 values (NOT 512)
//
// ISR()-call interval = (0xFF * 2) / F_CPU(16MHz) = 31.875us
// 
// [MIDI Tuning Standard]
// http://en.wikipedia.org/wiki/MIDI_Tuning_Standard
//    fn(d) = 440 Hz * 2^( (d - 69) / 12 )  MIDI note # d = 0..127
//
#define PHASE_SPEED_OF(note_number) ( \
  pow( 2, (double)(note_number - 69)/12 + (BitSizeOf(unsigned long) + 1) ) \
  * PWMDAC_NOTE_A_FREQUENCY * 0xFF / F_CPU )

class VoiceStatus {
  public:
    AdsrStatus getAdsrStatus() const { return adsr; }
    boolean isForChannel(MidiChannel *cp) { return this->channel == cp; }
    MidiChannel *getChannel() const { return channel; }
    boolean isSoundOn() { return adsr > ADSR_OFF; }
    boolean isSoundOn(MidiChannel *cp) { return isSoundOn() && isForChannel(cp); }
    boolean isNoteOn()  { return adsr > ADSR_RELEASE; }
    boolean isNoteOn(MidiChannel *cp) { return isNoteOn() && isForChannel(cp); }
    boolean isNoteOn(byte note) { return this->note == note && isNoteOn(); }
    boolean isNoteOn(byte note, MidiChannel *cp) { return isNoteOn(note) && isForChannel(cp); }
    byte getNote() const { return note; }
    unsigned int getVolume16() const { return volume.v16; }
    inline byte getVolume8() const { return volume.v8[sizeof(volume.v8) - 1]; }
    inline unsigned int nextPulseWidth() { return getVolume8() * nextWavePoint(); }
    unsigned int getTemperature() {
      unsigned int t = volume.v16 >> 1;
      return adsr == ADSR_ATTACK ? UINT_MAX - t : t;
    }
    VoiceStatus() { reset(); }
    void setChannel(MidiChannel *cp) { if( ! isForChannel(cp) ) reset(cp); }
    void attack(byte note) {
      this->wavetable = channel->wavetable;
      this->note = note;
      updatePitch();
      adsr = ADSR_ATTACK;
    }
    void release() { adsr = ADSR_RELEASE; }
    void reset(MidiChannel *cp = NULL) {
      adsr = ADSR_OFF; volume.v16 = 0; note = UCHAR_MAX;
      phase.v32 = dphase32.real = dphase32.moffset = dphase32.bended = 0L;
      channel = cp;
    }
    void update(int modulation_offset) {
      updateModulationStatus(modulation_offset);
      updateEnvelopeStatus();
    }
    void updatePitch() {
      static PROGMEM const unsigned long phase_speed_table[] = ARRAY128(PHASE_SPEED_OF);
      unsigned long phase_speed = pgm_read_dword(phase_speed_table + note);
      if( channel->getPitchBend() == 0 ) {
        dphase32.bended = phase_speed;
      } else {
        dphase32.bended = phase_speed * channel->getPitchRate();
      }
      dphase32.real = dphase32.bended + dphase32.moffset;
    }
  protected:
    PROGMEM const byte *wavetable;
    struct {
      unsigned long real;     // Real phase speed
      unsigned long bended;   // Pitch-bended phase speed
      long moffset;           // Modulation pitch offset
    } dphase32;
    union { unsigned long v32; byte v8[4]; } phase;
    inline byte nextWavePoint() {
      phase.v32 += dphase32.real;
      return pgm_read_byte( wavetable + phase.v8[sizeof(phase.v8) - 1] );
    }
    union { unsigned int v16; byte v8[2]; } volume;
    MidiChannel *channel;
    byte note; // 0..127
    AdsrStatus adsr;
    void updateModulationStatus(char modulation_offset) {
      if( channel->modulation <= 0x10 ) {
        if( dphase32.moffset == 0 ) return;
        dphase32.moffset = 0;
        dphase32.real = dphase32.bended;
        return;
      }
      dphase32.moffset = (dphase32.real >> 19) * channel->modulation * modulation_offset;
      dphase32.real = dphase32.bended + dphase32.moffset;
    }
    EnvelopeParam *getEnvelope() { return &(channel->envelope); }
    void updateEnvelopeStatus() {
      switch(adsr) {
        case ADSR_ATTACK: {
          unsigned long v32 = volume.v16;
          if( ( v32 += (UINT_MAX >> *(getEnvelope()->getParam(ADSR_ATTACK))) ) > UINT_MAX ) {
             volume.v16 = UINT_MAX; adsr = ADSR_DECAY; break;
          }
          volume.v16 = v32; break;
        }
        case ADSR_DECAY: {
          EnvelopeParam *e = getEnvelope();
          unsigned int dv = volume.v16 >> *(e->getParam(ADSR_DECAY));
          if( dv == 0 ) dv = 1;
          long v32 = volume.v16;
          unsigned int s = (unsigned int)(*(e->getParam(ADSR_SUSTAIN))) << 8;
          if( (v32 -= dv) <= s ) { volume.v16 = s; adsr = ADSR_SUSTAIN; break; }
          volume.v16 = v32; break;
        }
        case ADSR_RELEASE: {
          unsigned int dv = volume.v16 >> *(getEnvelope()->getParam(ADSR_RELEASE));
          if( dv == 0 ) dv = 1;
          long v32 = volume.v16;
          if( (v32 -= dv) < 0x100 ) { reset(); break; }
          volume.v16 = v32; break;
        }
      }
    }
};

#if PWMDAC_OUTPUT_PIN == 6 || PWMDAC_OUTPUT_PIN == 5
// In Arduino, TIMER0 has been reserved by wiring.c in Arduino core,
//   so defining PWMDAC_OUTPUT_PIN = 5 or 6 causes compile error
//   (multiple definition of `__vector_16')
#define PWMDAC_USE_TIMER0
#define PWMDAC_OVF_vect TIMER0_OVF_vect
#if PWMDAC_OUTPUT_PIN == 6
#define PWMDAC_OCR OCR0A
#else
#define PWMDAC_OCR OCR0B
#endif
#elif PWMDAC_OUTPUT_PIN == 9 || PWMDAC_OUTPUT_PIN == 10
#define PWMDAC_USE_TIMER1
#define PWMDAC_OVF_vect TIMER1_OVF_vect
#if PWMDAC_OUTPUT_PIN == 9
#define PWMDAC_OCR OCR1A  // OC1A maybe used as MOSI for SPI
#else
#define PWMDAC_OCR OCR1B  // OC1B maybe used as SS for SPI
#endif
#elif PWMDAC_OUTPUT_PIN == 11 || PWMDAC_OUTPUT_PIN == 3
#define PWMDAC_USE_TIMER2
#define PWMDAC_OVF_vect TIMER2_OVF_vect
#if PWMDAC_OUTPUT_PIN == 11
#define PWMDAC_OCR OCR2A
#else
#define PWMDAC_OCR OCR2B  // OC2B maybe used as INT1
#endif
#endif

class PWMDACSynth {
  public:
    static void setup() { // must be called from setup() once
      pinMode(PWMDAC_OUTPUT_PIN,OUTPUT);
#ifdef PWMDAC_USE_TIMER1
      // No prescaling
      sbi (TCCR1B, CS10);
      cbi (TCCR1B, CS11);
      cbi (TCCR1B, CS12);
      // Phase-correct PWM
      sbi (TCCR1A, WGM10);
      cbi (TCCR1A, WGM11);
      cbi (TCCR1B, WGM12);
      cbi (TCCR1B, WGM13);
#if PWMDAC_OUTPUT_PIN == 9
      // Compare Output Mode
      //    Phase Correct PWM Mode (TCNTn dual-slope operation)
      //      Clear OCnB on Compare Match when up-counting.
      //      Set OCnB on Compare Match when down-counting.
      cbi (TCCR1A, COM1A0);
      sbi (TCCR1A, COM1A1);
#else
      cbi (TCCR1A, COM1B0);
      sbi (TCCR1A, COM1B1);
#endif
      sbi(TIMSK1,TOIE1); // Enable interrupt
#endif
#ifdef PWMDAC_USE_TIMER2
      // No prescaling
      sbi (TCCR2B, CS20);
      cbi (TCCR2B, CS21);
      cbi (TCCR2B, CS22);
      // Phase-correct PWM
      sbi (TCCR2A, WGM20);
      cbi (TCCR2A, WGM21);
      cbi (TCCR2B, WGM22);
#if PWMDAC_OUTPUT_PIN == 11
      cbi (TCCR2A, COM2A0);
      sbi (TCCR2A, COM2A1);
#else
      cbi (TCCR2A, COM2B0);
      sbi (TCCR2A, COM2B1);
#endif
      sbi(TIMSK2,TOIE2); // Enable interrupt
#endif
    }
#define EACH_VOICE(p) for(VoiceStatus *(p)=voices; (p)<= voices + (PWMDAC_POLYPHONY - 1); (p)++)
#define EACH_CHANNEL(c) for(MidiChannel *(c)=channels; (c)<= channels + (NumberOf(channels) - 1); (c)++)
    inline static void updatePulseWidth() {
      unsigned int pw = 0;
      EACH_VOICE(v) pw += v->nextPulseWidth();
      PWMDAC_OCR = pw >> 8;
    }
    static void update() { // must be called from loop() repeatedly
      static byte modulation_phase = 0;
      int modulation_offset = pgm_read_byte(maxVolumeSineWavetable + (++modulation_phase)) - 0x7F;
      EACH_VOICE(v) v->update(modulation_offset);
    }
    static void noteOff(byte channel, byte pitch, byte velocity) {
      EACH_VOICE(v) if( v->isNoteOn(pitch,getChannel(channel)) ) v->release();
    }
    static void noteOn(byte channel, byte pitch, byte velocity) {
      getVoiceToAttack(getChannel(channel),pitch)->attack(pitch);
    }
    static void pitchBend(byte channel, int bend) {
      MidiChannel *cp = getChannel(channel);
      if ( ! cp->pitchBendChange(bend) ) return;
      EACH_VOICE(v) if( v->isSoundOn(cp) ) v->updatePitch();
    }
    static void controlChange(byte channel, byte number, byte value) {
      MidiChannel *cp = getChannel(channel);
      cp->controlChange(number, value);
      switch(number) {
        case 120: // All sound off
          EACH_VOICE(v) if( v->isSoundOn(cp) ) v->reset();
          break;
        case 123: // All notes off
          EACH_VOICE(v) if( v->isNoteOn(cp) ) v->release();
          break; 
      }
    }
    static void systemReset() {
      EACH_VOICE(v) v->reset();
      EACH_CHANNEL(c) c->reset(defaultInstrument);
    }
    static MidiChannel *getChannel(char channel) { return channels + (channel - 1); }
    static char getChannel(MidiChannel *cp) { return (cp - channels) + 1; }
    static byte musicalMod12(char);
    static byte musicalMod7(char);
    static byte log2(unsigned int);
    static int musicalConstrain12(int note, int min_note, int max_note) {
      if( max_note < note ) {
        note = max_note - musicalMod12(max_note - note);
      }
      else if( min_note > note ) {
        note = min_note + musicalMod12(note - min_note);
      }
      return note;
    }
  protected:
    static PROGMEM const Instrument * const defaultInstrument;
    static MidiChannel channels[16];
    static VoiceStatus voices[PWMDAC_POLYPHONY];
    static PROGMEM const byte maxVolumeSineWavetable[];
    static VoiceStatus *getVoiceToAttack(MidiChannel *channel, byte note) {
      EACH_VOICE(v) if( v->isNoteOn(note,channel) ) return v;
      struct {
        unsigned int temperature = UINT_MAX;
        VoiceStatus *voice = NULL;
      } coldest;
      unsigned int temperature;
      EACH_VOICE(v) {
        if( (temperature = v->getTemperature()) > coldest.temperature ) continue;
        coldest.temperature = temperature;
        coldest.voice = v;
      }
      coldest.voice->setChannel(channel);
      return coldest.voice;
    }
#undef EACH_VOICE
};

#define PWMDAC_CREATE_INSTANCE(instrument) \
  ISR(PWMDAC_OVF_vect) { PWMDACSynth::updatePulseWidth(); } \
  VoiceStatus PWMDACSynth::voices[PWMDAC_POLYPHONY]; \
  PWMDAC_CREATE_WAVETABLE(PWMDACSynth::maxVolumeSineWavetable, PWMDAC_MAX_VOLUME_SINE_WAVE); \
  PROGMEM const Instrument * const PWMDACSynth::defaultInstrument = instrument; \
  MidiChannel PWMDACSynth::channels[16] = MidiChannel(instrument);

