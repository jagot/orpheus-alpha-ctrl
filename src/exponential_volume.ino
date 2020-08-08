// -*- c++ -*-
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <SPI.h>
#include <math.h>

// #define VERBOSE
#define NODEMCU

#ifdef NODEMCU

const int VOLMUTE = D3; // 0
const int VOLZCEN = D4; // 2
const int VOLSCLK = D5; // 14
const int VOLMISO = D6; // 12
const int VOLMOSI = D7; // 13
const int VOLCS = D8; // 15

const int ROT1 = D2; // 4
const int ROT2 = D1; // 5

//const long CLKRATE = 6250000;
const long CLKRATE = 1000000;
//const long CLKRATE = 6200;

// const auto spi_settings = SPISettings(CLKRATE, MSBFIRST, SPI_MODE0);

#else // e.g. Arduino Uno

const int ROT1 = 2;
const int ROT2 = 3;

#endif

const uint8_t VOL_MIN = 0;
const uint8_t VOL_MAX = 255;

Encoder volumeKnob(ROT1, ROT2);
uint8_t volume;
int prev_knob_pos;
const float KNOB_SENSITIVITY = 0.001f;

// Inspired by
// https://www.dr-lex.be/info-stuff/volumecontrols.html

const float GAIN_MIN = -80.0f,
  GAIN_MAX = 0.0f,
  GAIN_DIFF = GAIN_MAX - GAIN_MIN,
  GAIN_RATIO = pow(10.0f, GAIN_DIFF/20.0f);

const float A = 1.0f / GAIN_RATIO,
  B = log(GAIN_RATIO),
  LOG10E = 1.0f / log(10.0f),
  LOG10A = log10(A);

const float TAPER = 0.1f,
  LOG10TAPER = log10(TAPER);

float x;
const float X_INIT = 0.2f;

void compute_volume()
{
  if(x <= 0.0f){
    volume = 0;
  }
  else {
    float gain = LOG10A + B*x*LOG10E;
    if(x < TAPER){
      gain += log(x)*LOG10E - LOG10TAPER;
    }
    gain *= 20.0f;
    volume = static_cast<uint8_t>(constrain(255 - 2*(31.5f - gain),
                                            VOL_MIN, VOL_MAX));
  }
}

void set_volume()
{
  compute_volume();
  
  uint16_t vol16 = volume;
  vol16 <<= 8;
  vol16 |= volume;
  //SPI.beginTransaction(spi_settings);
  digitalWrite(VOLCS, LOW);
  SPI.write16(vol16);

  digitalWrite(VOLCS, HIGH);
}

void setup()
{
  delay(300); // Give PGA2311 some time to initialize
  pinMode(VOLCS, OUTPUT);
  SPI.begin();
  //  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE1));
  //  SPI.pins(VOLSCLK, VOLMISO, VOLMOSI, VOLCS); // Not supported in esp8266 2.3.0

  x = X_INIT;  
  set_volume();
  
  digitalWrite(VOLZCEN, LOW); // Enable zero-crossing detection
  digitalWrite(VOLMUTE, HIGH); // Unmute

  #ifdef VERBOSE
  Serial.begin(9600);
  Serial.println("Volume controller\nParameters:");
  Serial.print("Gain min: ");
  Serial.print(GAIN_MIN);
  Serial.print(" dB, max: ");
  Serial.print(GAIN_MAX);
  Serial.print(" dB => gain ratio = ");
  Serial.print(GAIN_RATIO);
  Serial.print(" => y = ");
  Serial.print(A);
  Serial.print("*exp(");
  Serial.print(B);
  Serial.println("*x)");
  #endif
}

void loop()
{
  int knob_pos = volumeKnob.read();
  float knob_diff = static_cast<float>(knob_pos - prev_knob_pos);
  
  if(knob_pos != prev_knob_pos) {
    x = constrain(x + KNOB_SENSITIVITY*knob_diff, 0.0f, 1.0f);
    set_volume();
    
    float volf = constrain(volume + knob_diff, VOL_MIN, VOL_MAX);
    volume = static_cast<uint8_t>(volf);
    prev_knob_pos = knob_pos;
    set_volume();
    delay(10);

    #ifdef VERBOSE
    Serial.print(knob_diff);
    Serial.print(" ");
    Serial.print(x);
    Serial.print(" ");
    Serial.print(volume);
    Serial.println();
    #endif
  }
}
