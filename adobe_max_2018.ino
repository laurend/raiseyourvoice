/*
LED VU meter for Arduino and Adafruit NeoPixel LEDs.

Hardware requirements:
 - Most Arduino or Arduino-compatible boards (ATmega 328P or better).
 - Adafruit Electret Microphone Amplifier (ID: 1063)
 - Adafruit Flora RGB Smart Pixels (ID: 1260)
   OR
 - Adafruit NeoPixel Digital LED strip (ID: 1138)
 - Optional: battery for portable use (else power through USB or adapter)
Software requirements:
 - Adafruit NeoPixel library

Connections:
 - 3.3V to mic amp +
 - GND to mic amp -
 - Analog pin to microphone output (configurable below)
 - Digital pin to LED data input (configurable below)
 See notes in setup() regarding 5V vs. 3.3V boards - there may be an
 extra connection to make and one line of code to enable or disable.

Written by Adafruit Industries.  Distributed under the BSD license.
This paragraph must be included in any redistribution.
*/

#include <Adafruit_DotStar.h>
#include <SPI.h>

#define N_ROWS  24  // Number of rows
#define N_COLS  20  // Number of cols
#define MIC_PIN   A3  // Microphone is attached to this analog pin
#define LED_DATA  4  // NeoPixel LED strand is connected to this pin
#define LED_CLOCK 5
#define DC_OFFSET 0  // DC offset in mic signal - if unusure, leave 0
#define NOISE     10  // Noise/hum/interference in mic signal
#define SAMPLES   60  // Length of buffer for dynamic level adjustment
#define TOP       (N_ROWS + 2) // Allow dot to go slightly off scale
#define PEAK_FALL 40  // Rate of peak falling dot

Adafruit_DotStar strip = Adafruit_DotStar(
  N_ROWS, LED_DATA, LED_CLOCK, DOTSTAR_GRB);

byte
  peak      = 0,      // Used for falling dot
  dotCount  = 0,      // Frame counter for delaying dot-falling speed
  volCount  = 0;      // Frame counter for storing past volume data
int
  vol[SAMPLES],       // Collection of prior volume samples
  lvl       = 10,      // Current "dampened" audio level
  minLvlAvg = 0,      // For dynamic adjustment of graph low & high
  maxLvlAvg = 512;
  
// LED array. Defines pixels a grid rather than chain.
int ledArray[24][20] =
{
  { 0, 47, -1, 99, 48, 49, 50, -1, 100, -1, 156, 157, 158, 159, -1, 171, 170, 169, 168, 167},
  { 1, 46, -1, 98, -1, -1, 51, -1, 101, -1, 155, -1, -1, 160, -1, 172, -1, -1, -1, -1},
  { 2, 45, -1, 97, -1, -1, 52, -1, 102, -1, 154, -1, -1, 161, -1, 173, -1, -1, -1, -1},
  { 3, 44, -1, 96, -1, -1, 53, -1, 103, -1, 153, -1, -1, 162, -1, 174, -1, -1, -1, -1},
  { 4, 43, -1, 95, -1, -1, 54, -1, 104, -1, 152, -1, -1, 163, -1, 175, -1, -1, -1, -1},
  { 5, 42, -1, 94, -1, -1, 55, -1, 105, -1, 151, -1, -1, 164, -1, 176, -1, -1, -1, -1},
  { 6, 41, -1, 93, -1, -1, 56, -1, 106, -1, 150, -1, -1, 165, -1, 177, -1, -1, -1, -1},
  { 7, 40, -1, 92, -1, -1, 57, -1, 107, -1, 149, -1, -1, 166, -1, 178, -1, -1, -1, -1},
  { 8, 39, -1, 91, -1, -1, 58, -1, 108, -1, 148, -1, -1, -1, -1, 179, -1, -1, -1, -1},
  { 9, 38, -1, 90, -1, -1, 59, -1, 109, -1, 147, -1, -1, -1, -1, 180, -1, -1, -1, -1},
  { 10, 37, -1, 89, -1, -1, 60, -1, 110, -1, 146, -1, -1, -1, -1, 181, -1, -1, -1, -1},
  { 11, 36, -1, 88, -1, -1, 61, -1, 111, -1, 145, -1, -1, -1, -1, 182, 199, 200, 201, -1},
  { 12, 35, -1, 87, -1, -1, 62, -1, 112, -1, 144, -1, -1, -1, -1, 183, -1, -1, -1, -1},
  { 13, 34, -1, 86, -1, -1, 63, -1, 113, -1, 143, -1, -1, -1, -1, 184, -1, -1, -1, -1},
  { 14, 33, -1, 85, -1, -1, 64, -1, 114, -1, 142, -1, -1, -1, -1, 185, -1, -1, -1, -1},
  { 15, 32, -1, 84, -1, -1, 65, -1, 115, -1, 141, -1, -1, -1, -1, 186, -1, -1, -1, -1},
  { 16, 31, -1, 83, -1, -1, 66, -1, 116, -1, 140, -1, -1, -1, -1, 187, -1, -1, -1, -1},
  { 17, 30, -1, 82, -1, -1, 67, -1, 117, -1, 139, -1, -1, 124, -1, 188, -1, -1, -1, -1},
  { 18, 29, -1, 81, -1, -1, 68, -1, 118, -1, 138, -1, -1, 125, -1, 189, -1, -1, -1, -1},
  { 19, 28, -1, 80, -1, -1, 69, -1, 119, -1, 137, -1, -1, 126, -1, 190, -1, -1, -1, -1},
  { 20, 27, -1, 79, -1, -1, 70, -1, 120, -1, 136, -1, -1, 127, -1, 191, -1, -1, -1, -1},
  { 21, 26, -1, 78, -1, -1, 71, -1, 121, -1, 135, -1, -1, 128, -1, 192, -1, -1, -1, -1},
  { 22, 25, -1, 77, -1, -1, 72, -1, 122, -1, 134, -1, -1, 129, -1, 193, -1, -1, -1, -1},
  { 23, 24, -1, 76, 75, 74, 73, -1, 123, -1, 133, 132, 131, 130, -1, 194, 195, 196, 197, 198}
};

void setup() {

  // This is only needed on 5V Arduinos (Uno, Leonardo, etc.).
  // Connect 3.3V to mic AND TO AREF ON ARDUINO and enable this
  // line.  Audio samples are 'cleaner' at 3.3V.
  // COMMENT OUT THIS LINE FOR 3.3V ARDUINOS (FLORA, ETC.):
  
  analogReference(EXTERNAL);

  memset(vol, 0, sizeof(vol));
  strip.begin();
  strip.show();
}

void loop() {
  uint8_t  i, j;
  uint16_t minLvl, maxLvl;
  int      n, height;
  
  n   = analogRead(MIC_PIN);            // Raw reading from mic 
  n   = abs(n - 512 - DC_OFFSET);       // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE); // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;           // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if(height < 0L)       height = 0;      // Clip output
  else if(height > TOP) height = TOP;
  //if(height > peak)     peak   = height; // Keep 'peak' dot at top

  // Color pixels based on rainbow gradient
  for(i=0; i<N_ROWS; i++) {
    for(j=0; j<N_COLS; j++) {
      if (ledArray[i][j] != -1) {
        if(i >= height) strip.setPixelColor(ledArray[i][j], 0, 0, 0);
        else strip.setPixelColor(ledArray[i][j],Wheel(map(i,0,N_ROWS-1,30,150)));    
      }
    }
    strip.show();
  }

  // Draw peak dot  
  //if(peak > 0 && peak <= N_ROWS-1) strip.setPixelColor(peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
  
    // Update strip

// Every few frames, make the peak pixel drop by 1:

    if(++dotCount >= PEAK_FALL) { //fall rate 
      
      if(peak > 0) peak--;
      dotCount = 0;
    }

  vol[volCount] = n;                      // Save sample for dynamic leveling
  if(++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for(i=1; i<SAMPLES; i++) {
    if(vol[i] < minLvl)      minLvl = vol[i];
    else if(vol[i] > maxLvl) maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)

}

// Input a value 0 to 255 to get a color value.
// The colors are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
