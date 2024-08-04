/*
 * si5351_sweeper.c - Si5351 Simple Sweep Generator
 *
 * Copyright (c) 2016 Thomas S. Knutsen <la3pna@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 *WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

/*
 * Connect Si5351 to I2C
 * Sweep out is on pin 5, ranging from 0-5V (3.3V).
 * Use a filter on sweep out voltage. 100K + 1uF should be a good start.
 * A op-amp can be used to improve the filtering of the DC voltage.
 */

#include <stdbool.h>
#include <stdio.h>

#include "si5351_h"

int correction = 0; // use the Si5351 correction sketch to find the frequency
                    // correction factor

int inData = 0;
long steps = 100;
unsigned long startFreq = 10000000;
unsigned long stopFreq = 100000000;
int analogpin = 5;
int delaytime = 50;

void info() {
  printf("Si5351 Sweeper\n");
  printf("A = Start frequency\n");
  printf("B = Stop frequency\n");
  printf("S = Stepsize\n");
  printf("M = Single sweep\n");
  printf("C = Continious sweep until Q\n");
  printf("T = Timestep in ms, currently %d\n", delaytime);
}

void setup(void) {
  si5351_init(SI5351_CRYSTAL_LOAD_8PF, 0, correction);
  si5351_drive_strength(SI5351_CLK0, SI5351_DRIVE_6MA);
  info();
}

void loop(void) {
  inData = 0;
  if (Serial.available() > 0) // see if incoming serial data:
  {
    inData = Serial.read(); // read oldest byte in serial buffer:
  }

  if (inData == 'M' || inData == 'm') {
    inData = 0;
    unsigned long freqstep = (stopFreq - startFreq) / steps;
    for (int i = 0; i < (steps + 1); i++) {
      unsigned long freq = startFreq + (freqstep * i);
      si5351_set_freq(freq * SI5351_FREQ_MULT, SI5351_CLK0);
      analogWrite(analogpin, map(i, 0, steps, 0, 255));
      vTaskDelay(pdMS_TO_TICKS(delaytime));
    }
    si5351_output_enable(SI5351_CLK0, 0);
  }

  if (inData == 'C' || inData == 'c') {
    bool running = true;
    inData = 0;
    while (running) {
      unsigned long freqstep = (stopFreq - startFreq) / steps;
      for (int i = 0; i < (steps + 1); i++) {
        unsigned long freq = startFreq + (freqstep * i);
        si5351_set_freq(freq * SI5351_FREQ_MULT, SI5351_CLK0);
        analogWrite(analogpin, map(i, 0, steps, 0, 255));
        vTaskDelay(pdMS_TO_TICKS(delaytime));
        if (Serial.available() > 0) // see if incoming serial data:
        {
          inData = Serial.read(); // read oldest byte in serial buffer:
          if (inData == 'Q' || inData == 'q') {
            running = false;
            inData = 0;
          }
        }
      }
    }

    si5351_output_enable(SI5351_CLK0, 0);
  }

  if (inData == 'S' || inData == 's') {
    steps = Serial.parseInt();
    printf("Steps: %d\n", steps);
    inData = 0;
  }

  if (inData == 'H' || inData == 'h') {
    info();
  }

  if (inData == 'T' || inData == 't') {
    delaytime = Serial.parseInt();
    printf("time pr step: %d\n", delaytime);
    inData = 0;
  }

  if (inData == 'L' || inData == 'l') {
    for (int i = 0; i < (steps + 1); i++) {
      // print out the value you read:
      printf(i * 10);
      printf(';');
      printf(steps);
      printf(';');
      printf(-i);
      vTaskDelay(pdMS_TO_TICKS(10)); // delay in between reads for stability
    }
    inData = 0;
  }

  if (inData == 'A' || inData == 'a') {
    startFreq = Serial.parseInt();
    printf("Start: %d\n", startFreq);
    inData = 0;
  }

  if (inData == 'B' || inData == 'b') {
    stopFreq = Serial.parseInt();
    printf("Stop: %d\n", stopFreq);
    inData = 0;
  }
}
