/*
 * si5351_vcxo.c - Example for using the Si5351B VCXO functions
 *                 with Si5351 library
 *
 * Copyright (C) 2016 Jason Milldrum <milldrum@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>

#include "si5351_h"

#define PLLB_FREQ 87600000000ULL

void setup(void) {
  si5351_init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);

  // Set VCXO osc to 876 MHz (146 MHz x 6), 40 ppm pull
  si5351_set_vcxo(PLLB_FREQ, 40);

  // Set CLK0 to be locked to VCXO
  si5351_set_ms_source(SI5351_CLK0, SI5351_PLLB);

  // Tune to 146 MHz center frequency
  si5351_set_freq_manual(14600000000ULL, PLLB_FREQ, SI5351_CLK0);

  si5351_update_status();
  // delay(500);
}

void loop(void) {
  // Read the Status Register and print it every 10 seconds
  si5351_update_status();
  printf("corr: %d", si5351_get_correction());
  printf("  SYS_INIT: %d", si5351_dev_status.SYS_INIT);
  printf("  LOL_A: %d", si5351_dev_status.LOL_A);
  printf("  LOL_B: %d", si5351_dev_status.LOL_B);
  printf("  LOS: %d", si5351_dev_status.LOS);
  printf("  REVID: %d\n", si5351_dev_status.REVID);

  // delay(10000);
}
