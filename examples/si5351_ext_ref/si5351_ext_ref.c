/*
 * si5351_ext_ref.ino - Simple example of using an external reference
 *                      clock with the Si5351Arduino library
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

void setup(void) {
  // Initialize the Si5351 to use a 25 MHz clock on the XO input
  si5351_init(SI5351_CRYSTAL_LOAD_0PF, 0, 0);

  // Set the CLKIN reference frequency to 10 MHz
  si5351_set_ref_freq(10000000UL, SI5351_PLL_INPUT_CLKIN);

  // Apply a correction factor to CLKIN
  si5351_set_correction(0, SI5351_PLL_INPUT_CLKIN);

  // Set PLLA and PLLB to use the signal on CLKIN instead of the XTAL
  si5351_set_pll_input(SI5351_PLLA, SI5351_PLL_INPUT_CLKIN);
  si5351_set_pll_input(SI5351_PLLB, SI5351_PLL_INPUT_CLKIN);

  // Set CLK0 to output 14 MHz
  si5351_set_freq(1400000000ULL, SI5351_CLK0);

  si5351_update_status();
  // delay(500);
}

void loop() {
  // Read the Status Register and print it every 10 seconds
  si5351_update_status();
  printf("  SYS_INIT: %d", si5351_dev_status.SYS_INIT);
  printf("  LOL_A: %d", si5351_dev_status.LOL_A);
  printf("  LOL_B: %d", si5351_dev_status.LOL_B);
  printf("  LOS: %d", si5351_dev_status.LOS);
  printf("  REVID: %d\n", si5351_dev_status.REVID);

  // delay(10000);
}
