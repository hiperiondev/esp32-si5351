/*
 * si5351_clk67_example.c - Simple example of setting CLK6 and CLK7
 *                            outputs using Si5351Arduino library
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
  // Start serial and initialize the Si5351
  si5351_init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);

  // Set CLK0 to output 14 MHz
  si5351_set_freq(1400000000ULL, SI5351_CLK0);

  // Set CLK6 to 23 MHz, library sets PLL to 874 MHz
  si5351_set_freq(2300000000ULL, SI5351_CLK6);

  // This won't work since it is not an even multiple of the PLL (874 MHz)
  si5351_set_freq(1234567800ULL, SI5351_CLK7);

  // Setting CLK7 to 87.4 MHz will work
  si5351_set_freq(8740000000ULL, SI5351_CLK7);

  si5351_update_status();
  // delay(500);
}

void loop(void) {
  // Read the Status Register and print it every 10 seconds
  si5351_update_status();
  printf("PLLA: %d"), (uint32_t)(si5351_plla_freq / 100));
  printf("   PLLB: %d", (uint32_t)(si5351_pllb_freq / 100));
  printf("  SYS_INIT: %d", si5351_dev_status.SYS_INIT);
  printf("  LOL_A: %d", si5351_dev_status.LOL_A);
  printf("  LOL_B: %d", si5351_dev_status.LOL_B);
  printf("  LOS: %d", si5351_dev_status.LOS);
  printf("  REVID: %d\n", si5351_dev_status.REVID);

  //delay(10000);
}
