/*
 * Copyright 2024 Emiliano Gonzalez (egonzalez . hiperion @ gmail . com))
 * * Project Site: https://github.com/hiperiondev/esp32-si5351 *
 *
 * This is based on other projects:
 *    Si5351 library for Arduino: Jason Milldrum <milldrum@gmail.com>, Dana H. Myers <k6jq@comcast.net>
 *    Others (see individual files)
 *
 *    please contact their authors for more information.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <stdint.h>
#include <stdbool.h>

#include "si5351.h"

static uint64_t si5351_pll_calc(enum si5351_pll, uint64_t, struct si5351_reg_set*, int32_t, uint8_t);
static uint64_t si5351_multisynth_calc(uint64_t, uint64_t, struct si5351_reg_set*);
static uint64_t si5351_multisynth67_calc(uint64_t, uint64_t, struct si5351_reg_set*);
static void si5351_update_sys_status(struct si5351_status*);
static void si5351_update_int_status(struct si5351_int_status*);
static void si5351_ms_div(enum si5351_clock, uint8_t, uint8_t);
static uint8_t si5351_select_r_div(uint64_t*);
static uint8_t si5351_select_r_div_ms67(uint64_t*);

struct si5351_status dev_status = {
        .SYS_INIT = 0,
        .LOL_B = 0,
        .LOL_A = 0,
        .LOS = 0,
        .REVID = 0
};

struct si5351_int_status dev_int_status = {
        .SYS_INIT_STKY = 0,
        .LOL_B_STKY = 0,
        .LOL_A_STKY = 0,
        .LOS_STKY = 0
};

static enum si5351_pll pll_assignment[8];
static uint64_t clk_freq[8];
static uint64_t plla_freq;
static uint64_t pllb_freq;
static enum si5351_pll_input plla_ref_osc;
static enum si5351_pll_input pllb_ref_osc;
static uint32_t xtal_freq[2];
static int32_t ref_correction[2];
static uint8_t clkin_div;
static bool clk_first_set[8];

////////////////////////////////////////////////////////////////////////////////

static uint64_t si5351_pll_calc(enum si5351_pll pll, uint64_t freq, struct si5351_reg_set *reg, int32_t correction, uint8_t vcxo) {
    uint64_t ref_freq;
    if (pll == SI5351_PLLA) {
        ref_freq = xtal_freq[(uint8_t) plla_ref_osc] * SI5351_FREQ_MULT;
    } else {
        ref_freq = xtal_freq[(uint8_t) pllb_ref_osc] * SI5351_FREQ_MULT;
    }
    //ref_freq = 15974400ULL * SI5351_FREQ_MULT;
    uint32_t a, b, c, p1, p2, p3;
    uint64_t lltmp; //, denom;

    // Factor calibration value into nominal crystal frequency
    // Measured in parts-per-billion
    ref_freq = ref_freq + (int32_t) ((((((int64_t) correction) << 31) / 1000000000LL) * ref_freq) >> 31);

    // PLL bounds checking
    if (freq < SI5351_PLL_VCO_MIN * SI5351_FREQ_MULT) {
        freq = SI5351_PLL_VCO_MIN * SI5351_FREQ_MULT;
    }
    if (freq > SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT) {
        freq = SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT;
    }

    // Determine integer part of feedback equation
    a = freq / ref_freq;

    if (a < SI5351_PLL_A_MIN) {
        freq = ref_freq * SI5351_PLL_A_MIN;
    }
    if (a > SI5351_PLL_A_MAX) {
        freq = ref_freq * SI5351_PLL_A_MAX;
    }

    // Find best approximation for b/c = fVCO mod fIN
    // denom = 1000ULL * 1000ULL;
    // lltmp = freq % ref_freq;
    // lltmp *= denom;
    // do_div(lltmp, ref_freq);

    //b = (((uint64_t)(freq % ref_freq)) * RFRAC_DENOM) / ref_freq;
    if (vcxo) {
        b = (((uint64_t) (freq % ref_freq)) * 1000000ULL) / ref_freq;
        c = 1000000ULL;
    } else {
        b = (((uint64_t) (freq % ref_freq)) * RFRAC_DENOM) / ref_freq;
        c = b ? RFRAC_DENOM : 1;
    }

    // Calculate parameters
    p1 = 128 * a + ((128 * b) / c) - 512;
    p2 = 128 * b - c * ((128 * b) / c);
    p3 = c;

    // Recalculate frequency as fIN * (a + b/c)
    lltmp = ref_freq;
    lltmp *= b;
    do_div(lltmp, c);
    freq = lltmp;
    freq += ref_freq * a;

    reg->p1 = p1;
    reg->p2 = p2;
    reg->p3 = p3;

    if (vcxo) {
        return (uint64_t) (128 * a * 1000000ULL + b);
    } else {
        return freq;
    }
}

static uint64_t si5351_multisynth_calc(uint64_t freq, uint64_t pll_freq, struct si5351_reg_set *reg) {
    uint64_t lltmp;
    uint32_t a, b, c, p1, p2, p3;
    uint8_t divby4 = 0;
    uint8_t ret_val = 0;

    // Multisynth bounds checking
    if (freq > SI5351_MULTISYNTH_MAX_FREQ * SI5351_FREQ_MULT) {
        freq = SI5351_MULTISYNTH_MAX_FREQ * SI5351_FREQ_MULT;
    }
    if (freq < SI5351_MULTISYNTH_MIN_FREQ * SI5351_FREQ_MULT) {
        freq = SI5351_MULTISYNTH_MIN_FREQ * SI5351_FREQ_MULT;
    }

    if (freq >= SI5351_MULTISYNTH_DIVBY4_FREQ * SI5351_FREQ_MULT) {
        divby4 = 1;
    }

    if (pll_freq == 0) {
        // Find largest integer divider for max
        // VCO frequency and given target frequency
        if (divby4 == 0) {
            lltmp = SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT; // margin needed?
            do_div(lltmp, freq);
            if (lltmp == 5) {
                lltmp = 4;
            } else if (lltmp == 7) {
                lltmp = 6;
            }
            a = (uint32_t) lltmp;
        } else {
            a = 4;
        }

        b = 0;
        c = 1;
        pll_freq = a * freq;
    } else {
        // Preset PLL, so return the actual freq for these params instead of PLL freq
        ret_val = 1;

        // Determine integer part of feedback equation
        a = pll_freq / freq;

        if (a < SI5351_MULTISYNTH_A_MIN) {
            freq = pll_freq / SI5351_MULTISYNTH_A_MIN;
        }
        if (a > SI5351_MULTISYNTH_A_MAX) {
            freq = pll_freq / SI5351_MULTISYNTH_A_MAX;
        }

        b = (pll_freq % freq * RFRAC_DENOM) / freq;
        c = b ? RFRAC_DENOM : 1;
    }

    // Calculate parameters
    if (divby4 == 1) {
        p3 = 1;
        p2 = 0;
        p1 = 0;
    } else {
        p1 = 128 * a + ((128 * b) / c) - 512;
        p2 = 128 * b - c * ((128 * b) / c);
        p3 = c;
    }

    reg->p1 = p1;
    reg->p2 = p2;
    reg->p3 = p3;

    if (ret_val == 0) {
        return pll_freq;
    } else {
        return freq;
    }
}

static uint64_t si5351_multisynth67_calc(uint64_t freq, uint64_t pll_freq, struct si5351_reg_set *reg) {
    uint32_t a;
    uint64_t lltmp;

    // Multisynth bounds checking
    if (freq > SI5351_MULTISYNTH67_MAX_FREQ * SI5351_FREQ_MULT) {
        freq = SI5351_MULTISYNTH67_MAX_FREQ * SI5351_FREQ_MULT;
    }
    if (freq < SI5351_MULTISYNTH_MIN_FREQ * SI5351_FREQ_MULT) {
        freq = SI5351_MULTISYNTH_MIN_FREQ * SI5351_FREQ_MULT;
    }

    if (pll_freq == 0) {
        // Find largest integer divider for max
        // VCO frequency and given target frequency
        lltmp = (SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT) - 100000000UL; // margin needed?
        do_div(lltmp, freq);
        a = (uint32_t) lltmp;

        // Divisor has to be even
        if (a % 2 != 0) {
            a++;
        }

        // Divisor bounds check
        if (a < SI5351_MULTISYNTH_A_MIN) {
            a = SI5351_MULTISYNTH_A_MIN;
        }
        if (a > SI5351_MULTISYNTH67_A_MAX) {
            a = SI5351_MULTISYNTH67_A_MAX;
        }

        pll_freq = a * freq;

        // PLL bounds checking
        if (pll_freq > (SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT)) {
            a -= 2;
            pll_freq = a * freq;
        } else if (pll_freq < (SI5351_PLL_VCO_MIN * SI5351_FREQ_MULT)) {
            a += 2;
            pll_freq = a * freq;
        }

        reg->p1 = (uint8_t) a;
        reg->p2 = 0;
        reg->p3 = 0;
        return pll_freq;
    } else {
        // Multisynth frequency must be integer division of PLL
        if (pll_freq % freq) {
            // No good
            return 0;
        } else {
            a = pll_freq / freq;

            // Division ratio bounds check
            if (a < SI5351_MULTISYNTH_A_MIN || a > SI5351_MULTISYNTH67_A_MAX) {
                // No bueno
                return 0;
            } else {
                reg->p1 = (uint8_t) a;
                reg->p2 = 0;
                reg->p3 = 0;
                return 1;
            }
        }
    }
}

static void si5351_update_sys_status(struct si5351_status *status) {
    uint8_t reg_val = 0;

    reg_val = si5351_read(SI5351_DEVICE_STATUS);

    // Parse the register
    status->SYS_INIT = (reg_val >> 7) & 0x01;
    status->LOL_B = (reg_val >> 6) & 0x01;
    status->LOL_A = (reg_val >> 5) & 0x01;
    status->LOS = (reg_val >> 4) & 0x01;
    status->REVID = reg_val & 0x03;
}

static void si5351_update_int_status(struct si5351_int_status *int_status) {
    uint8_t reg_val = 0;

    reg_val = si5351_read(SI5351_INTERRUPT_STATUS);

    // Parse the register
    int_status->SYS_INIT_STKY = (reg_val >> 7) & 0x01;
    int_status->LOL_B_STKY = (reg_val >> 6) & 0x01;
    int_status->LOL_A_STKY = (reg_val >> 5) & 0x01;
    int_status->LOS_STKY = (reg_val >> 4) & 0x01;
}

static void si5351_ms_div(enum si5351_clock clk, uint8_t r_div, uint8_t div_by_4) {
    uint8_t reg_val = 0;
    uint8_t reg_addr = 0;

    switch (clk) {
        case SI5351_CLK0:
            reg_addr = SI5351_CLK0_PARAMETERS + 2;
            break;
        case SI5351_CLK1:
            reg_addr = SI5351_CLK1_PARAMETERS + 2;
            break;
        case SI5351_CLK2:
            reg_addr = SI5351_CLK2_PARAMETERS + 2;
            break;
        case SI5351_CLK3:
            reg_addr = SI5351_CLK3_PARAMETERS + 2;
            break;
        case SI5351_CLK4:
            reg_addr = SI5351_CLK4_PARAMETERS + 2;
            break;
        case SI5351_CLK5:
            reg_addr = SI5351_CLK5_PARAMETERS + 2;
            break;
        case SI5351_CLK6:
            reg_addr = SI5351_CLK6_7_OUTPUT_DIVIDER;
            break;
        case SI5351_CLK7:
            reg_addr = SI5351_CLK6_7_OUTPUT_DIVIDER;
            break;
    }

    reg_val = si5351_read(reg_addr);

    if (clk <= (uint8_t) SI5351_CLK5) {
        // Clear the relevant bits
        reg_val &= ~(0x7c);

        if (div_by_4 == 0) {
            reg_val &= ~(SI5351_OUTPUT_CLK_DIVBY4);
        } else {
            reg_val |= (SI5351_OUTPUT_CLK_DIVBY4);
        }

        reg_val |= (r_div << SI5351_OUTPUT_CLK_DIV_SHIFT);
    } else if (clk == SI5351_CLK6) {
        // Clear the relevant bits
        reg_val &= ~(0x07);

        reg_val |= r_div;
    } else if (clk == SI5351_CLK7) {
        // Clear the relevant bits
        reg_val &= ~(0x70);

        reg_val |= (r_div << SI5351_OUTPUT_CLK_DIV_SHIFT);
    }

    si5351_write(reg_addr, reg_val);
}

static uint8_t si5351_select_r_div(uint64_t *freq) {
    uint8_t r_div = SI5351_OUTPUT_CLK_DIV_1;

    // Choose the correct R divider
    if ((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 2)) {
        r_div = SI5351_OUTPUT_CLK_DIV_128;
        *freq *= 128ULL;
    } else if ((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 2) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 4)) {
        r_div = SI5351_OUTPUT_CLK_DIV_64;
        *freq *= 64ULL;
    } else if ((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 4) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 8)) {
        r_div = SI5351_OUTPUT_CLK_DIV_32;
        *freq *= 32ULL;
    } else if ((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 8) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 16)) {
        r_div = SI5351_OUTPUT_CLK_DIV_16;
        *freq *= 16ULL;
    } else if ((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 16) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 32)) {
        r_div = SI5351_OUTPUT_CLK_DIV_8;
        *freq *= 8ULL;
    } else if ((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 32) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 64)) {
        r_div = SI5351_OUTPUT_CLK_DIV_4;
        *freq *= 4ULL;
    } else if ((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 64) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 128)) {
        r_div = SI5351_OUTPUT_CLK_DIV_2;
        *freq *= 2ULL;
    }

    return r_div;
}

static uint8_t si5351_select_r_div_ms67(uint64_t *freq) {
    uint8_t r_div = SI5351_OUTPUT_CLK_DIV_1;

    // Choose the correct R divider
    if ((*freq >= SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT) && (*freq < SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 2)) {
        r_div = SI5351_OUTPUT_CLK_DIV_128;
        *freq *= 128ULL;
    } else if ((*freq >= SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 2) && (*freq < SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 4)) {
        r_div = SI5351_OUTPUT_CLK_DIV_64;
        *freq *= 64ULL;
    } else if ((*freq >= SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 4) && (*freq < SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 8)) {
        r_div = SI5351_OUTPUT_CLK_DIV_32;
        *freq *= 32ULL;
    } else if ((*freq >= SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 8) && (*freq < SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 16)) {
        r_div = SI5351_OUTPUT_CLK_DIV_16;
        *freq *= 16ULL;
    } else if ((*freq >= SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 16) && (*freq < SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 32)) {
        r_div = SI5351_OUTPUT_CLK_DIV_8;
        *freq *= 8ULL;
    } else if ((*freq >= SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 32) && (*freq < SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 64)) {
        r_div = SI5351_OUTPUT_CLK_DIV_4;
        *freq *= 4ULL;
    } else if ((*freq >= SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 64) && (*freq < SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT * 128)) {
        r_div = SI5351_OUTPUT_CLK_DIV_2;
        *freq *= 2ULL;
    }

    return r_div;
}

////////////////////////////////////////////////////////////////////////////////

bool si5351_init(uint8_t xtal_load_c, uint32_t xo_freq, int32_t corr) {
    uint8_t reg_val = 0;

    xtal_freq[0] = SI5351_XTAL_FREQ;

    // Start by using XO ref osc as default for each PLL
    plla_ref_osc = SI5351_PLL_INPUT_XO;
    pllb_ref_osc = SI5351_PLL_INPUT_XO;
    clkin_div = SI5351_CLKIN_DIV_1;

    if (reg_val == 0) {
        // Wait for SYS_INIT flag to be clear, indicating that device is ready
        uint8_t status_reg = 0;
        do {
            status_reg = si5351_read(SI5351_DEVICE_STATUS);
        } while (status_reg >> 7 == 1);

        // Set crystal load capacitance
        si5351_write(SI5351_CRYSTAL_LOAD, (xtal_load_c & SI5351_CRYSTAL_LOAD_MASK) | 0b00010010);

        // Set up the XO reference frequency
        if (xo_freq != 0) {
            si5351_set_ref_freq(xo_freq, SI5351_PLL_INPUT_XO);
        } else {
            si5351_set_ref_freq(SI5351_XTAL_FREQ, SI5351_PLL_INPUT_XO);
        }

        // Set the frequency calibration for the XO
        si5351_set_correction(corr, SI5351_PLL_INPUT_XO);

        si5351_reset();

        return true;
    }

    return false;
}

void si5351_reset(void) {
    // Initialize the CLK outputs according to flowchart in datasheet
    // First, turn them off
    si5351_write(16, 0x80);
    si5351_write(17, 0x80);
    si5351_write(18, 0x80);
    si5351_write(19, 0x80);
    si5351_write(20, 0x80);
    si5351_write(21, 0x80);
    si5351_write(22, 0x80);
    si5351_write(23, 0x80);

    // Turn the clocks back on...
    si5351_write(16, 0x0c);
    si5351_write(17, 0x0c);
    si5351_write(18, 0x0c);
    si5351_write(19, 0x0c);
    si5351_write(20, 0x0c);
    si5351_write(21, 0x0c);
    si5351_write(22, 0x0c);
    si5351_write(23, 0x0c);

    // Set PLLA and PLLB to 800 MHz for automatic tuning
    si5351_set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
    si5351_set_pll(SI5351_PLL_FIXED, SI5351_PLLB);

    // Make PLL to CLK assignments for automatic tuning
    pll_assignment[0] = SI5351_PLLA;
    pll_assignment[1] = SI5351_PLLA;
    pll_assignment[2] = SI5351_PLLA;
    pll_assignment[3] = SI5351_PLLA;
    pll_assignment[4] = SI5351_PLLA;
    pll_assignment[5] = SI5351_PLLA;
    pll_assignment[6] = SI5351_PLLB;
    pll_assignment[7] = SI5351_PLLB;

    si5351_set_ms_source(SI5351_CLK0, SI5351_PLLA);
    si5351_set_ms_source(SI5351_CLK1, SI5351_PLLA);
    si5351_set_ms_source(SI5351_CLK2, SI5351_PLLA);
    si5351_set_ms_source(SI5351_CLK3, SI5351_PLLA);
    si5351_set_ms_source(SI5351_CLK4, SI5351_PLLA);
    si5351_set_ms_source(SI5351_CLK5, SI5351_PLLA);
    si5351_set_ms_source(SI5351_CLK6, SI5351_PLLB);
    si5351_set_ms_source(SI5351_CLK7, SI5351_PLLB);

    // Reset the VCXO param
    si5351_write(SI5351_VXCO_PARAMETERS_LOW, 0);
    si5351_write(SI5351_VXCO_PARAMETERS_MID, 0);
    si5351_write(SI5351_VXCO_PARAMETERS_HIGH, 0);

    // Then reset the PLLs
    si5351_pll_reset(SI5351_PLLA);
    si5351_pll_reset(SI5351_PLLB);

    // Set initial frequencies
    uint8_t i;
    for (i = 0; i < 8; i++) {
        clk_freq[i] = 0;
        si5351_output_enable((enum si5351_clock) i, 0);
        clk_first_set[i] = false;
    }
}

bool si5351_set_freq(uint64_t freq, enum si5351_clock clk) {
    struct si5351_reg_set ms_reg;
    uint64_t pll_freq;
    uint8_t int_mode = 0;
    uint8_t div_by_4 = 0;
    uint8_t r_div = 0;

    // Check which Multisynth is being set
    if ((uint8_t) clk <= (uint8_t) SI5351_CLK5) {
        // MS0 through MS5 logic
        // ---------------------

        // Lower bounds check
        if (freq > 0 && freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT) {
            freq = SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT;
        }

        // Upper bounds check
        if (freq > SI5351_MULTISYNTH_MAX_FREQ * SI5351_FREQ_MULT) {
            freq = SI5351_MULTISYNTH_MAX_FREQ * SI5351_FREQ_MULT;
        }

        // If requested freq >100 MHz and no other outputs are already >100 MHz,
        // we need to recalculate PLLA and then recalculate all other CLK outputs
        // on same PLL
        if (freq > (SI5351_MULTISYNTH_SHARE_MAX * SI5351_FREQ_MULT)) {
            // Check other clocks on same PLL
            uint8_t i;
            for (i = 0; i < 6; i++) {
                if (clk_freq[i] > (SI5351_MULTISYNTH_SHARE_MAX * SI5351_FREQ_MULT)) {
                    if (i != (uint8_t) clk && pll_assignment[i] == pll_assignment[clk]) {
                        return 1; // won't set if any other clks already >100 MHz
                    }
                }
            }

            // Enable the output on first set_freq only
            if (clk_first_set[(uint8_t) clk] == false) {
                si5351_output_enable(clk, 1);
                clk_first_set[(uint8_t) clk] = true;
            }

            // Set the freq in memory
            clk_freq[(uint8_t) clk] = freq;

            // Calculate the proper PLL frequency
            pll_freq = si5351_multisynth_calc(freq, 0, &ms_reg);

            // Set PLL
            si5351_set_pll(pll_freq, pll_assignment[clk]);

            // Recalculate params for other synths on same PLL
            for (i = 0; i < 6; i++) {
                if (clk_freq[i] != 0) {
                    if (pll_assignment[i] == pll_assignment[clk]) {
                        struct si5351_reg_set temp_reg;
                        uint64_t temp_freq;

                        // Select the proper R div value
                        temp_freq = clk_freq[i];
                        r_div = si5351_select_r_div(&temp_freq);

                        si5351_multisynth_calc(temp_freq, pll_freq, &temp_reg);

                        // If freq > 150 MHz, we need to use DIVBY4 and integer mode
                        if (temp_freq >= SI5351_MULTISYNTH_DIVBY4_FREQ * SI5351_FREQ_MULT) {
                            div_by_4 = 1;
                            int_mode = 1;
                        } else {
                            div_by_4 = 0;
                            int_mode = 0;
                        }

                        // Set multisynth registers
                        si5351_set_ms((enum si5351_clock) i, temp_reg, int_mode, r_div, div_by_4);
                    }
                }
            }

            // Reset the PLL
            si5351_pll_reset(pll_assignment[clk]);
        } else {
            clk_freq[(uint8_t) clk] = freq;

            // Enable the output on first set_freq only
            if (clk_first_set[(uint8_t) clk] == false) {
                si5351_output_enable(clk, 1);
                clk_first_set[(uint8_t) clk] = true;
            }

            // Select the proper R div value
            r_div = si5351_select_r_div(&freq);

            // Calculate the synth parameters
            if (pll_assignment[clk] == SI5351_PLLA) {
                si5351_multisynth_calc(freq, plla_freq, &ms_reg);
            } else {
                si5351_multisynth_calc(freq, pllb_freq, &ms_reg);
            }

            // Set multisynth registers
            si5351_set_ms(clk, ms_reg, int_mode, r_div, div_by_4);

            // Reset the PLL
            //si5351_pll_reset(pll_assignment[clk]);
        }

        return true;
    } else {
        // MS6 and MS7 logic
        // -----------------

        // Lower bounds check
        if (freq > 0 && freq < SI5351_CLKOUT67_MIN_FREQ * SI5351_FREQ_MULT) {
            freq = SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT;
        }

        // Upper bounds check
        if (freq >= SI5351_MULTISYNTH_DIVBY4_FREQ * SI5351_FREQ_MULT) {
            freq = SI5351_MULTISYNTH_DIVBY4_FREQ * SI5351_FREQ_MULT - 1;
        }

        // If one of CLK6 or CLK7 is already set when trying to set the other,
        // we have to ensure that it will also have an integer division ratio
        // with the same PLL, otherwise do not set it.
        if (clk == SI5351_CLK6) {
            if (clk_freq[7] != 0) {
                if (pllb_freq % freq == 0) {
                    if ((pllb_freq / freq) % 2 != 0) {
                        // Not an even divide ratio, no bueno
                        return false;
                    } else {
                        // Set the freq in memory
                        clk_freq[(uint8_t) clk] = freq;

                        // Select the proper R div value
                        r_div = si5351_select_r_div_ms67(&freq);

                        si5351_multisynth67_calc(freq, pllb_freq, &ms_reg);
                    }
                } else {
                    // Not an integer divide ratio, no good
                    return false;
                }
            } else {
                // No previous assignment, so set PLLB based on CLK6

                // Set the freq in memory
                clk_freq[(uint8_t) clk] = freq;

                // Select the proper R div value
                r_div = si5351_select_r_div_ms67(&freq);

                pll_freq = si5351_multisynth67_calc(freq, 0, &ms_reg);
                //pllb_freq = pll_freq;
                si5351_set_pll(pll_freq, SI5351_PLLB);
            }
        } else {
            if (clk_freq[6] != 0) {
                if (pllb_freq % freq == 0) {
                    if ((pllb_freq / freq) % 2 != 0) {
                        // Not an even divide ratio, no bueno
                        return false;
                    } else {
                        // Set the freq in memory
                        clk_freq[(uint8_t) clk] = freq;

                        // Select the proper R div value
                        r_div = si5351_select_r_div_ms67(&freq);

                        si5351_multisynth67_calc(freq, pllb_freq, &ms_reg);
                    }
                } else {
                    // Not an integer divide ratio, no good
                    return false;
                }
            } else {
                // No previous assignment, so set PLLB based on CLK7

                // Set the freq in memory
                clk_freq[(uint8_t) clk] = freq;

                // Select the proper R div value
                r_div = si5351_select_r_div_ms67(&freq);

                pll_freq = si5351_multisynth67_calc(freq, 0, &ms_reg);
                //pllb_freq = pll_freq;
                si5351_set_pll(pll_freq, pll_assignment[clk]);
            }
        }

        div_by_4 = 0;
        int_mode = 0;

        // Set multisynth registers (MS must be set before PLL)
        si5351_set_ms(clk, ms_reg, int_mode, r_div, div_by_4);

        return true;
    }
}

bool si5351_set_freq_manual(uint64_t freq, uint64_t pll_freq, enum si5351_clock clk) {
    struct si5351_reg_set ms_reg;
    uint8_t int_mode = 0;
    uint8_t div_by_4 = 0;

    // Lower bounds check
    if (freq > 0 && freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT) {
        freq = SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT;
    }

    // Upper bounds check
    if (freq > SI5351_CLKOUT_MAX_FREQ * SI5351_FREQ_MULT) {
        freq = SI5351_CLKOUT_MAX_FREQ * SI5351_FREQ_MULT;
    }

    uint8_t r_div;

    clk_freq[(uint8_t) clk] = freq;

    si5351_set_pll(pll_freq, pll_assignment[clk]);

    // Enable the output
    si5351_output_enable(clk, 1);

    // Select the proper R div value
    r_div = si5351_select_r_div(&freq);

    // Calculate the synth parameters
    si5351_multisynth_calc(freq, pll_freq, &ms_reg);

    // If freq > 150 MHz, we need to use DIVBY4 and integer mode
    if (freq >= SI5351_MULTISYNTH_DIVBY4_FREQ * SI5351_FREQ_MULT) {
        div_by_4 = 1;
        int_mode = 1;
    }

    // Set multisynth registers (MS must be set before PLL)
    si5351_set_ms(clk, ms_reg, int_mode, r_div, div_by_4);

    return true;
}

void si5351_set_pll(uint64_t pll_freq, enum si5351_pll target_pll) {
    struct si5351_reg_set pll_reg;

    if (target_pll == SI5351_PLLA) {
        si5351_pll_calc(SI5351_PLLA, pll_freq, &pll_reg, ref_correction[plla_ref_osc], 0);
    } else {
        si5351_pll_calc(SI5351_PLLB, pll_freq, &pll_reg, ref_correction[pllb_ref_osc], 0);
    }

    // Derive the register values to write

    // Prepare an array for parameters to be written to
    uint8_t params[20];
    uint8_t i = 0;
    uint8_t temp;

    // Registers 26-27
    temp = ((pll_reg.p3 >> 8) & 0xFF);
    params[i++] = temp;

    temp = (uint8_t) (pll_reg.p3 & 0xFF);
    params[i++] = temp;

    // Register 28
    temp = (uint8_t) ((pll_reg.p1 >> 16) & 0x03);
    params[i++] = temp;

    // Registers 29-30
    temp = (uint8_t) ((pll_reg.p1 >> 8) & 0xFF);
    params[i++] = temp;

    temp = (uint8_t) (pll_reg.p1 & 0xFF);
    params[i++] = temp;

    // Register 31
    temp = (uint8_t) ((pll_reg.p3 >> 12) & 0xF0);
    temp += (uint8_t) ((pll_reg.p2 >> 16) & 0x0F);
    params[i++] = temp;

    // Registers 32-33
    temp = (uint8_t) ((pll_reg.p2 >> 8) & 0xFF);
    params[i++] = temp;

    temp = (uint8_t) (pll_reg.p2 & 0xFF);
    params[i++] = temp;

    // Write the parameters
    if (target_pll == SI5351_PLLA) {
        si5351_write_bulk(SI5351_PLLA_PARAMETERS, i, params);
        plla_freq = pll_freq;
    } else if (target_pll == SI5351_PLLB) {
        si5351_write_bulk(SI5351_PLLB_PARAMETERS, i, params);
        pllb_freq = pll_freq;
    }
}

void si5351_set_ms(enum si5351_clock clk, struct si5351_reg_set ms_reg, uint8_t int_mode, uint8_t r_div, uint8_t div_by_4) {
    uint8_t params[20];
    uint8_t i = 0;
    uint8_t temp;
    uint8_t reg_val;

    if ((uint8_t) clk <= (uint8_t) SI5351_CLK5) {
        // Registers 42-43 for CLK0
        temp = (uint8_t) ((ms_reg.p3 >> 8) & 0xFF);
        params[i++] = temp;

        temp = (uint8_t) (ms_reg.p3 & 0xFF);
        params[i++] = temp;

        // Register 44 for CLK0
        reg_val = si5351_read((SI5351_CLK0_PARAMETERS + 2) + (clk * 8));
        reg_val &= ~(0x03);
        temp = reg_val | ((uint8_t) ((ms_reg.p1 >> 16) & 0x03));
        params[i++] = temp;

        // Registers 45-46 for CLK0
        temp = (uint8_t) ((ms_reg.p1 >> 8) & 0xFF);
        params[i++] = temp;

        temp = (uint8_t) (ms_reg.p1 & 0xFF);
        params[i++] = temp;

        // Register 47 for CLK0
        temp = (uint8_t) ((ms_reg.p3 >> 12) & 0xF0);
        temp += (uint8_t) ((ms_reg.p2 >> 16) & 0x0F);
        params[i++] = temp;

        // Registers 48-49 for CLK0
        temp = (uint8_t) ((ms_reg.p2 >> 8) & 0xFF);
        params[i++] = temp;

        temp = (uint8_t) (ms_reg.p2 & 0xFF);
        params[i++] = temp;
    } else {
        // MS6 and MS7 only use one register
        temp = ms_reg.p1;
    }

    // Write the parameters
    switch (clk) {
        case SI5351_CLK0:
            si5351_write_bulk(SI5351_CLK0_PARAMETERS, i, params);
            si5351_set_int(clk, int_mode);
            si5351_ms_div(clk, r_div, div_by_4);
            break;
        case SI5351_CLK1:
            si5351_write_bulk(SI5351_CLK1_PARAMETERS, i, params);
            si5351_set_int(clk, int_mode);
            si5351_ms_div(clk, r_div, div_by_4);
            break;
        case SI5351_CLK2:
            si5351_write_bulk(SI5351_CLK2_PARAMETERS, i, params);
            si5351_set_int(clk, int_mode);
            si5351_ms_div(clk, r_div, div_by_4);
            break;
        case SI5351_CLK3:
            si5351_write_bulk(SI5351_CLK3_PARAMETERS, i, params);
            si5351_set_int(clk, int_mode);
            si5351_ms_div(clk, r_div, div_by_4);
            break;
        case SI5351_CLK4:
            si5351_write_bulk(SI5351_CLK4_PARAMETERS, i, params);
            si5351_set_int(clk, int_mode);
            si5351_ms_div(clk, r_div, div_by_4);
            break;
        case SI5351_CLK5:
            si5351_write_bulk(SI5351_CLK5_PARAMETERS, i, params);
            si5351_set_int(clk, int_mode);
            si5351_ms_div(clk, r_div, div_by_4);
            break;
        case SI5351_CLK6:
            si5351_write(SI5351_CLK6_PARAMETERS, temp);
            si5351_ms_div(clk, r_div, div_by_4);
            break;
        case SI5351_CLK7:
            si5351_write(SI5351_CLK7_PARAMETERS, temp);
            si5351_ms_div(clk, r_div, div_by_4);
            break;
    }
}

void si5351_output_enable(enum si5351_clock clk, uint8_t enable) {
    uint8_t reg_val;

    reg_val = si5351_read(SI5351_OUTPUT_ENABLE_CTRL);

    if (enable == 1) {
        reg_val &= ~(1 << (uint8_t) clk);
    } else {
        reg_val |= (1 << (uint8_t) clk);
    }

    si5351_write(SI5351_OUTPUT_ENABLE_CTRL, reg_val);
}

void si5351_drive_strength(enum si5351_clock clk, enum si5351_drive drive) {
    uint8_t reg_val;
    const uint8_t mask = 0x03;

    reg_val = si5351_read(SI5351_CLK0_CTRL + (uint8_t) clk);
    reg_val &= ~(mask);

    switch (drive) {
        case SI5351_DRIVE_2MA:
            reg_val |= 0x00;
            break;
        case SI5351_DRIVE_4MA:
            reg_val |= 0x01;
            break;
        case SI5351_DRIVE_6MA:
            reg_val |= 0x02;
            break;
        case SI5351_DRIVE_8MA:
            reg_val |= 0x03;
            break;
        default:
            break;
    }

    si5351_write(SI5351_CLK0_CTRL + (uint8_t) clk, reg_val);
}

void si5351_update_status(void) {
    si5351_update_sys_status(&dev_status);
    si5351_update_int_status(&dev_int_status);
}

void si5351_set_correction(int32_t corr, enum si5351_pll_input ref_osc) {
    ref_correction[(uint8_t) ref_osc] = corr;

    // Recalculate and set PLL freqs based on correction value
    si5351_set_pll(plla_freq, SI5351_PLLA);
    si5351_set_pll(pllb_freq, SI5351_PLLB);
}

void si5351_set_phase(enum si5351_clock clk, uint8_t phase) {
    // Mask off the upper bit since it is reserved
    phase = phase & 0b01111111;

    si5351_write(SI5351_CLK0_PHASE_OFFSET + (uint8_t) clk, phase);
}

int32_t si5351_get_correction(enum si5351_pll_input ref_osc) {
    return ref_correction[(uint8_t) ref_osc];
}

void si5351_pll_reset(enum si5351_pll target_pll) {
    if (target_pll == SI5351_PLLA) {
        si5351_write(SI5351_PLL_RESET, SI5351_PLL_RESET_A);
    } else if (target_pll == SI5351_PLLB) {
        si5351_write(SI5351_PLL_RESET, SI5351_PLL_RESET_B);
    }
}

void si5351_set_ms_source(enum si5351_clock clk, enum si5351_pll pll) {
    uint8_t reg_val;

    reg_val = si5351_read(SI5351_CLK0_CTRL + (uint8_t) clk);

    if (pll == SI5351_PLLA) {
        reg_val &= ~(SI5351_CLK_PLL_SELECT);
    } else if (pll == SI5351_PLLB) {
        reg_val |= SI5351_CLK_PLL_SELECT;
    }

    si5351_write(SI5351_CLK0_CTRL + (uint8_t) clk, reg_val);

    pll_assignment[(uint8_t) clk] = pll;
}

void si5351_set_int(enum si5351_clock clk, uint8_t enable) {
    uint8_t reg_val;
    reg_val = si5351_read(SI5351_CLK0_CTRL + (uint8_t) clk);

    if (enable == 1) {
        reg_val |= (SI5351_CLK_INTEGER_MODE);
    } else {
        reg_val &= ~(SI5351_CLK_INTEGER_MODE);
    }

    si5351_write(SI5351_CLK0_CTRL + (uint8_t) clk, reg_val);

    // Integer mode indication
    /*
     switch(clk)
     {
     case SI5351_CLK0:
     clk0_int_mode = enable;
     break;
     case SI5351_CLK1:
     clk1_int_mode = enable;
     break;
     case SI5351_CLK2:
     clk2_int_mode = enable;
     break;
     default:
     break;
     }
     */
}

void si5351_set_clock_pwr(enum si5351_clock clk, uint8_t pwr) {
    uint8_t reg_val; //, reg;
    reg_val = si5351_read(SI5351_CLK0_CTRL + (uint8_t) clk);

    if (pwr == 1) {
        reg_val &= 0b01111111;
    } else {
        reg_val |= 0b10000000;
    }

    si5351_write(SI5351_CLK0_CTRL + (uint8_t) clk, reg_val);
}

void si5351_set_clock_invert(enum si5351_clock clk, uint8_t inv) {
    uint8_t reg_val;
    reg_val = si5351_read(SI5351_CLK0_CTRL + (uint8_t) clk);

    if (inv == 1) {
        reg_val |= (SI5351_CLK_INVERT);
    } else {
        reg_val &= ~(SI5351_CLK_INVERT);
    }

    si5351_write(SI5351_CLK0_CTRL + (uint8_t) clk, reg_val);
}

void si5351_set_clock_source(enum si5351_clock clk, enum si5351_clock_source src) {
    uint8_t reg_val;
    reg_val = si5351_read(SI5351_CLK0_CTRL + (uint8_t) clk);

    // Clear the bits first
    reg_val &= ~(SI5351_CLK_INPUT_MASK);

    switch (src) {
        case SI5351_CLK_SRC_XTAL:
            reg_val |= (SI5351_CLK_INPUT_XTAL);
            break;
        case SI5351_CLK_SRC_CLKIN:
            reg_val |= (SI5351_CLK_INPUT_CLKIN);
            break;
        case SI5351_CLK_SRC_MS0:
            if (clk == SI5351_CLK0) {
                return;
            }

            reg_val |= (SI5351_CLK_INPUT_MULTISYNTH_0_4);
            break;
        case SI5351_CLK_SRC_MS:
            reg_val |= (SI5351_CLK_INPUT_MULTISYNTH_N);
            break;
        default:
            return;
    }

    si5351_write(SI5351_CLK0_CTRL + (uint8_t) clk, reg_val);
}

void si5351_set_clock_disable(enum si5351_clock clk, enum si5351_clock_disable dis_state) {
    uint8_t reg_val, reg = SI5351_CLK3_0_DISABLE_STATE;

    if (clk >= SI5351_CLK0 && clk <= SI5351_CLK3) {
        reg = SI5351_CLK3_0_DISABLE_STATE;
    } else if (clk >= SI5351_CLK4 && clk <= SI5351_CLK7) {
        reg = SI5351_CLK7_4_DISABLE_STATE;
    }

    reg_val = si5351_read(reg);

    if (clk >= SI5351_CLK0 && clk <= SI5351_CLK3) {
        reg_val &= ~(0b11 << (clk * 2));
        reg_val |= dis_state << (clk * 2);
    } else if (clk >= SI5351_CLK4 && clk <= SI5351_CLK7) {
        reg_val &= ~(0b11 << ((clk - 4) * 2));
        reg_val |= dis_state << ((clk - 4) * 2);
    }

    si5351_write(reg, reg_val);
}

void si5351_set_clock_fanout(enum si5351_clock_fanout fanout, uint8_t enable) {
    uint8_t reg_val;
    reg_val = si5351_read(SI5351_FANOUT_ENABLE);

    switch (fanout) {
        case SI5351_FANOUT_CLKIN:
            if (enable) {
                reg_val |= SI5351_CLKIN_ENABLE;
            } else {
                reg_val &= ~(SI5351_CLKIN_ENABLE);
            }
            break;
        case SI5351_FANOUT_XO:
            if (enable) {
                reg_val |= SI5351_XTAL_ENABLE;
            } else {
                reg_val &= ~(SI5351_XTAL_ENABLE);
            }
            break;
        case SI5351_FANOUT_MS:
            if (enable) {
                reg_val |= SI5351_MULTISYNTH_ENABLE;
            } else {
                reg_val &= ~(SI5351_MULTISYNTH_ENABLE);
            }
            break;
    }

    si5351_write(SI5351_FANOUT_ENABLE, reg_val);
}

void si5351_set_pll_input(enum si5351_pll pll, enum si5351_pll_input input) {
    uint8_t reg_val;
    reg_val = si5351_read(SI5351_PLL_INPUT_SOURCE);

    // Clear the bits first
    //reg_val &= ~(SI5351_CLKIN_DIV_MASK);

    switch (pll) {
        case SI5351_PLLA:
            if (input == SI5351_PLL_INPUT_CLKIN) {
                reg_val |= SI5351_PLLA_SOURCE;
                reg_val |= clkin_div;
                plla_ref_osc = SI5351_PLL_INPUT_CLKIN;
            } else {
                reg_val &= ~(SI5351_PLLA_SOURCE);
                plla_ref_osc = SI5351_PLL_INPUT_XO;
            }
            break;
        case SI5351_PLLB:
            if (input == SI5351_PLL_INPUT_CLKIN) {
                reg_val |= SI5351_PLLB_SOURCE;
                reg_val |= clkin_div;
                pllb_ref_osc = SI5351_PLL_INPUT_CLKIN;
            } else {
                reg_val &= ~(SI5351_PLLB_SOURCE);
                pllb_ref_osc = SI5351_PLL_INPUT_XO;
            }
            break;
        default:
            return;
    }

    si5351_write(SI5351_PLL_INPUT_SOURCE, reg_val);

    si5351_set_pll(plla_freq, SI5351_PLLA);
    si5351_set_pll(pllb_freq, SI5351_PLLB);
}

void si5351_set_vcxo(uint64_t pll_freq, uint8_t ppm) {
    struct si5351_reg_set pll_reg;
    uint64_t vcxo_param;

    // Bounds check
    if (ppm < SI5351_VCXO_PULL_MIN) {
        ppm = SI5351_VCXO_PULL_MIN;
    }

    if (ppm > SI5351_VCXO_PULL_MAX) {
        ppm = SI5351_VCXO_PULL_MAX;
    }

    // Set PLLB params
    vcxo_param = si5351_pll_calc(SI5351_PLLB, pll_freq, &pll_reg, ref_correction[pllb_ref_osc], 1);

    // Derive the register values to write

    // Prepare an array for parameters to be written to
    uint8_t params[20];
    uint8_t i = 0;
    uint8_t temp;

    // Registers 26-27
    temp = ((pll_reg.p3 >> 8) & 0xFF);
    params[i++] = temp;

    temp = (uint8_t) (pll_reg.p3 & 0xFF);
    params[i++] = temp;

    // Register 28
    temp = (uint8_t) ((pll_reg.p1 >> 16) & 0x03);
    params[i++] = temp;

    // Registers 29-30
    temp = (uint8_t) ((pll_reg.p1 >> 8) & 0xFF);
    params[i++] = temp;

    temp = (uint8_t) (pll_reg.p1 & 0xFF);
    params[i++] = temp;

    // Register 31
    temp = (uint8_t) ((pll_reg.p3 >> 12) & 0xF0);
    temp += (uint8_t) ((pll_reg.p2 >> 16) & 0x0F);
    params[i++] = temp;

    // Registers 32-33
    temp = (uint8_t) ((pll_reg.p2 >> 8) & 0xFF);
    params[i++] = temp;

    temp = (uint8_t) (pll_reg.p2 & 0xFF);
    params[i++] = temp;

    // Write the parameters
    si5351_write_bulk(SI5351_PLLB_PARAMETERS, i, params);

    // Write the VCXO parameters
    vcxo_param = ((vcxo_param * ppm * SI5351_VCXO_MARGIN) / 100ULL) / 1000000ULL;

    temp = (uint8_t) (vcxo_param & 0xFF);
    si5351_write(SI5351_VXCO_PARAMETERS_LOW, temp);

    temp = (uint8_t) ((vcxo_param >> 8) & 0xFF);
    si5351_write(SI5351_VXCO_PARAMETERS_MID, temp);

    temp = (uint8_t) ((vcxo_param >> 16) & 0x3F);
    si5351_write(SI5351_VXCO_PARAMETERS_HIGH, temp);
}

void si5351_set_ref_freq(uint32_t ref_freq, enum si5351_pll_input ref_osc) {
    //uint8_t reg_val;
    //reg_val = si5351_read(SI5351_PLL_INPUT_SOURCE);

    // Clear the bits first
    //reg_val &= ~(SI5351_CLKIN_DIV_MASK);

    if (ref_freq <= 30000000UL) {
        xtal_freq[(uint8_t) ref_osc] = ref_freq;
        //reg_val |= SI5351_CLKIN_DIV_1;
        if (ref_osc == SI5351_PLL_INPUT_CLKIN) {
            clkin_div = SI5351_CLKIN_DIV_1;
        }
    } else if (ref_freq > 30000000UL && ref_freq <= 60000000UL) {
        xtal_freq[(uint8_t) ref_osc] = ref_freq / 2;
        //reg_val |= SI5351_CLKIN_DIV_2;
        if (ref_osc == SI5351_PLL_INPUT_CLKIN) {
            clkin_div = SI5351_CLKIN_DIV_2;
        }
    } else if (ref_freq > 60000000UL && ref_freq <= 100000000UL) {
        xtal_freq[(uint8_t) ref_osc] = ref_freq / 4;
        //reg_val |= SI5351_CLKIN_DIV_4;
        if (ref_osc == SI5351_PLL_INPUT_CLKIN) {
            clkin_div = SI5351_CLKIN_DIV_4;
        }
    } else {
        //reg_val |= SI5351_CLKIN_DIV_1;
    }

    //si5351_write(SI5351_PLL_INPUT_SOURCE, reg_val);
}

uint8_t si5351_write_bulk(uint8_t addr, uint8_t bytes, uint8_t *data) {
    return (si5351_write_xfer(addr, data, bytes));
}

uint8_t si5351_write(uint8_t addr, uint8_t data) {
    si5351_write_byte(addr, data);
    return (0);
}

uint8_t si5351_read(uint8_t addr) {
    return (si5351_read_byte(addr));
}
