/*
  Intan Technologies RHD STM32 Firmware Framework
  Version 1.1

  Copyright (c) 2024 Intan Technologies

  This file is part of the Intan Technologies RHD STM32 Firmware Framework.

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the “Software”), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.


  See <http://www.intantech.com> for documentation and product information.
  
 */

#ifndef INC_RHDREGISTERS_H_
#define INC_RHDREGISTERS_H_

#include <stdint.h>

typedef enum ZcheckCs {
	ZcheckCs100fF,
	ZcheckCs1pF,
	ZcheckCs10pF
} ZcheckCs;

typedef enum ZcheckPolarity {
	ZcheckPositiveInput,
	ZcheckNegativeInput
} ZcheckPolarity;

typedef enum RHDCommandType {
	RHDCommandConvert,
	RHDCommandCalibrate,
	RHDCommandCalClear,
	RHDCommandRegWrite,
	RHDCommandRegRead
} RHDCommandType;

typedef struct rhdconfigparameters {
	double sample_rate;
	int adc_reference_bw;
	int amp_vref_enable;
	int adc_comparator_bias;
	int adc_comparator_select;
	int vdd_sense_enable;
	int adc_buffer_bias;
	int mux_bias;
	int mux_load;
	int temp_S1;
	int temp_S2;
	int temp_en;
	int digOut;
	int digOut_hiZ;
	int weak_miso;
	int twos_comp;
	int abs_mode;
	int DSP_en;
	int DSP_cutoff_freq;
	int zcheck_DAC_power;
	int zcheck_load;
	int zcheck_scale;
	int zcheck_conn_all;
	int zcheck_sel_pol;
	int zcheck_en;
	int zcheck_select;
	int off_chip_RH1;
	int off_chip_RH2;
	int off_chip_RL;
	int adc_Aux1_en;
	int adc_Aux2_en;
	int adc_Aux3_en;
	int rH1_DAC1;
	int rH1_DAC2;
	int rH2_DAC1;
	int rH2_DAC2;
	int rL_DAC1;
	int rL_DAC2;
	int rL_DAC3;

	int amp_pwr[64];
	int amp_fast_settle;

} RHDConfigParameters;

void set_DigOut_low(RHDConfigParameters *p);
void set_DigOut_high(RHDConfigParameters *p);
void set_DigOut_hiZ(RHDConfigParameters *p);

double set_DSP_cutoff_freq(RHDConfigParameters *p, double new_DSP_cutoff_freq);
double get_DSP_cutoff_freq(RHDConfigParameters *p);

void set_zcheck_scale(RHDConfigParameters *p, ZcheckCs scale);
void set_zcheck_polarity(RHDConfigParameters *p, ZcheckPolarity polarity);
int set_zcheck_channel(RHDConfigParameters *p, int channel);

void set_amp_powered(RHDConfigParameters *p, int channel, uint8_t powered);
void power_up_all_amps(RHDConfigParameters *p);
void power_down_all_amps(RHDConfigParameters *p);

double set_upper_bandwidth(RHDConfigParameters *p, double upper_bandwidth);
double set_lower_bandwidth(RHDConfigParameters *p, double lower_bandwidth);

double rH1_from_upper_bandwidth(double upper_bandwidth);
double rH2_from_upper_bandwidth(double upper_bandwidth);
double rL_from_lower_bandwidth(double lower_bandwidth);
double upper_bandwidth_from_rH1(double rH1);
double upper_bandwidth_from_rH2(double rH2);
double lower_bandwidth_from_rL(double rL);

int max_num_channels_per_chip();

void set_default_rhd_settings(RHDConfigParameters *p);

void set_biases_based_on_sample_rate(int *adc_buffer_bias, int *mux_bias, const double sample_rate);

uint16_t get_register_value(RHDConfigParameters *p, int reg);

uint16_t convert_command(uint8_t channel, uint8_t h_bit);
uint16_t calibrate_command();
uint16_t clear_command();
uint16_t write_command(uint8_t reg_addr, uint8_t data);
uint16_t read_command(uint8_t reg_addr);

#endif /* INC_RHDREGISTERS_H_ */
