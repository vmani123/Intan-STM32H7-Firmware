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

#include "userfunctions.h"
#include <stddef.h>
#include <math.h>
#include "arm_math.h"
//#include "arm_rms_q15.c"



uint16_t samples[2 * NUM_SAMPLED_CHANNELS];
uint16_t samples_2[2 * NUM_SAMPLED_CHANNELS];

uint16_t samples_50_ms_1[2*NUM_SAMPLED_CHANNELS][50];
uint16_t samples_50_ms_2[2*NUM_SAMPLED_CHANNELS][50];

uint16_t sampled_rms_1[2*NUM_SAMPLED_CHANNELS + 1];
uint16_t sampled_rms_2[2*NUM_SAMPLED_CHANNELS + 1];

int sample_counter_1 = 0;
int sample_counter_2 = 0;
int counter = 0;

// Specify condition that should result in the main while loop ending.
// By default, escape once 1 second of data has been gathered.
int loop_escape()
{
	// Escape once sample memory capacity (default 1 second of data) has been reached.
#ifdef OFFLINE_TRANSFER
	return sample_counter > per_channel_sample_memory_capacity;
#else
	return 0;
#endif
}


// Write any desired data from this sequence to memory.
// By default, only the result corresponding to a CONVERT on FIRST_SAMPLED_CHANNEL is saved per sequence.
void write_data_to_memory()
{
#ifdef OFFLINE_TRANSFER
	// Extract 2 16-bit samples interleaved (stream A and stream B) from each 32-bit word, and save to sample_memory.
	for (int i = 0; i < NUM_SAMPLED_CHANNELS; i++) {
		extract_ddr_words(command_sequence_MISO[FIRST_SAMPLED_CHANNEL + i + 2],
				&sample_memory[(sample_counter * NUM_SAMPLED_CHANNELS * 2) + i],
				&sample_memory[(sample_counter * NUM_SAMPLED_CHANNELS * 2) + i + NUM_SAMPLED_CHANNELS],
				command_sequence_MISO_2[FIRST_SAMPLED_CHANNEL + i + 2],
				&sample_memory_2[(sample_counter * NUM_SAMPLED_CHANNELS * 2) + i],
				&sample_memory_2[(sample_counter * NUM_SAMPLED_CHANNELS * 2) + i + NUM_SAMPLED_CHANNELS]);


	}
	sample_counter++;

//	//	 Read results of aux command slots (not used in this sample example).
//	//	 For more advanced programs that require reading of aux command results, those would be read and saved here.
//	uint16_t aux0_result_A, aux0_result_B;
//	uint16_t aux1_result_A, aux1_result_B;
//	uint16_t aux2_result_A, aux2_result_B;
//	extract_ddr_words(command_sequence_MISO[34], &aux0_result_A, &aux0_result_B); // Result of AUX SLOT 1 from this command sequence
//	extract_ddr_words(command_sequence_MISO[0],  &aux1_result_A, &aux1_result_B); // Result of AUX SLOT 2 from the previous command sequence
//	extract_ddr_words(command_sequence_MISO[1],  &aux2_result_A, &aux2_result_B); // Result of AUX SLOT 3 from the previous command sequence
#endif
}


/**
  @brief         Q15 square root function.
  @param[in]     in    input value.  The range of the input value is [0 +1) or 0x0000 to 0x7FFF
  @param[out]    pOut  points to square root of input value
  @return        execution status
                   - \ref ARM_MATH_SUCCESS        : input value is positive
                   - \ref ARM_MATH_ARGUMENT_ERROR : input value is negative; *pOut is set to 0
 */

arm_status arm_sqrt_q15(
  q15_t in,
  q15_t * pOut)
{
  q31_t bits_val1;
  q15_t number, temp1, var1, signBits1, half;
  float32_t temp_float1;
  union
  {
    q31_t fracval;
    float32_t floatval;
  } tempconv;

  number = in;

  /* If the input is a positive number then compute the signBits. */
  if (number > 0)
  {
    signBits1 = __CLZ(number) - 17;

    /* Shift by the number of signBits1 */
    if ((signBits1 % 2) == 0)
    {
      number = number << signBits1;
    }
    else
    {
      number = number << (signBits1 - 1);
    }

    /* Calculate half value of the number */
    half = number >> 1;
    /* Store the number for later use */
    temp1 = number;

    /* Convert to float */
    temp_float1 = number * 3.051757812500000e-005f;
    /* Store as integer */
    tempconv.floatval = temp_float1;
    bits_val1 = tempconv.fracval;
    /* Subtract the shifted value from the magic number to give intial guess */
    bits_val1 = 0x5f3759df - (bits_val1 >> 1);  /* gives initial guess */
    /* Store as float */
    tempconv.fracval = bits_val1;
    temp_float1 = tempconv.floatval;
    /* Convert to integer format */
    var1 = (q31_t) (temp_float1 * 16384);

    /* 1st iteration */
    var1 = ((q15_t) ((q31_t) var1 * (0x3000 -
                                     ((q15_t)
                                      ((((q15_t)
                                         (((q31_t) var1 * var1) >> 15)) *
                                        (q31_t) half) >> 15))) >> 15)) << 2;
    /* 2nd iteration */
    var1 = ((q15_t) ((q31_t) var1 * (0x3000 -
                                     ((q15_t)
                                      ((((q15_t)
                                         (((q31_t) var1 * var1) >> 15)) *
                                        (q31_t) half) >> 15))) >> 15)) << 2;
    /* 3rd iteration */
    var1 = ((q15_t) ((q31_t) var1 * (0x3000 -
                                     ((q15_t)
                                      ((((q15_t)
                                         (((q31_t) var1 * var1) >> 15)) *
                                        (q31_t) half) >> 15))) >> 15)) << 2;

    /* Multiply the inverse square root with the original value */
    var1 = ((q15_t) (((q31_t) temp1 * var1) >> 15)) << 1;

    /* Shift the output down accordingly */
    if ((signBits1 % 2) == 0)
    {
      var1 = var1 >> (signBits1 / 2);
    }
    else
    {
      var1 = var1 >> ((signBits1 - 1) / 2);
    }
    *pOut = var1;

    return (ARM_MATH_SUCCESS);
  }
  /* If the number is a negative number then store zero as its square root value */
  else
  {
    *pOut = 0;

    return (ARM_MATH_ARGUMENT_ERROR);
  }
}


void arm_rms_q15(
  const q15_t * pSrc,
        uint32_t blockSize,
        q15_t * pResult)
{
        uint32_t blkCnt;                               /* Loop counter */
        q63_t sum = 0;                                 /* Temporary result storage */
        q15_t in;                                      /* Temporary variable to store input value */

#if defined (ARM_MATH_LOOPUNROLL) && defined (ARM_MATH_DSP)
        q31_t in32;                                    /* Temporary variable to store input value */
#endif

#if defined (ARM_MATH_LOOPUNROLL)

  /* Loop unrolling: Compute 4 outputs at a time */
  blkCnt = blockSize >> 2U;

  while (blkCnt > 0U)
  {
    /* C = A[0] * A[0] + A[1] * A[1] + ... + A[blockSize-1] * A[blockSize-1] */

    /* Compute sum of squares and store result in a temporary variable. */
#if defined (ARM_MATH_DSP)
    in32 = read_q15x2_ia ((q15_t **) &pSrc);
    sum = __SMLALD(in32, in32, sum);

    in32 = read_q15x2_ia ((q15_t **) &pSrc);
    sum = __SMLALD(in32, in32, sum);
#else
    in = *pSrc++;
    sum += ((q31_t) in * in);

    in = *pSrc++;
    sum += ((q31_t) in * in);

    in = *pSrc++;
    sum += ((q31_t) in * in);

    in = *pSrc++;
    sum += ((q31_t) in * in);
#endif /* #if defined (ARM_MATH_DSP) */

    /* Decrement loop counter */
    blkCnt--;
  }

  /* Loop unrolling: Compute remaining outputs */
  blkCnt = blockSize % 0x4U;

#else

  /* Initialize blkCnt with number of samples */
  blkCnt = blockSize;

#endif /* #if defined (ARM_MATH_LOOPUNROLL) */

  while (blkCnt > 0U)
  {
    /* C = A[0] * A[0] + A[1] * A[1] + ... + A[blockSize-1] * A[blockSize-1] */

    in = *pSrc++;
    /* Compute sum of squares and store result in a temporary variable. */
    sum += ((q31_t) in * in);

    /* Decrement loop counter */
    blkCnt--;
  }

  /* Truncating and saturating the accumulator to 1.15 format */
  /* Store result in destination */
  arm_sqrt_q15(__SSAT((sum / (q63_t)blockSize) >> 15, 16), pResult);
}

/**
  @} end of RMS group
 */

//Add data to a buffer that contains 50 ms of data from each channel
//If the count of samples is high enough, calculcate the RMS and send that over SPI
//Include a padded/magic number before and after for readability
void send_rms_as_necessary(int called_from)
{
	if(called_from==1)
	{
		//time to send!
		if(sample_counter_1==50){
			sample_counter_1 = 0;

			//calculcate RMS for each channel
			for(int i=0;i<NUM_SAMPLED_CHANNELS*2;i++)
			{
				arm_rms_q15((q15_t*)samples_50_ms_1[i], 50, (q15_t*) &sampled_rms_1[i+1]);
			}

			//send over SPI
			sampled_rms_1[0] = 0x0F0F;


			transmit_dma_to_spi(&hspi4, sampled_rms_1, sizeof(sampled_rms_1));
//			transmit_dma_to_spi(hspi, tx_data, num_bytes)


		}
		else //just add to buffer
		{
			sample_counter_1++;
			for(int i=0;i<NUM_SAMPLED_CHANNELS;i++)
			{
				extract_ddr_words(command_sequence_MISO[FIRST_SAMPLED_CHANNEL + i + 2],
						&samples_50_ms_1[i][sample_counter_1],
						&samples_50_ms_1[i + NUM_SAMPLED_CHANNELS][sample_counter_1]);
			}

		}
	} else {
		//time to send!
		if(sample_counter_2==50){
			sample_counter_2 = 0;

			//calculcate RMS for each channel
			for(int i=0;i<NUM_SAMPLED_CHANNELS*2;i++)
			{
				arm_rms_q15((q15_t*)samples_50_ms_1[i], 50, (q15_t*) &sampled_rms_1[i+1]);
			}

			//send over SPI
			sampled_rms_1[0] = 2;

			transmit_dma_to_spi(&hspi4, sampled_rms_2, sizeof(sampled_rms_2));


		}
		else //just add to buffer
		{
			sample_counter_2++;
			for(int i=0;i<NUM_SAMPLED_CHANNELS;i++)
			{
				extract_ddr_words(command_sequence_MISO[FIRST_SAMPLED_CHANNEL + i + 2],
						&samples_50_ms_2[i][sample_counter_2],
						&samples_50_ms_2[i + NUM_SAMPLED_CHANNELS][sample_counter_2]);
			}

		}
	}


}


// Determine if data is ready to be transmitted, and if so, transmit (for example via USART).
void transmit_data_realtime()
{
#ifndef OFFLINE_TRANSFER
	// By default, do nothing (default example program will only transmit all data at once after acquisition
	// period has finished). So, this function (which is executed once per interrupt routine) should do nothing.

	// If instead, real-time data transfer is desired, user should uncomment the code below.
	// Note that unless loop_escape() is altered, main loop will exit after a period, at which point realtime data
	// transfer will stop. If this is not desired, change loop_escape() so that it never returns 1.


	// IMPORTANT NOTE - Data is written to memory from SPI through DMA, and read from memory to USART through DMA.
	// DMA transmission is automatic, so if it takes too long for USART data to transmit, it's possible for the next sample
	// of data to be writing into memory before the USART read completes. Reading and writing at the same time leads to data corruption.
	// If you uncomment the following code, the data in memory will be overwritten with hardcoded integer values.
	// This allows for obvious detection of corrupted data, as anything transmitted across USART that's not an integer between 0 and
	// CONVERT_COMMANDS_PER_SEQUENCE + AUX_COMMANDS_PER_SEQUENCE will be a result of corruption.
	// Data corruption is more likely to occur with larger NUM_CHANNELS_TO_TRANSMIT, slower USART Baud rate, and faster SPI Baud rate.
//	for (int i = 0; i < CONVERT_COMMANDS_PER_SEQUENCE + AUX_COMMANDS_PER_SEQUENCE; i++) {
//		command_sequence_MISO[i] = i;
//	}

	for (int i = 0; i < NUM_SAMPLED_CHANNELS; i++) {
		extract_ddr_words(command_sequence_MISO[FIRST_SAMPLED_CHANNEL + i + 2],
				&samples[i],
				&samples[i + NUM_SAMPLED_CHANNELS]);
	}

	counter++;

	if(counter>=4500){
		samples[0] = 1000;
		samples[1] = 2000;
		samples[2] = 3000;
		samples[3] = 4000;

		if(counter==5000)
		{
			counter = 0;
		}

	}


	transmit_dma_to_usart(samples, NUM_SAMPLED_CHANNELS * 2 * sizeof(uint16_t));
	transmit_dma_to_usart(samples_2 , NUM_SAMPLED_CHANNELS * 2 * sizeof(uint16_t));
#endif
}


// Transmit accumulated data after acquisition has finished (for example via USART).
void transmit_data_offline()
{
	// This is a relatively large transfer, too much for a single HAL DMA function call.
	// Ideally, we'd do something like:
	//	if (HAL_UART_Transmit(&USART, (uint8_t*) &sample_memory[0], NUM_SAMPLED_CHANNELS * SAMPLES_IN_MEMORY * 2 * sizeof(uint16_t), HAL_MAX_DELAY) != HAL_OK)
	//	{
	//		Error_Handler();
	//	}
	// but, 320,000 byte (if NUM_SAMPLED_CHANNELS is 4 and SAMPLES_IN_MEMORY is 20000) transfer too much for a single HAL function call.

	// 2*samples_per_chunk needs to fit into a uint16_t (max value 65535), so the max value of samples_per_chunk
	// is 32767. Ideally, total_samples_in_memory divides into this value cleanly, so 20000 is a reasonable candidate.
	// However, for reasons that are unclear, at high Baud rates, large transfers seem more likely to fail. So, dividing
	// into very small chunks seems to be the most reliable at high Baud rates.

	// We do the same thing for LL, for consistency - optimized performance is not critical for offline transfers, so there is likely
	// no significant downside to chunking data into many smaller transfers.

	uint16_t samples_per_chunk = 1;
	uint32_t total_samples_in_memory = NUM_SAMPLED_CHANNELS * 2 * calculate_sample_rate() * NUMBER_OF_SECONDS_TO_ACQUIRE;
	uint32_t num_chunks = floor(total_samples_in_memory / samples_per_chunk);
	uint16_t remaining_samples = total_samples_in_memory % samples_per_chunk;

	// Transmit multiple complete chunks of data
	for (int i = 0; i < num_chunks; i++) {
		uart_ready = 0;
		transmit_dma_to_usart(&sample_memory[samples_per_chunk * i], samples_per_chunk * sizeof(uint16_t));
		while (uart_ready != 1) {}
		uart_ready = 0;
		transmit_dma_to_usart(&sample_memory_2[samples_per_chunk * i], samples_per_chunk * sizeof(uint16_t));
		while (uart_ready != 1) {}

	}

	// Transmit any remaining data too small to fit in a complete chunk
	if (remaining_samples > 0) {
		uart_ready = 0;
		transmit_dma_to_usart(&sample_memory[samples_per_chunk * num_chunks], remaining_samples * sizeof(uint16_t));
		while (uart_ready != 1) {}
		uart_ready = 0;
		transmit_dma_to_usart(&sample_memory_2[samples_per_chunk * num_chunks], remaining_samples * sizeof(uint16_t));
		while (uart_ready != 1) {}
	}
}


// Configure and transmit register values.
// Initial register values default to the same default settings in the RHX software.
// Any desired changes to these values added after the 'write_initial_reg_values()' function call.
void configure_registers(RHDConfigParameters *parameters)
{
	write_initial_reg_values(parameters);

	/* Make any changes that differ from defaults here. For example, configure registers 5 and 7 for impedance check: */

//	// Reg 5: Set zcheck_DAC_power, zcheck_en, zcheck_scale, and zcheck_polarity
//	parameters->zcheck_DAC_power = 1;
//	parameters->zcheck_en = 1;
//	set_zcheck_scale(parameters, ZcheckCs1pF);
//	set_zcheck_polarity(parameters, ZcheckPositiveInput);
//	write_command(5, get_register_value(parameters, 5));
//
//	// Reg 6: (Actual DAC value which changes over time - instead of setting once here, this should be written sample-by-sample in an aux command list).
//
//	// Reg 7: Set zcheck_select
//	set_zcheck_channel(parameters, FIRST_SAMPLED_CHANNEL);
//	write_command(7, get_register_value(parameters, 7));
}


// Configure the CONVERT commands that are loaded at the beginning of command_sequence_MOSI.
// By default, channels from 0 to CONVERT_COMMANDS_PER_SEQUENCE - 1 (0 to 31) are loaded consecutively (0, 1, 2, 3, ... 31).
void configure_convert_commands()
{
	// If default ordering of channel CONVERT commands (0, 1, 2, 3, ... 31) is desired, pass a NULL 2nd parameter to create_convert_sequence().
	create_convert_sequence(NULL);

	// If a custom ordering of channel CONVERT commands is instead desired, create a uint8_t array of size CONVERT_COMMANDS_PER_SEQUENCE
	// and populate each entry with the desired channel number. Then pass this array as the 2nd parameter to create_convert_sequence().
	// For example, if sampling in descending order from CONVERT_COMMANDS_PER_SEQUENCE - 1 (31 to 0) is desired:
	//	uint8_t channel_numbers[CONVERT_COMMANDS_PER_SEQUENCE] = {0};
	//	for (int i = 0; i < CONVERT_COMMANDS_PER_SEQUENCE; i++) {
	//		channel_numbers[i] = (CONVERT_COMMANDS_PER_SEQUENCE - 1) - i;
	//	}
	//	create_convert_sequence(channel_numbers);
}


// Configure the AUX commands that are loaded at the end of command_sequence_MOSI.
// By defaults, command lists from 0 to AUX_COMMANDS_PER_SEQUENCE - 1 (0 to 2) are loaded consecutively (32, 33, 34).
void configure_aux_commands(RHDConfigParameters *parameters)
{
	  // All create_command_list functions return -1 to indicate failure.
	  // Additionally, they should all be used to create command lists of length AUX_COMMAND_LIST_LENGTH, except
	  // for create_command_list_zcheck_DAC. This function returns a command list with a length that depends on the
	  // desired frequency, so if using this command list it's important to set zcheck_DAC_command_slot_position to 0, 1, or
	  // 2 (one of the 3 command slots) to indicate its position, and set zcheck_DAC_command_list_length so that during
	  // execution of this list, after the length has been reached it can begin at 0 again.

	// Slot 0: Write RHD register loading to aux_command_list[0], so that the register values saved in software (parameters) are continually re-written.
	create_command_list_RHD_register_config(parameters, (uint16_t*) aux_command_list[0], 0, AUX_COMMAND_LIST_LENGTH);

	// Slot 1: Write dummy reads to aux_command_list[1], so that register 40 is repeatedly read.
	create_command_list_dummy(parameters, (uint16_t*) aux_command_list[1], AUX_COMMAND_LIST_LENGTH, read_command(40));

	// Slot 2: Write dummy reads to aux_command_list[2], so that register 41 is repeatedly read.
	create_command_list_dummy(parameters, (uint16_t*) aux_command_list[2], AUX_COMMAND_LIST_LENGTH, read_command(41));

	// NOTE: If an impedance check command list is desired, it is created and used slightly differently, because its length is not AUX_COMMAND_LIST_LENGTH
	// but rather depends on impedance test signal frequency. For this demonstration, a zcheck_DAC command list is created and populates aux command slot 2.
	// In this case, the above creation of a dummy command on slot 2 would be redundant and should be commented out.

	// Write impedance check DAC control to aux_command_list[2], so that a sine wave is approximated by the DAC.
	// Note that, as opposed to all other command lists which should be AUX_COMMAND_LIST_LENGTH long, these
	// zcheck_DAC commands can have different lengths depending on desired frequency. To handle this, be sure to:
	// a) assign create_command_list_zcheck_DAC()'s return value to zcheck_DAC_command_list_length, and
	// b) assign which command slot the zcheck_DAC command list is in to zcheck_DAC_command_slot_position.
//	zcheck_DAC_command_list_length = create_command_list_zcheck_DAC(parameters, (uint16_t*) aux_command_list[2], 1000.0, 100);
//	zcheck_DAC_command_slot_position = 2;
}


// Use DMA to transmit num_bytes of data from memory pointer tx_data directly to USART.
// Non-blocking, so it may be helpful to set the 'uart_ready' variable to 0 prior to this function call,
// monitor it, and hold off on further transmissions until the USART Tx complete callback sets it to 1.
void transmit_dma_to_usart(volatile uint16_t *tx_data, uint16_t num_bytes)
{
#ifdef USE_HAL
	if (HAL_UART_Transmit_DMA(&USART, (uint8_t*) tx_data, num_bytes) != HAL_OK)
	{
		Error_Handler();
	}
#else
	// Configure the DMA channel data size
	LL_DMA_SetDataLength(DMA, DMA_USART_CHANNEL, num_bytes);

	// Clear all interrupt flags
	LL_DMA_ClearFlag_TC2(DMA);
	LL_DMA_ClearFlag_DME2(DMA);
	LL_DMA_ClearFlag_FE2(DMA);
	LL_DMA_ClearFlag_HT2(DMA);
	LL_DMA_ClearFlag_TE2(DMA);

	// Configure DMA channel source address
	LL_DMA_SetMemoryAddress(DMA, DMA_USART_CHANNEL, (uint32_t) tx_data);

	// Configure DMA channel destination address
	LL_DMA_SetPeriphAddress(DMA, DMA_USART_CHANNEL, LL_USART_DMA_GetRegAddr(USART, LL_USART_DMA_REG_DATA_TRANSMIT));

	// Enable common interrupts: Transfer Complete and Transfer Errors ITs
	LL_DMA_EnableIT_TC(DMA, DMA_USART_CHANNEL);
	LL_DMA_EnableIT_DME(DMA, DMA_USART_CHANNEL);
	LL_DMA_EnableIT_FE(DMA, DMA_USART_CHANNEL);

	// Clear TC flag in ICR register
	LL_USART_ClearFlag_TC(USART);

	// Enable DMA channel
	LL_DMA_EnableStream(DMA, DMA_USART_CHANNEL);

	// Enable DMA transfer for transmit request by setting DMAT bit in UART CR3 register
	LL_USART_EnableDMAReq_TX(USART);
#endif
}

void transmit_dma_to_spi(SPI_HandleTypeDef* hspi, volatile uint16_t* tx_data, uint32_t num_bytes)
{
	if(HAL_SPI_Transmit_DMA(hspi, (uint8_t*) tx_data, num_bytes)!= HAL_OK)
	{
		Error_Handler();
	}
}

