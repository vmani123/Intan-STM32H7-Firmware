# Intan-STM32H7-Firmware
Contains the firmware to receive data from Intan's RHD2164 chip on an STM32H7.

This code was developed/tested on an STM32 H723ZG devboard. It consists mostly of Intan's firmware framework from this repository, with a couple of edits to the code in order to make it usable given our circumstances.

Give the Intan firmware framework a read through here: https://intantech.com/files/Intan_RHD_STM32_Framework.pdf.

The STM32H7 communicates via SPI to the Intan RDH2164 chip. In our case, we were unable to communicate at the full speed (~20 MHz SPI transmit, ~40 Mhz DDR SPI receive), and slowed the transmission speed down. Due to how custom clocks are used for some the communication, this makes some of the timing logic different from Intan's, but the gist is that we just slowed transmission down to ~1.4 Mhz for the SPI transmit (~2.8 Mhz for the DDR receive), and adjusted the timers that govern the SPI receive accordingly. We also had issues with triggering the CS_DELAY_TIM outlined in the Intan's document, so we do it differently with an interrupt off of another GPIO that has the same CS signal routed to it. 
