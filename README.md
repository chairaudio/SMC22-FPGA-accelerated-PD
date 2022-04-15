# Terasic DE10-Nano running Pure Data with FPGA DSP Subsystem

This source code accompanies the paper Wegener, Clemens et al. (2022). “FPGA-accelerated Real-Time Audio in Pure Data”. In: *Proceedings of the 19th Sound & Music Computing Conference*. Saint-Étienne: Université Jean Monnet of Saint-Étienne.

## Overview

This repository contains the FPGA design (Quartus project) for enabling WM8731 sound card support and a pass-through audio stream from Pure Data running on Linux (ARM core) and the FPGA fabric. 

The sound card support is based on Bjarne Steinsbo's Source Code:
https://github.com/bsteinsbo/DE1-SoC-Sound

Sound card support was adapted to the DE10-Nano and a newer Linux kernel (linux-socfpga-5.4.54-lts). 

## How to use

This system was built and tested with the Linux Console (kernel 4.5) image as a starting point from https://www.terasic.com.tw/cgi-bin/page/archive.pl?Language=English&CategoryNo=165&No=1046&PartNo=4. Other images might work as well. Keep in mind, that the digital logic of the video frame buffers for use with the desktop images are proprietary and only compile with an appropriate license.

- build `de10_nano_audio_stream.qpf` with Intel Quartus Prime and generate the .rbf file
- compile 5.4.54-lts kernel from https://github.com/altera-opensource/linux-socfpga  with the supplied `linux.config` file from this repository
- compile bjarne steinsbo's `opencores_i2s.c` driver as a kernel module
- recompile bootloader to reflect the changes in pin muxing the I2C lines for the WM8731 codec
  - get u-boot from https://github.com/altera-opensource/u-boot-socfpga.git
  - use the `hps_isw_handoff` folder together with `bsp-create-settings` from the Intel FPGA embedded command shell
- compile the boot script `u-boot.script`with `mkimage`, which is part of u-boot
- compile the accompanying device tree `socfpga_cyclone5_de10_nano.dts` with the device tree compiler that ships with the Linux kernel
- download and compile Pure Data and the given externals in `software/pure-data-patches/externals/` 
- run the pure data patches via ssh and x-forwarding

## Hardware Setup

Since the DE10-Nano does not feature an audio codec, a WM8731 on the MIKROE-506 board was used. The connections were made as shown in the following figure.

![hardware-conections](https://github.com/chairaudio/SMC22-FPGA-accelerated-PD/blob/main/hardware-conections.png?raw=true)

### Desoldering the Onboard Oscillator

Since the Opencores I2S linux driver that we will be using is not capable of working with codec chips with external oscillators, we will have to desolder the oscillator on our MIKROE-506 board. The oscillator is marked “X2” on the board. Desolder it with heating up the two pads at once, either use two soldering irons or a heat gun. Also desolder the two little capacitors “C21” and “C22”. Desoldering the SMD oscillator can be a bit tricky. You can also try with a desoldering pump. 

When the oscillator is gone, solder a new connection with a wire between the pad of the “X2” footprint and the “AG11” pin as indicated by the dashed line in figure 2. The AG11 pin will supply the master clock from the FPGA to the codec chip and will replace the onboard oscillator.
