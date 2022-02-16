#************************************************************
# THIS IS A WIZARD-GENERATED FILE.                           
#
# Version 13.1.4 Build 182 03/12/2014 SJ Full Version
#
#************************************************************

# Copyright (C) 1991-2014 Altera Corporation
# Your use of Altera Corporation's design tools, logic functions 
# and other software and tools, and its AMPP partner logic 
# functions, and any output files from any of the foregoing 
# (including device programming or simulation files), and any 
# associated documentation or information are expressly subject 
# to the terms and conditions of the Altera Program License 
# Subscription Agreement, Altera MegaCore Function License 
# Agreement, or other applicable license agreement, including, 
# without limitation, that your use is for the sole purpose of 
# programming logic devices manufactured by Altera and sold by 
# Altera or its authorized distributors.  Please refer to the 
# applicable agreement for further details.


set sdram_clk "${topmodule}pll|altpll_component|auto_generated|pll1|clk[1]"
set mem_clk   "${topmodule}pll|altpll_component|auto_generated|pll1|clk[0]"
set sys_clk   "${topmodule}pll|altpll_component|auto_generated|pll1|clk[0]"

# Automatically calculate clock uncertainty to jitter and other effects.
derive_clock_uncertainty

set_clock_groups -asynchronous -group [get_clocks $hostclk] -group [get_clocks $sys_clk]
set_clock_groups -asynchronous -group [get_clocks spiclk] -group [get_clocks $sys_clk]

# SDRAM delays
set_input_delay -clock [get_clocks $sdram_clk] -reference_pin [get_ports ${RAM_CLK}] -max 6.6 [get_ports ${RAM_IN}]
set_input_delay -clock [get_clocks $sdram_clk] -reference_pin [get_ports ${RAM_CLK}] -min 3.5 [get_ports ${RAM_IN}]

set_output_delay -clock [get_clocks $sdram_clk] -reference_pin [get_ports ${RAM_CLK}] -max 1.5 [get_ports ${RAM_OUT}]
set_output_delay -clock [get_clocks $sdram_clk] -reference_pin [get_ports ${RAM_CLK}] -min -0.8 [get_ports ${RAM_OUT}]

#SDRAM_CLK to internal memory clock
set_multicycle_path -from [get_clocks $sdram_clk] -to [get_clocks $mem_clk] -setup -end 2

# Some relaxed constrain to the VGA pins. The signals should arrive together, the delay is not really important.
#set_output_delay -clock [get_clocks $sys_clk] -max 0 [get_ports {VGA_*}]
#set_output_delay -clock [get_clocks $sys_clk] -min -5 [get_ports {VGA_*}]
#set_multicycle_path -to [get_ports {VGA_*}] -setup 5
#set_multicycle_path -to [get_ports {VGA_*}] -hold 4

#set_multicycle_path -from {video:video|video_mixer:video_mixer|scandoubler:scandoubler|Hq2x:Hq2x|*} -setup 6
#set_multicycle_path -from {video:video|video_mixer:video_mixer|scandoubler:scandoubler|Hq2x:Hq2x|*} -hold 5

# Effective clock is only half of the system clock, so allow 2 clock cycles for the paths in the T80 cpu
set_multicycle_path -from ${topmodule}cpu|u0|* -setup 2
set_multicycle_path -from ${topmodule}cpu|u0|* -hold 1

set_multicycle_path -from ${topmodule}gs|cpu|u0|* -setup 2
set_multicycle_path -from ${topmodule}gs|cpu|u0|* -hold 1

# The CE is only active in every 2 clocks, so allow 2 clock cycles
set_multicycle_path -to ${topmodule}tape|tape|addr[*] -setup 2
set_multicycle_path -to ${topmodule}tape|tape|addr[*] -hold 1
set_multicycle_path -to ${topmodule}tape|tape|read_cnt[*] -setup 2
set_multicycle_path -to ${topmodule}tape|tape|read_cnt[*] -hold 1
set_multicycle_path -to ${topmodule}tape|tape|blocksz[*] -setup 2
set_multicycle_path -to ${topmodule}tape|tape|blocksz[*] -hold 1
set_multicycle_path -to ${topmodule}tape|tape|timeout[*] -setup 2
set_multicycle_path -to ${topmodule}tape|tape|timeout[*] -hold 1
set_multicycle_path -to ${topmodule}tape|tape|pilot[*] -setup 2
set_multicycle_path -to ${topmodule}tape|tape|pilot[*] -hold 1
set_multicycle_path -to ${topmodule}tape|tape|tick[*] -setup 2
set_multicycle_path -to ${topmodule}tape|tape|tick[*] -hold 1
set_multicycle_path -to ${topmodule}tape|tape|blk_list[*] -setup 2
set_multicycle_path -to ${topmodule}tape|tape|blk_list[*] -hold 1
set_multicycle_path -to ${topmodule}tape|tape|bitcnt[*] -setup 2
set_multicycle_path -to ${topmodule}tape|tape|bitcnt[*] -hold 1
set_multicycle_path -from ${topmodule}tape|act_cnt[*] -setup 2
set_multicycle_path -from ${topmodule}tape|act_cnt[*] -hold 1


# The effective clock fo the AY chips are 112/1.75=64 cycles, so allow at least 2 cycles for the paths
set_multicycle_path -to ${topmodule}turbosound|ym2149_0|* -setup 2
set_multicycle_path -to ${topmodule}turbosound|ym2149_0|* -hold 1
set_multicycle_path -to ${topmodule}turbosound|ym2149_1|* -setup 2
set_multicycle_path -to ${topmodule}turbosound|ym2149_1|* -hold 1

set_multicycle_path -from ${topmodule}fdd|sbuf|* -setup 2
set_multicycle_path -from ${topmodule}fdd|sbuf|* -hold 1

set_multicycle_path -to ${topmodule}fdd|state[*] -setup 2
set_multicycle_path -to ${topmodule}fdd|state[*] -hold 1
set_multicycle_path -to ${topmodule}fdd|wait_time[*] -setup 2
set_multicycle_path -to ${topmodule}fdd|wait_time[*] -hold 1

set_multicycle_path -from ${topmodule}u765|sbuf|* -setup 2
set_multicycle_path -from ${topmodule}u765|sbuf|* -hold 1
set_multicycle_path -from ${topmodule}u765|image_track_offsets_rtl_0|* -setup 2
set_multicycle_path -from ${topmodule}u765|image_track_offsets_rtl_0|* -hold 1
set_multicycle_path -to ${topmodule}u765|i_* -setup 2
set_multicycle_path -to ${topmodule}u765|i_* -hold 1
set_multicycle_path -to ${topmodule}u765|i_*[*] -setup 2
set_multicycle_path -to ${topmodule}u765|i_*[*] -hold 1
set_multicycle_path -to ${topmodule}u765|pcn[*] -setup 2
set_multicycle_path -to ${topmodule}u765|pcn[*] -hold 1
set_multicycle_path -to ${topmodule}u765|ncn[*] -setup 2
set_multicycle_path -to ${topmodule}u765|ncn[*] -hold 1
set_multicycle_path -to ${topmodule}u765|state[*] -setup 2
set_multicycle_path -to ${topmodule}u765|state[*] -hold 1
set_multicycle_path -to ${topmodule}u765|status[*] -setup 2
set_multicycle_path -to ${topmodule}u765|status[*] -hold 1
set_multicycle_path -to ${topmodule}u765|i_rpm_time[*][*][*] -setup 8
set_multicycle_path -to ${topmodule}u765|i_rpm_time[*][*][*] -hold 7

# False paths

set_false_path -to ${FALSE_OUT}
set_false_path -from ${FALSE_IN}
