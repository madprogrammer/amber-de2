## Generated SDC file "de2_115.out.sdc"

## Copyright (C) 1991-2016 Altera Corporation. All rights reserved.
## Your use of Altera Corporation's design tools, logic functions 
## and other software and tools, and its AMPP partner logic 
## functions, and any output files from any of the foregoing 
## (including device programming or simulation files), and any 
## associated documentation or information are expressly subject 
## to the terms and conditions of the Altera Program License 
## Subscription Agreement, the Altera Quartus Prime License Agreement,
## the Altera MegaCore Function License Agreement, or other 
## applicable license agreement, including, without limitation, 
## that your use is for the sole purpose of programming logic 
## devices manufactured by Altera and sold by Altera or its 
## authorized distributors.  Please refer to the applicable 
## agreement for further details.


## VENDOR  "Altera"
## PROGRAM "Quartus Prime"
## VERSION "Version 16.0.0 Build 211 04/27/2016 SJ Lite Edition"

## DATE    "Sun Sep 18 13:03:43 2016"

##
## DEVICE  "EP4CE115F29C7"
##


#**************************************************************
# Time Information
#**************************************************************

set_time_format -unit ns -decimal_places 3



#**************************************************************
# Create Clock
#**************************************************************

create_clock -name {altera_reserved_tck} -period 100.000 -waveform { 0.000 50.000 } [get_ports {altera_reserved_tck}]
create_clock -name {brd_clk_p} -period 20.000 -waveform { 0.000 10.000 } [get_ports {brd_clk_p}]


#**************************************************************
# Create Generated Clock
#**************************************************************

create_generated_clock -name {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[0]} -source [get_pins {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|inclk[0]}] -duty_cycle 50/1 -multiply_by 2 -divide_by 5 -master_clock {brd_clk_p} [get_pins {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[0]}] 
create_generated_clock -name {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[1]} -source [get_pins {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|inclk[0]}] -duty_cycle 50/1 -multiply_by 4 -divide_by 5 -master_clock {brd_clk_p} [get_pins {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[1]}] 


#**************************************************************
# Set Clock Latency
#**************************************************************



#**************************************************************
# Set Clock Uncertainty
#**************************************************************

set_clock_uncertainty -rise_from [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[1]}] -rise_to [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[1]}]  0.020  
set_clock_uncertainty -rise_from [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[1]}] -fall_to [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[1]}]  0.020  
set_clock_uncertainty -rise_from [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[1]}] -rise_to [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[0]}]  0.020  
set_clock_uncertainty -rise_from [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[1]}] -fall_to [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[0]}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[1]}] -rise_to [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[1]}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[1]}] -fall_to [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[1]}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[1]}] -rise_to [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[0]}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[1]}] -fall_to [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[0]}]  0.020  
set_clock_uncertainty -rise_from [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[0]}] -rise_to [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[1]}]  0.020  
set_clock_uncertainty -rise_from [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[0]}] -fall_to [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[1]}]  0.020  
set_clock_uncertainty -rise_from [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[0]}] -rise_to [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[0]}]  0.020  
set_clock_uncertainty -rise_from [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[0]}] -fall_to [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[0]}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[0]}] -rise_to [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[1]}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[0]}] -fall_to [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[1]}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[0]}] -rise_to [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[0]}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[0]}] -fall_to [get_clocks {u_clocks_resets|pll_inst|altpll_component|auto_generated|pll1|clk[0]}]  0.020  
set_clock_uncertainty -rise_from [get_clocks {altera_reserved_tck}] -rise_to [get_clocks {altera_reserved_tck}]  0.020  
set_clock_uncertainty -rise_from [get_clocks {altera_reserved_tck}] -fall_to [get_clocks {altera_reserved_tck}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {altera_reserved_tck}] -rise_to [get_clocks {altera_reserved_tck}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {altera_reserved_tck}] -fall_to [get_clocks {altera_reserved_tck}]  0.020  


#**************************************************************
# Set Input Delay
#**************************************************************



#**************************************************************
# Set Output Delay
#**************************************************************



#**************************************************************
# Set Clock Groups
#**************************************************************

set_clock_groups -asynchronous -group [get_clocks {altera_reserved_tck}] 
set_clock_groups -asynchronous -group [get_clocks {altera_reserved_tck}] 


#**************************************************************
# Set False Path
#**************************************************************



#**************************************************************
# Set Multicycle Path
#**************************************************************



#**************************************************************
# Set Maximum Delay
#**************************************************************



#**************************************************************
# Set Minimum Delay
#**************************************************************



#**************************************************************
# Set Input Transition
#**************************************************************

