all: clean
	verilator -CFLAGS -O3 -DICARUS=1 -Wall -I../../vlog/tb -I../../vlog/lib -I../../vlog/system -I../../vlog/amber23 -I../ -I../../vlog/sdram/sdr_ctrl/trunk/rtl/core -I../../vlog/sdram/sdr_ctrl/trunk/rtl/wb2sdrc -I../../vlog/sdram/sdr_ctrl/trunk/rtl/top -I../../vlog/sdram/sdr_ctrl/trunk/rtl/lib --cc tb.v --top-module tb -Wno-fatal --exe sim_main.cpp && make -C obj_dir -f Vtb.mk

clean:
	rm -rf obj_dir

