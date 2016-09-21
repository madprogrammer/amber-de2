#include <verilated.h>
#if VM_TRACE
#include "verilated_vcd_c.h"
#endif

#include "Vtb.h"

Vtb *tb;

vluint64_t main_time = 0;

double sc_time_stamp () {
    return main_time;
}

int main(int argc, char **argv, char **env) {
    if (0 && argc && argv && env) {}
    tb = new Vtb;

    Verilated::commandArgs(argc, argv);
    Verilated::debug(0);

    tb->sysrst = 0;
    tb->clk_80mhz = 0;

#if VM_TRACE
    Verilated::traceEverOn(true);
    VerilatedVcdC* tfp = new VerilatedVcdC;
    tb->trace(tfp, 99);
    tfp->spTrace()->set_time_resolution("1 ns");
    tfp->open("out.vcd");
#endif

    while (!Verilated::gotFinish())
	{
		tb->clk_80mhz = !tb->clk_80mhz;
		if (((main_time - 4) % 9) == 0) {
			tb->uart_clk = !tb->uart_clk;
		}
		if (main_time > 500) {
	    	tb->sysrst = 1;
		}
		tb->eval();
#if VM_TRACE
		tfp->dump(main_time);
#endif
		main_time++;
    }

    tb->final();
#if VM_TRACE
	if (tfp)
	    tfp->close();
#endif
    exit(0);
}
