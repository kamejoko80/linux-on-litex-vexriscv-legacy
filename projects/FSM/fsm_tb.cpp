#include "Vfsm.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

// Number of simulation cycles
#define NUM_CYCLES  ((vluint64_t)200)
// Frequency (in MHz)
#define CLK_FREQ_MHZ (50)
// Half period (in ms)
#define HALF_PER_PS ((vluint64_t)1000/(2*CLK_FREQ_MHZ))

int main(int argc, char **argv, char **env) {
  
  // Half cycles
  vluint64_t hcycle;
  
  Verilated::commandArgs(argc, argv);
  
  // init top verilog instance
  Vfsm* top = new Vfsm;
  
  // init trace dump
  Verilated::traceEverOn(true);
  VerilatedVcdC* tfp = new VerilatedVcdC;
  top->trace (tfp, 99);
  tfp->spTrace()->set_time_resolution("1 us");
  tfp->open ("fsm.vcd");
  
  // initialize simulation inputs
  top->sys_clk = 0;
  top->sys_rst = 0;
  

  
  // run simulation for NUM_CYCLES clock periods
  for (hcycle = 0; hcycle < (NUM_CYCLES * 2); ) 
  {
    
    top->sys_clk = !top->sys_clk;
 
    if(hcycle > 5)
    {
        top->en = 1;
    }
 
    // Evaluate verilated model
    top->eval ();
    
    // Dump signals into VCD file
    if (tfp) tfp->dump (hcycle * HALF_PER_PS);
    
    // Next half cycle
    hcycle++;
        
    if (Verilated::gotFinish())  exit(0);
  }
  
  tfp->close();
  
  exit(0);
}

