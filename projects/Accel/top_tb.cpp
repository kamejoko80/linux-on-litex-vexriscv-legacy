#include "Vtop.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

// Number of simulation cycles
#define NUM_CYCLES  ((vluint64_t)200)
// Frequency (in MHz)
#define CLK_FREQ_MHZ (50)
// Half period (in ms)
#define HALF_PER_PS ((vluint64_t)1000/(2*CLK_FREQ_MHZ))

// Accel commandArgs
#define REG_RD (0x0B) // CMD = 1, Read Register
#define REG_WR (0x0A) // CMD = 2, Write Register
#define FIF_RD (0x0D) // CMD = 3, FIFO Read

// SDI pattern
#define SDI_PATT 0x0B

int main(int argc, char **argv, char **env) {
  
  // Half cycles
  vluint64_t hcycle;
  char toggle; 
  char sdi_pattern;
  
  Verilated::commandArgs(argc, argv);
  
  // init top verilog instance
  Vtop* top = new Vtop;
  
  // init trace dump
  Verilated::traceEverOn(true);
  VerilatedVcdC* tfp = new VerilatedVcdC;
  top->trace (tfp, 99);
  tfp->spTrace()->set_time_resolution("1 us");
  tfp->open ("top.vcd");
  
  // initialize simulation inputs
  top->spi0_sclk = 0;
  top->spi0_mosi = 0;
  top->spi0_csn = 1;
  toggle     = 1;
  
  sdi_pattern = SDI_PATT;
  
  // run simulation for NUM_CYCLES clock periods
  for (hcycle = 0; hcycle < (NUM_CYCLES * 2); ) 
  {
    
    top->spi0_csn = (hcycle < 2);
    
    if(hcycle % 2 == 0 )
    {
        top->spi0_sclk = !top->spi0_sclk;
    }
    else
    {
        toggle = toggle^1;
        
        if(toggle && !top->spi0_csn && hcycle > 3)
        {
            // Shift bit
            sdi_pattern <<= 1; 
        }  
    }
    
    // Generate sdi signal following data pattern
    if((toggle) && (sdi_pattern & 0x80))
    {
        top->spi0_mosi = 1;
    }else
    {
        top->spi0_mosi = 0;
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

