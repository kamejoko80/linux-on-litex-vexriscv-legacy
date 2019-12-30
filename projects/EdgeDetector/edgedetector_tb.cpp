#include "VEdgeDetector.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

// Number of simulation cycles
#define NUM_CYCLES  ((vluint64_t)1000)
// Frequency (in MHz)
#define CLK_FREQ_MHZ (50)
// Half period (in ms)
#define HALF_PER_PS ((vluint64_t)1000/(2*CLK_FREQ_MHZ))

// Accel commandArgs
#define REG_RD (0xA50B) 
#define REG_WR (0x190A)
#define FIF_RD (0xFA0D)

// SDI pattern
#define SDI_PATT REG_RD

int main(int argc, char **argv, char **env) {
  
  // Half cycles
  vluint64_t hcycle;
  char toggle; 
  vluint16_t sdi_pattern;

  
  
  Verilated::commandArgs(argc, argv);
  
  // init top verilog instance
  VEdgeDetector* top = new VEdgeDetector;
  
  // init trace dump
  Verilated::traceEverOn(true);
  VerilatedVcdC* tfp = new VerilatedVcdC;
  top->trace (tfp, 99);
  tfp->spTrace()->set_time_resolution("1 us");
  tfp->open ("EdgeDetector.vcd");
  
  // initialize simulation inputs
  top->sys_clk = 0;
  top->sys_rst = 0;
  toggle       = 1;
  sdi_pattern  = SDI_PATT;

  char count = 0; 
  char count2 = 0;    
  
  top->sck = 1; 
  
   
  // run simulation for NUM_CYCLES clock periods
  for (hcycle = 0; hcycle < (NUM_CYCLES * 2); ) 
  {
    
    if(hcycle == 15)
    {
        top->start = 1;
    }
    
    if (hcycle > 5)
    {
        if (count2 < 10)
        {
            count2++;
        }
        else 
        {
            top->sck = top->sck^1;
            count2 = 0;
        }   
    }

    if(hcycle % 2 == 0 )
    {
        top->sys_clk = !top->sys_clk;
    }
    else
    {
        if (count < 10)
        {
            count++;
        }
        else
        {
            count = 0;

            toggle = toggle^1;
        
            if(toggle)
            {
                // Shift bit
                sdi_pattern <<= 1; 
            }      

            // Generate sdi signal following data pattern
            if((toggle) && (sdi_pattern & 0x8000))
            {
                top->si = 1;
            }else
            {
                top->si = 0;
            }
        }
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

