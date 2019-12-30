#!/bin/sh


# cleanup
rm -rf obj_dir
rm -f  fsm.vcd


# run Verilator to translate Verilog into C++, include C++ testbench
verilator -Wall --cc --trace fsm.v --exe fsm_tb.cpp
# build C++ project
make -j -C obj_dir/ -f Vfsm.mk Vfsm
# run executable simulation
obj_dir/Vfsm


# view waveforms
gtkwave fsm.vcd &

