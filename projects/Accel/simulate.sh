#!/bin/sh


# cleanup
rm -rf obj_dir
rm -f  top.vcd


# run Verilator to translate Verilog into C++, include C++ testbench
verilator -Wall --cc --trace top.v --exe top_tb.cpp
# build C++ project
make -j -C obj_dir/ -f Vtop.mk Vtop
# run executable simulation
obj_dir/Vtop


# view waveforms
gtkwave top.vcd top.sav &

