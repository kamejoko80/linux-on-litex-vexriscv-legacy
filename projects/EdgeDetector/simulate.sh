#!/bin/sh


# cleanup
rm -rf obj_dir
rm -f  EdgeDetector.vcd


# run Verilator to translate Verilog into C++, include C++ testbench
verilator -Wall --cc --trace EdgeDetector.v --exe edgedetector_tb.cpp
# build C++ project
make -j -C obj_dir/ -f VEdgeDetector.mk VEdgeDetector
# run executable simulation
obj_dir/VEdgeDetector


# view waveforms
gtkwave EdgeDetector.vcd &

