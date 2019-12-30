echo "GENERATE ACCEL SIMULATOR CORE..."
./soc_generator.py accel_simulator_config.py

echo "GENERATE ACCEL TEST CORE..."
./soc_generator.py accel_test_config.py

echo "BUILD SYSTEM ACCEL TEST PLATFORM..."
./sys_accel_test.py --build

echo "LOAD SYSTEM ACCEL SIMULATOR TEST BITSTREAM..."
./sys_accel_test.py --load
   
