echo "GENERATE ACCEL SIMULATOR RELEASE CORE..."
./soc_generator.py accel_simulator_release_config.py

echo "BUILD SYSTEM ACCEL SIMULATOR RELEASE PLATFORM..."
./sys_accel_simulator_release.py --build

echo "LOAD SYSTEM ACCEL SIMULATOR RELEASE BITSTREAM..."
./sys_accel_simulator_release.py --load
