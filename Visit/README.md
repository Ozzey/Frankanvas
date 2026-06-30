1. First unzip the program folder to ~/Documents like ~/Documents/Sensor_Expreiment
2. Modify the CMakeLists.txt -> MSRM_UTILS_DIR to fit your username
3. Clone Cppserver in the ~/Documents/Sensor_Expreiment folder and setup based on https://github.com/chronoxor/CppServer
4. Modify local_ip, mios_ip in the main.cpp
5. Currently the input is expected as "timestamp sensor_force" (checkout the udp.h -> class SensorServer -> onReceived()), modify the input in case you have other udp package format or more inputs
6. The telemetry server is implemented to read the external force detected by the robot
7. The program supports text input such as "c 10" (move the robot to the table and reach until 10N from the telemetry/sensor), "record data.txt" etc
8. Current control frequency is 200Hz, if the motion is still not smooth, modify that to 500Hz or 1kHz if necessary 
9. Please read the program first before doing the experiment, and verify program function part by part (from package input, telemetry server, calibration mode, some functions are disabled for now which you can modify and enable like file_record, replay)