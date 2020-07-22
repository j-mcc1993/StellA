# StellA
Wirelessly track the field of view of your telescope or camera in the Stellarium star map
application.

# How It Works
StellA connects an Arduino 101, Nano BLE or Nano BLE Sense board to the open-source star map application Stellarium. On the Arduino side, the program uses the Madgwick fusion algorithm to compute the board's orientation from its gyroscope, accelerometer and (possibly) magnetometer sensors. The altitude and azimuth angles (pitch and yaw) are sent to StellA.py over a Bluetooth LE Service, and the Python code calls the Stellarium HTTP to track the view in real time. StellA.py offers the option to request the temperature, humidity and dewpoint from the Arduino if using a Nano board, along with the option to calibrate alt/az values for improved accuracy.
