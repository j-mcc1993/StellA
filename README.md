# StellA
Wirelessly track the field of view of your telescope or camera in the Stellarium star map
application.

# How It Works
StellA uses an Arduino 101 with a built-in gyroscope, accelerometer and
Bluetooth LE modules. The Arduino uses the Madgewick algorithm to compute its
orientation and transmits the necessary values over Bluetooth for Python to
update the Stellarium API.
