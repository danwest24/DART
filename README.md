# DART
This set of applications work together to form the control system and UI for DART, the Device for Autonomous Removal of Trash. This trash - cleaning robot utilizes a continously rotating brush wheel and conveyor belt which scoop up garbage and carry it to an onboard trash can.

Applications - 

Android UI application: DART control - this application allows the user to select an operational polygon using the Google Maps Api (a "geofence"). This information is then sent to DART's onboard Raspberry Pi via bluetooth, which begins the operation of the device. 

Raspberry Pi high level software:
- bluetooth serial transmission
- Arduino serial transmission
- GPS polling
- transformation of GPS points into local reference frame. 
- map generation from input polygon
- path generation using cubic splines
- Positioning with dead reckoning and Kalman filter with IMU (TODO: GPS kalman filter)
- pursuit algorithm to generate headings based on path
- PID to follow path
- Ultrasonic rangefinder operation for 4 sensors, with raycasting into body and local reference frames
- obstacle avoidance routine
- motor transformation functions (turning speed -> output PWM for left and right motor channels)

Arduino low-level software:
- sets motors
- reads IMU, Ultrasonic rangefinders, force-sensitive resistor, and current sensors
- handles serial transmission to Pi
