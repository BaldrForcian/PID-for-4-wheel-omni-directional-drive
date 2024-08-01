# PID-for-4-wheel-omni-directional-drive-RCJ-LWL
Simple PID program that straightens the robot while moving in a direction. The robot uses a BNO055 gyroscope to read the angle, then using PID parameters we calculate the needed PWM on the motors to drive straight or to straighten itself out.

The Kp, Ki, and Kd are arbitrary values different for each PID system and application, therefore the values in the program worked for me but might not work for you.
