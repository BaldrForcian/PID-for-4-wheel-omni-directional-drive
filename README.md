# PID-for-4-wheel-omni-directional-drive-RCJ-LWL
Simple PID program that straightens the robot while moving in a direction. The robot uses a BNO055 gyroscope to read the angle, then using PID parameters we calculate the needed PWM on the motors to drive straight or to straighten itself out. The program is very simple and can be further optimized it's just the solution we've used.

The Kp, Ki, and Kd are arbitrary values, different for each PID system and application, meaning that the values in the program worked for me but might not work for you.


Video of it working:
https://github.com/user-attachments/assets/4afd5bf9-6be4-4579-a6ba-fcdfc4d77e8b

