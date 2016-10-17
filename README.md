# Drone-KK2-Arduino
Arduino used to control the throttle and pitch PWM of the KK2 flight controller for my 450mm Drone (quadcopter). 

There are currently two modes: 1. manual mode and, 2. altitude hold with obstacle avoidance. Altitude determined using an ultrasonic sensor and altitude hold via custom PID library. Ultrasonic sensor added to the front of the quad to detect objects and reverses until the threshold is clear. 

Work in progress. I hope this will be useful to others who are trying to do the same. Please give credit and let me know if this helps. 

Disclamer: I am not responsible for any damage caused by the use of this code. Use at your own risk. This is for educational use only. Tie down the quad and fly safely. 
