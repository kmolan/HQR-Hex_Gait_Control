# ReadMe

Code written for experiments under obstacle aided-locomotion and gait control, at kod\*lab, a subsidiary of the GRASP Robotics Lab at Upenn. I was supervised directly under Dr. Feifei Qian (https://kodlab.seas.upenn.edu/group/feifei/), and overall worked under Prof. Daniel E. Koditschek (https://www.seas.upenn.edu/~kod/). 

The world information is estimated using the topics published by the VICON Motion Capture system (https://github.com/KumarRobotics/motion_capture_system), and the robot is controlled using the SDK library written by Ghost Robotics (https://ghostrobotics.gitlab.io/SDK).

1. Collects pose and world information about the robot body using vicon system (done) 
2. Collects internal state information through raspberry pi (master done, slave unwritten)
3. Controller node activates suitable legs (only skeleton)
