# Chainable Controllers 

ROS 2 chainable controllers interacting only through reference interfaces and one ROS topic.  
- rigid_pose_broadcaster: exports pose.x/y/theta  
- broyden_controller: exports vel.v/w and subscribes to `/kalman_estimate  
- kalman_controller: reads all five references, fuses them, and publishes /kalman_estimate

Activation order must be: kalman_controller then broyden_controller & rigid_pose_broadcaster(downstream consumer (Kalman) loads first and the upstream providers connect after it).
