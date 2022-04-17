# [REF] http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
# [REF] http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html

# [Moses C. Nah] [2022.04.17]
#     - DO NOT FORGET TO REBUILD THE MESSAGES AFTER MODIFICATION!!
#     - $ cd /home/baxterplayground/ros_ws
#     - $ catkin_make

#std_msgs/Header

float64 stamp

# [Moses C. Nah] [2022.04.17]
#     - This is required to record the data just before and after the initiation.
#     - If not, then we need to manually find the start and end point, which is sort of a hassle.
#     - Will be great to know in case if there is a better alternative than introducing a new argument
bool on

# Right hand Data
float64[7]  q0_R
float64[7] dq0_R
float64[7]   q_R
float64[7]  dq_R
float64[7] tau_R

# Left hand Data
float64[7]  q0_L
float64[7] dq0_L
float64[7]   q_L
float64[7]  dq_L
float64[7] tau_L
