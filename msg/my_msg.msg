# [REF] http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
# [REF] http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html

#std_msgs/Header

float64 stamp
bool on

float64[7] q0_R    # Right hand perspective/
float64[7] q_R
float64[7] dq_R
float64[7] tau_R

float64[7] q0_L    # Left hand perspective/
float64[7] q_L
float64[7] dq_L
float64[7] tau_L
