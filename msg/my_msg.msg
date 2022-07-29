# [REF] http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
# [REF] http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html

# [Moses C. Nah] [2022.04.17]
#     - DO NOT FORGET TO REBUILD THE MESSAGES AFTER MODIFICATION!!
#     - $ cd /home/baxterplayground/ros_ws
#     - $ catkin_make

#std_msgs/Header

float64 stamp
string name
string which_arm

# [Moses C. Nah] [2022.04.17]
#     - This is required to record the data just before and after the initiation.
#     - If not, then we need to manually find the start and end point, which is sort of a hassle.
#     - Will be great to know in case if there is a better alternative than introducing a new argument
bool on

# Details 
float64[]  x0
float64[] dx0
float64[]   x
float64[]  dx
float64[]   K
float64[]   B
float64[] tau
