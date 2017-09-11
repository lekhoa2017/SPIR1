Header header

# pile state in laser frame
geometry_msgs/Point point

# desired pile heading w.r.t to robot in inertial frame aligned with magnetometer
# used when losing visibility of the desired pile 
float64 yaw

#projected point
geometry_msgs/Point projection

# define pile detection status:
# 1 : desired pile detected
# 2 : desired pile not detected, other pile detected
# 3 : no pile is detected
# 4 : target pile is not signed and no pile is detected
# 5 : target pile is not signed, piles are detected
int32 status



