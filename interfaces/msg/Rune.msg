#armor vector
VPoint[] vvpoint

#R标坐标
geometry_msgs/Point r_center

#armor对应的pose
Pose pose

#/// 识别是否正常的标志位
bool flag_detect_success

#/// 是否正常激活至少一个扇叶
bool flag_all_activate

float64 energy_yaw

#时间戳
float64 visual_time
builtin_interfaces/Time stamp
