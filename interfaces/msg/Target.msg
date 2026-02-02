#/// x轴坐标，单位m
float64 x

#/// y轴坐标，单位m
float64 y

#/// z轴坐标，单位m
float64 z

#/// 水平投影距离,单位m
float64 xy_plane_distance

# /// 球坐标R--3d
float64 xyz_distance
#    /// 该坐标值在云台系中对应的yaw和考虑弹道因素的pitch,单位度
float64 yaw

float64 pitch

#@brief 速度相关参数

float64 xy_v
float64 xyz_v
float64 yaw_v
float64 pitch_v

#/// 时钟时间,单位s
float64 clock

Pose pose
