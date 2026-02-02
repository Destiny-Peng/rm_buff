# /// 工作模式，在发送线程的具体函数中赋值即可
int32 mode

# /// 设定值时刻，在发送时刻赋值即可
int32 clock

# /// 预测pitch，单位度
float32 pred_pitch

# /// 预测yaw，单位度
float32 pred_yaw

# /// yaw和pitch的精度用于使得电控判断当前云台跟随是否达到能够击中的要求
# /// 是自动发弹的必要条件
float32 yaw_resolution # yaw精度,半角宽，单位度,
float32 pitch_resolution#pitch精度,半角宽，单位度

# /// 电控端需要平滑处理云台转动
float32 yaw_palstance  # 预测yaw轴转速,单位度/s
float32 pitch_palstance#预测pitch轴转速,单位度/s

# /// 运动预测是否给出目标,值为false时云台不受控制
bool target_found

# /// 是否有把握开火，值
bool fire

# /// 当前能量机关装甲板角位置,适用于小能量机关和静止能量机关
# float32 SMALLRUNEANGLE=0

# /// 爆发式自瞄,默认没被触发
bool burst_shoot

# /// 退出能量机关模式的标志位,翻转生效
# bool EXITRUNE =0
# /// 能量机关的发弹标志位
bool rune_fire

# /// 能量机关的连发标志位
bool rune_burst_shoot

# /// 视觉模式工作是否正常标志位
bool isproperly 

# /// 新增，开火电控时间
int64 wannfiretime