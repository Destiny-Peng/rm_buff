# /// 包的类型
int32 pack_type

# /// 对时包的电控基准时刻（单位：ms）
int32 base_mcu_time

# /// 姿态包的电控时刻（未补全，最原始的情况）（单位：ms）
int32 pose_mcu_time

#对时包的视觉时间
int32 base_visual_time

#姿态包的视觉时间
int32 pose_visual_time

#视觉-电控 时间差
int32 time_gap

# /// 自瞄模式
int32 mode
# /// 当前子弹速度
float32 bullet_speed

# /// 敌方颜色
int32 enemy_color

# /// 要追踪的数字 默认为0，代表不进行追踪
int32 enemy_number

# /// 是否按下了鼠标右键(脉冲沿)
bool rightclick

# /// 接受到的鼠标状态
bool rightpress

# /// 已经开火的标志位，能量机关模式下使用
int32 fired

# /// 操作手请求发弹标志位，能量机关模式下使用
int32 request_fire

uint32 reload_delay_time

Pose pose