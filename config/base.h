/*
 * @file base.h
 * @brief 宏定义头文件
 * @details 通过宏定义控制项目代码块的使用和超参数的配置
 * @license Copyright© 2022 HITwh HERO-RoboMaster Group
 */

#ifndef BASE_H
#define BASE_H

//----------------是否开启一些debug语句----------
// 上传工程到仓库gitee前一定要注释！！！！
#define USER_DEBUG
#define USER_DEBUG_IMG_PROCESS
// #define USER_TUNE_PARAM

//-------------------根据车辆ID和是否使用网路计算，不要随便改！---------------
#if defined(PREFER_USE_NETWORK) && defined(NETWORK_SUPPORT) && defined(NETWORK_DEPENDECNCE_SUPPORT)
#define _USE_NETWORK
#endif

//-------------------性能分析---------------------
// #define PERFORMANCE_ANALYSIS
//
//--------------------终端日志调试-----------------
// 定义输出等级
// LOG_LEVEL_MSG    // 所有都输出
// LOG_LEVEL_DEBUG
// LOG_LEVEL_WARN
// LOG_LEVEL_ERROR
// LOG_LEVEL_NONE   // 全部不输出
#define CONSOLE_LOG_LEVEL LOG_LEVEL_MSG

//------------------------线程调试-----------------------------------------

//-------------------------自瞄调试---------------------------
// #define ARMOR_OLD     // 采用旧的图像预处理方法：灰度图与红蓝通道相减图二值化后去与操作
#define ARMOR_NEW // 采用新的图像预处理方法：取灰度图和颜色通道图亮度集中的区域
// #define DISTORTION_CORRECT         // 是否使用畸变矫正
// #define COMPILE_WITH_CUDA          // armordetectEor中图像预处理使用CUDA加速
// #define BURST_AIM                  // 判断是否使用爆发时自瞄

//-------------------------打击哨兵调试---------------------------
// #define SENTINEL_OLD      // 取亮度集中的地方
#define SENTINEL_NEW // 灰度图与红蓝通道相减图二值化后去与操作

//-----------------------能量机关调试-----------------------------
// #define RUNE_SUBTRACT                 // 红蓝通道相减二值化                                                                                                               \
                 // #define RUNE_ARMOR_METHOD             // 自瞄预处理方式                                                                                                                     \
                 // #define RUNE_RANGE_THRESH             // 区域二值化
#define RUNE_R_JUDGEDIR                      // 使用R标判断旋转方向
#define RUNE_SOON_FAN_SWITCHED_AND_CALIBRATE // 允许要换扇叶拟合
#define SOLVEPNP_R \
//---------------------------------------------------------------------------

// ------------------------相机调试---------------------------------
#define AUTO_EXP // 开启自动曝光
// #define FIX_EXP// 开启手动曝光，否则默认自动曝光

//-----------------------------------------------------------

//----------------------------------------------------------------------

// 打击高速陀螺或者远处陀螺模式选择
// #define GYRO_FIX_CAR_CENTER // 以固定敌方车辆中心的方式击打
// #define GYRO_FIND_MIDDLE // 使用循环队列存储最近装甲板坐标，选择中位数进行击打

// #define GYRO_MODEL // 使用建立的陀螺模型进行击打 2022赛季
#define GYRO_MODEL_THREE // 2023赛季
// #define GYRO_FIX_CAR_CENTER_AUTO_SHOOT // 以固定敌方车辆中心的方式自动发弹击打

// 自瞄图像预处理
// #define USE_OPENCV_FUNC
#define USE_OWN

#endif // BASE_H
