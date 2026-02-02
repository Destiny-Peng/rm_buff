#ifndef RUNE_PARAMETERS_HPP
#define RUNE_PARAMETERS_HPP
#include <cmath>

// 本hpp为全局参数
namespace rune
{
    // 虽然但是，我们自己的能量机关并不标准，730mm长
    constexpr double STANDARD_RUNE_RADIUS = 0.7;

    /// 小能量机关转速,单位 rad/s
    constexpr double CONST_PALSTANCE = M_PI / 3;

    /// 能量机关标准高度,单位m
    constexpr static double STANDARD_HEIGHT = 1.075;

    //...待补充
}

#endif