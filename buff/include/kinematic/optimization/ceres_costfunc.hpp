#ifndef CERES_COSTFUNC_HPP
#define CERES_ COSTFUNC_HPP
#include <Eigen/Dense>

namespace rune::optimization
{
    struct ConstrainedCircle3DFittingCost
    {
        ConstrainedCircle3DFittingCost(const Eigen::Vector3d &pt) : observed_pt(pt) {}

        template <typename T>
        bool operator()(const T *const theta,
                        const T *const d,
                        const T *const center,
                        const T *const radius,
                        T *residual) const
        {
            // 计算平面法向量
            T nx = cos(*theta);
            T ny = sin(*theta);

            // 验证圆心在平面上
            T plane_eq = nx * center[0] + ny * center[1] + *d;

            // 计算投影距离
            T dx = T(observed_pt.x()) - center[0];
            T dy = T(observed_pt.y()) - center[1];
            T dz = T(observed_pt.z()) - center[2];

            T dot = dx * nx + dy * ny + dz * T(0);
            T proj_x = dx - dot * nx;
            T proj_y = dy - dot * ny;

            T distance = sqrt(proj_x * proj_x + proj_y * proj_y);
            residual[0] = distance - *radius;
            residual[1] = plane_eq * T(10); // 强约束平面方程

            return true;
        }

    private:
        Eigen::Vector3d observed_pt;
    };

    /**
     * @brief 角度正则化约束项
     * @note 将角度偏差量作为正则化项，控制优化后的角度不要偏离初始估计太远
     */
    struct AngleRegularizationCost
    {
        explicit AngleRegularizationCost(double reference_theta)
            : ref_theta(reference_theta) {}

        template <typename T>
        bool operator()(const T *const theta, T *residual) const
        {
            // 计算角度偏差量（考虑周期特性）
            T delta = *theta - T(ref_theta);
            T wrapped_delta = ceres::abs(delta); // [0, π]
            if (wrapped_delta > T(M_PI))
            {
                wrapped_delta = T(2 * M_PI) - wrapped_delta;
            }
            residual[0] = wrapped_delta; // 直接使用角度偏差量
            return true;
        }

    private:
        double ref_theta; // 初始角度参考值
    };

    // 二维圆拟合损失函数（关键修改点）
    struct Circle2DFittingCost
    {
        explicit Circle2DFittingCost(const Eigen::Vector2d &pt) : observed(pt) {}

        template <typename T>
        bool operator()(const T *const center, const T *radius, T *residual) const
        {
            T dx = observed.x() - center[0];
            T dy = observed.y() - center[1];
            // 使用平方距离比较避免开方计算
            residual[0] = dx * dx + dy * dy - (*radius) * (*radius);
            return true;
        }

    private:
        Eigen::Vector2d observed;
    };
} // namespace rune::optimization

#endif