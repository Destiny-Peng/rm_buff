#ifndef VALIDATOR_HPP
#define VALIDATOR_HPP
#include <vector>
namespace rune::optimization
{
    // 默认模型验证器（总是返回true）
    struct AlwaysValidValidator
    {
        template <typename Model>
        bool isValid(const Model &) const { return true; }
    };

    /**
     * @brief 平面模型验证策略
     */
    struct PlaneValidator
    {
        bool isValid(const PlaneFittingProblem::Model &model) const
        {
            return model.first.norm() > 1e-6;
        }
    };

    /**
     * @brief 约束平面验证策略
     */
    struct ConstrainedPlaneValidator
    {
        bool isValid(const ZConstrainedPlaneFitting::Model &model) const
        {
            return (model[0] != 0 || model[1] != 0) && // 法向量非零
                   (std::abs(model[2]) < 1e-6);        // 确保z分量为零
        }
    };
}
#endif