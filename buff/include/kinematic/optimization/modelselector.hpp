#ifndef MODESELECTOR_HPP
#define MODESELECTOR_HPP

#include <vector>

namespace rune::optimization
{
    // 默认模型选择器（选择内点最多的模型）
    struct BestModelSelector
    {
        template <typename Model>
        void update(int inliers, const Model &model, int &best_inliers, Model &best_model)
        {
            if (inliers > best_inliers)
            {
                best_inliers = inliers;
                best_model = model;
            }
        }
    };

    // 动态模型选择器
    class DynamicModelSelector
    {
    public:
        template <typename ModelParams, typename DataPoint>
        void update(int inliers, const ModelParams &model,
                    int &best_inliers, ModelParams &best_model,
                    std::vector<DataPoint> &inliers_set,
                    std::vector<DataPoint> &outliers) const
        {
            if (inliers > best_inliers)
            {
                best_inliers = inliers;
                best_model = model;

                // 动态调整内点集
                if (inliers_set.size() > 400)
                {
                    std::shuffle(inliers_set.begin(), inliers_set.end() - 100,
                                 std::mt19937(std::random_device{}()));
                }
            }
        }
    };
}
#endif