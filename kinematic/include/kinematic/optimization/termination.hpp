#ifndef TERMINATION_HPP
#define TERMINATION_HPP

#include <vector>
#include <limits> // Required for std::numeric_limits

namespace rune::optimization
{
    // 默认终止条件（固定迭代次数）
    class FixedIterationTermination
    {
    public:
        FixedIterationTermination(int max_iters) : max_iters_(max_iters) {}

        bool operator()(int curr_iter, int /*best_inliers*/) const
        {
            return curr_iter >= max_iters_;
        }

    private:
        int max_iters_;
    };

    // 动态终止条件
    class AdaptiveTermination
    {
    public:
        bool operator()(int curr_iter, int best_inliers) const
        {
            return curr_iter >= (best_inliers > 300 ? 20 : 200);
        }
    };

    /**
     * @brief 终止条件策略
     */
    class TerminationCondition
    {
        int max_iter_;
        int min_outliers_;
        int total_points_;

    public:
        TerminationCondition(int max_iter, int min_allowed_outliers, int total_points)
            : max_iter_(max_iter),
              min_outliers_(min_allowed_outliers),
              total_points_(total_points) {}

        bool operator()(int iteration, int outlier_count) const
        {
            return iteration >= max_iter_ || min_outliers_ >= outlier_count;
        }
    };

    // 基于最大迭代次数的终止条件 (已存在，可复用或按需修改)
    struct MaxIterationsTermination
    {
        int max_iterations_;
        explicit MaxIterationsTermination(int max_iter) : max_iterations_(max_iter) {}
        bool operator()(int current_iteration, double /* current_best_metric */) const
        {
            return current_iteration >= max_iterations_;
        }
    };

    // 基于最大迭代次数或误差阈值的终止条件
    struct MaxIterationsOrErrorThresholdTermination
    {
        int max_iterations_;
        double error_threshold_; // Terminate if best_error_found < error_threshold_

        MaxIterationsOrErrorThresholdTermination(int max_iter, double err_thresh)
            : max_iterations_(max_iter), error_threshold_(err_thresh) {}

        /**
         * @brief 判断是否应终止 RANSAC 迭代
         * @param current_iteration 当前迭代次数
         * @param current_best_error 当前找到的最佳模型的误差
         * @return 如果达到终止条件则返回 true，否则返回 false
         */
        bool operator()(int current_iteration, double current_best_error) const
        {
            if (current_iteration >= max_iterations_)
            {
                return true;
            }
            if (current_best_error < error_threshold_)
            {
                return true;
            }
            return false;
        }
    };
}

#endif