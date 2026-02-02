#ifndef RANSAC_HPP
#define RANSAC_HPP

#include <vector>
#include <random>
#include <limits>
#include <algorithm>
#include <iostream>

#include "problem.hpp"
#include "sampler.hpp"
#include "validator.hpp"
#include "termination.hpp"
#include "modelselector.hpp"
#include "ceres_costfunc.hpp"

namespace rune::optimization
{

    // *****************************
    //         核心模板类
    // *****************************

    /**
     * @brief 通用RANSAC算法实现
     * @tparam Problem 优化问题类型（需实现fitModel和computeError）
     * @tparam Sampler 采样策略类型
     * @tparam Validator 模型验证策略类型
     * @tparam Termination 终止条件策略类型
     * @tparam ModelSelector 模型选择策略类型
     */

    template <
        typename Problem,
        typename Sampler,
        typename TerminationCondition>
    class ErrorDrivenRansac
    {
    public:
        using DataPoint = typename Problem::DataPoint;
        using Model = typename Problem::Model;

        /**
         * @brief 运行基于误差的RANSAC算法
         * @param all_data 包含所有数据点的向量
         * @param problem Problem 策略对象，定义了模型拟合和误差计算
         * @param sampler Sampler 策略对象，用于从数据中采样
         * @param termination_condition TerminationCondition 策略对象，定义了迭代终止条件
         * @param out_best_consensus_set (可选输出) 如果非空指针，将填充最佳共识集
         * @return 拟合得到的最佳模型。如果找不到合适的模型，可能返回一个默认构造的模型。
         */
        Model run(
            const std::vector<DataPoint> &all_data,
            const Problem &problem,
            const Sampler &sampler,
            const TerminationCondition &termination_condition,
            std::vector<DataPoint> *out_best_consensus_set = nullptr)
        {
            if (all_data.size() < Problem::min_samples_required())
            {
                // 数据点不足以拟合初始模型
                if (out_best_consensus_set)
                    out_best_consensus_set->clear();
                return Model{}; // 返回默认构造的模型表示失败
            }

            Model best_model_so_far{};
            std::vector<DataPoint> best_consensus_set_so_far;
            double lowest_aggregate_error_so_far = std::numeric_limits<double>::infinity();
            bool a_valid_model_was_found = false;

            for (int iter = 0; !termination_condition(iter, lowest_aggregate_error_so_far); ++iter)
            {
                // 1. 从所有数据中采样最小样本集
                std::vector<DataPoint> current_sample = sampler.sample(all_data, Problem::min_samples_required());
                if (current_sample.size() < Problem::min_samples_required())
                {
                    continue; // 采样失败或样本不足
                }

                // 2. 用采样点拟合模型
                Model candidate_model = problem.fitModel(current_sample);
                // 可选: 如果 Problem 提供了 isValid(Model) 方法，可以在这里验证 candidate_model

                // 3. 确定当前模型的内点集 (共识集)
                std::vector<DataPoint> current_consensus_set;
                current_consensus_set.reserve(all_data.size()); // 预分配
                for (const auto &pt : all_data)
                {
                    if (problem.computeError(pt, candidate_model) < Problem::inlier_threshold())
                    {
                        current_consensus_set.push_back(pt);
                    }
                }

                // 4. 评估候选模型：仅当共识集足够大时才进行
                if (current_consensus_set.size() >= Problem::min_samples_required())
                {
                    // 计算当前模型在其共识集上的总误差 (或平均误差/RMS误差)
                    double current_model_error_on_its_inliers = 0.0;
                    if (!current_consensus_set.empty())
                    {
                        for (const auto &inlier_pt : current_consensus_set)
                        {
                            current_model_error_on_its_inliers += problem.computeError(inlier_pt, candidate_model);
                        }
                        // 可以选择使用平均误差：
                        // current_model_error_on_its_inliers /= current_consensus_set.size();
                        // 或者RMS误差:
                        // double sum_sq_error = 0.0;
                        // for (const auto &inlier_pt : current_consensus_set) {
                        //     double err = problem.computeError(inlier_pt, candidate_model);
                        //     sum_sq_error += err * err;
                        // }
                        // current_model_error_on_its_inliers = std::sqrt(sum_sq_error / current_consensus_set.size());
                    }
                    else
                    {
                        current_model_error_on_its_inliers = std::numeric_limits<double>::infinity();
                    }

                    // 5. 如果当前模型更好 (误差更小)，则更新最佳模型
                    //    可以加入一个次要条件：如果误差相似，选择内点更多的模型
                    if (current_model_error_on_its_inliers < lowest_aggregate_error_so_far)
                    {
                        lowest_aggregate_error_so_far = current_model_error_on_its_inliers;
                        best_model_so_far = candidate_model; // 这是基于最小样本集拟合的模型
                        best_consensus_set_so_far = current_consensus_set;
                        a_valid_model_was_found = true;
                    }
                    // 可选的次要条件:
                    // else if (std::abs(current_model_error_on_its_inliers - lowest_aggregate_error_so_far) < some_epsilon &&
                    //          current_consensus_set.size() > best_consensus_set_so_far.size())
                    // {
                    //     lowest_aggregate_error_so_far = current_model_error_on_its_inliers;
                    //     best_model_so_far = candidate_model;
                    //     best_consensus_set_so_far = current_consensus_set;
                    //     a_valid_model_was_found = true;
                    // }
                }
            }

            // 6. 最终优化：如果找到了一个好的共识集，用它来重新拟合模型
            if (a_valid_model_was_found && best_consensus_set_so_far.size() >= Problem::min_samples_required())
            {
                if (out_best_consensus_set)
                {
                    *out_best_consensus_set = best_consensus_set_so_far;
                }
                return problem.fitModel(best_consensus_set_so_far); // 返回用所有最佳内点拟合的模型
            }

            // 未找到合适的模型
            if (out_best_consensus_set)
                out_best_consensus_set->clear();
            return Model{}; // 返回默认构造的模型表示失败
        }
    };

    template <
        typename Problem = PlaneFittingProblem,
        typename Validator = PlaneValidator,
        typename Termination = TerminationCondition>
    class PlaneRansac
    {
    public:
        using DataPoint = typename Problem::DataPoint;
        using Model = typename Problem::Model;

        Model run(
            const std::vector<DataPoint> &data,
            const Problem &problem = Problem(),
            const Validator &validator = Validator(),
            const Termination &termination = Termination(50, 10, 0))
        {
            std::vector<DataPoint> current_inliers = data;
            Model best_model;
            int best_inlier_count = 0;
            Termination term = termination;

            for (int iter = 0;; ++iter)
            {
                if (current_inliers.size() < problem.min_samples_required())
                    break;

                // 1. 模型拟合
                Model model = problem.fitModel(current_inliers);

                // 2. 模型验证
                if (!validator.isValid(model))
                    continue;

                // 3. 内点筛选
                std::vector<DataPoint> new_inliers;
                for (const auto &pt : data)
                {
                    if (problem.computeError(pt, model) < problem.inlier_threshold())
                        new_inliers.push_back(pt);
                }

                // 4. 更新最佳模型
                if (new_inliers.size() > best_inlier_count)
                {
                    best_model = model;
                    best_inlier_count = new_inliers.size();
                }

                // 5. 终止条件判断
                int outlier_count = data.size() - new_inliers.size();
                if (term(iter, outlier_count))
                    break;

                // 6. 更新迭代状态
                current_inliers = std::move(new_inliers);
            }

            return best_model;
        }
    };

} // namespace optimization

#endif