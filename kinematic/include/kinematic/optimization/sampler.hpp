#ifndef SAMPLER_HPP
#define SAMPLER_HPP
#include <vector>
#include <algorithm>

namespace rune::optimization
{
    // *****************************
    //         策略组件默认实现
    // *****************************

    // 默认随机采样器
    struct RandomSampler
    {
        template <typename Data>
        std::vector<Data> sample(const std::vector<Data> &data, size_t n) const
        {
            std::vector<Data> result;
            std::sample(data.begin(), data.end(), std::back_inserter(result),
                        n, std::mt19937{std::random_device{}()});
            return result;
        }
    };

    // *****************************
    //         策略组件
    // *****************************

    // 自适应采样策略
    struct AdaptiveSampler
    {
        template <typename Data>
        std::vector<Data> sample(const std::vector<Data> &data, size_t n) const
        {
            if (data.size() > 400)
            {
                std::vector<Data> shuffled(data.end() - 200, data.end());
                std::shuffle(shuffled.begin(), shuffled.end(),
                             std::mt19937(std::random_device{}()));
                return {shuffled.end() - 100, shuffled.end()};
            }
            return {data.begin(), data.end()};
        }
    };

    // 随机点采样器
    struct RandomPointSampler
    {
        template <typename Data>
        std::vector<Data> sample(const std::vector<Data> &data_pool, size_t n_to_sample) const
        {
            if (data_pool.empty() || n_to_sample == 0)
            {
                return {};
            }
            if (data_pool.size() < n_to_sample)
            {
                // Not enough data to sample the required number, return all data (or handle error)
                // For RANSAC, it's often better to signal failure or return an empty set
                // if the minimum cannot be met.
                // However, if data_pool.size() >= Problem::min_samples_required() but < n_to_sample,
                // it might be okay to return data_pool.
                // Let's assume n_to_sample is usually Problem::min_samples_required().
                // If data_pool.size() < n_to_sample, it implies data_pool.size() < Problem::min_samples_required().
                return {}; // Indicate failure to sample enough points
            }

            std::vector<Data> result;
            result.reserve(n_to_sample);
            std::sample(data_pool.begin(), data_pool.end(),
                        std::back_inserter(result),
                        n_to_sample,
                        std::mt19937{std::random_device{}()});
            return result;
        }
    };
}
#endif