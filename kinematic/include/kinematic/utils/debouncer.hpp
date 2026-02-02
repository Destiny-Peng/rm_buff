#ifndef DEBOUNCER_HPP
#define DEBOUNCER_HPP
#include <unordered_set>
#include <optional>

namespace rune
{
    class EdgeDebouncer
    {
    public:
        // 构造函数，设置确认次数
        explicit EdgeDebouncer(int confirm_times = 5)
            : confirm_threshold_(confirm_times) {}

        // 更新当前稳定值
        const std::unordered_set<int> &update(const std::unordered_set<int> &new_obs)
        {
            // 如果当前稳定值为空，则直接接受初始值
            if (!current_stable_.has_value())
            {
                // 首次调用，直接接受初始值
                current_stable_ = new_obs;
                return current_stable_.value();
            }

            if (*current_stable_ == new_obs) //  如果当前稳定值等于新观测值
            {
                reset_pending();         //  重置待定值
                return *current_stable_; //  返回当前稳定值
            }

            if (!pending_obs_.has_value() || *pending_obs_ != new_obs) //  如果待定值不存在或者待定值不等于新观测值
            {
                pending_obs_ = new_obs;  //  将新观测值赋值给待定值
                pending_counter_ = 1;    //  将待定计数器重置为1
                return *current_stable_; //  返回当前稳定值
            }

            if (++pending_counter_ >= confirm_threshold_) //  如果待定计数器大于等于确认阈值
            {
                current_stable_ = pending_obs_; //  将待定值赋值给当前稳定值
                reset_pending();                //  重置待定值
            }
            return *current_stable_;
        }

        // 返回当前稳定值
        const std::unordered_set<int> &get_current_stable() const { return current_stable_.value(); }

        // 可选：手动重置初始值
        void reset(const std::unordered_set<int> &obs)
        {
            current_stable_ = obs;
            reset_pending();
        }

    private:
        // 重置待处理数据
        void reset_pending()
        {
            // 重置待处理数据
            pending_obs_.reset();
            // 重置待处理数据计数器
            pending_counter_ = 0;
        }

        std::optional<std::unordered_set<int>> current_stable_; // 当前稳定值（明确可空）
        std::optional<std::unordered_set<int>> pending_obs_;    // 待确认值
        int pending_counter_ = 0;
        int confirm_threshold_;
    };
} // namespace rune

#endif