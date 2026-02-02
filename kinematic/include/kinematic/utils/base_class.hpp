#ifndef BASE_CLASS_HPP
#define BASE_CLASS_HPP

#include <atomic>
#include <cstddef>
#include <vector>
#include <memory>
#include <algorithm>
#include <iostream>
// #define CACHE_SIZE 300
namespace rune
{
    /**
     * @brief 枚举模式
     * 比赛场上依据不同的需求所设定的模式
     */
    enum Mode
    {
        /// 时间帧对齐标定模式
        MODE_SYNCHRONIZOR_CALIB = -2,

        /// 坐标解算标定模式
        MODE_TARGETSOLVE_CALIB = -1,

        /// 由电控控制
        MODE_AUTO = 0,

        /// 二代自瞄
        MODE_ARMOR = 1,

        /// 小能量机关
        MODE_SMALLRUNE = 2,

        /// 大能量机关
        MODE_BIGRUNE = 3,

        /// 跟随击打前哨战
        MODE_DYNAMIC_OUTPOST = 4,

        /// 固定击打前哨战
        MODE_FIX_OUTPOST = 5,

        MODE_OUTPOST_F = 6,
    };
    template <typename StateEnum>
    class StateMachine
    {
    protected:
        StateEnum current_state_;
        StateEnum previous_state_;

    public:
        explicit StateMachine(StateEnum init_state) : current_state_(init_state), previous_state_(init_state) {}

        StateEnum current_state() const { return current_state_; }
        virtual void transition_to(StateEnum new_state) = 0;
    };

    // 环形缓冲区槽位（无锁实现）
    template <typename CustomData>
    struct alignas(64) Slot
    { // 缓存行对齐，防止伪共享
        std::shared_ptr<CustomData> data;
        std::atomic<uint64_t> sequence = UINT64_MAX; // 序列号用于排序
    };
    template <typename CustomData, size_t CACHE_SIZE>
    class LockFreeCache
    {
        using SlotC = Slot<CustomData>;

    private:
        SlotC slots[CACHE_SIZE];               // 环形缓冲区
        std::atomic<uint64_t> producer_seq{0}; // 生产者序列号（原子计数器）

    public:
        void push(const CustomData &item)
        {
            // 获取序列号并计算槽位
            const uint64_t seq = producer_seq.fetch_add(1, std::memory_order_release);
            SlotC &slot = slots[seq % CACHE_SIZE];

            // 更新槽位（保证指针先于序列号更新）
            slot.data = std::make_shared<CustomData>(item);
            slot.sequence.store(seq, std::memory_order_release);
        }

        std::vector<std::shared_ptr<CustomData>> snapshot()
        {
            // 1. 获取当前生产者的序列号
            const uint64_t current_seq = producer_seq.load(std::memory_order_acquire);
            // std::cout << "current_seq: " << current_seq << " " << current_seq - CACHE_SIZE << std::endl;
            // 2. 遍历所有槽位收集有效数据
            std::vector<std::pair<uint64_t, std::shared_ptr<CustomData>>> temp;
            const uint64_t window_start = (current_seq > CACHE_SIZE) ? (current_seq - CACHE_SIZE) : 0;

            for (SlotC &slot : slots)
            {
                const uint64_t slot_seq = slot.sequence.load(std::memory_order_acquire);
                if (slot_seq >= window_start && slot_seq < current_seq)
                {
                    temp.emplace_back(slot_seq, slot.data);
                }
            }

            // 3. 按序列号排序
            std::sort(temp.begin(), temp.end(), [](auto &a, auto &b)
                      { return a.first < b.first; });

            // 4. 提取排序后的数据
            std::vector<std::shared_ptr<CustomData>> result;
            result.reserve(temp.size());
            for (auto &p : temp)
            {
                result.push_back(p.second);
            }

            return result;
        }

        std::shared_ptr<CustomData> get_latest()
        {
            const uint64_t current_seq = producer_seq.load(std::memory_order_acquire);
            SlotC &slot = slots[(current_seq - 1) % CACHE_SIZE];
            return slot.data;
        }

        const uint64_t get_seq() const { return producer_seq.load(std::memory_order_acquire); }

        void clear()
        {
            // 遍历slots中的每一个SlotC对象
            producer_seq.store(0, std::memory_order_release);
            for (SlotC &slot : slots)
            {
                // 将slot.sequence设置为UINT64_MAX，并使用std::memory_order_release内存顺序
                slot.sequence.store(UINT64_MAX, std::memory_order_release);
            }
            // 将producer_seq设置为0，并使用std::memory_order_release内存顺序
        }
    };

}

#endif