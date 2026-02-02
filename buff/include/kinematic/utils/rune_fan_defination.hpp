#ifndef RUNE_FAN_DEFINATION_HPP
#define RUNE_FAN_DEFINATION_HPP

#include "base_class.hpp"
#include "rune_parameters.hpp"
#include <unordered_set>
#include <vector>
#include <Eigen/Dense>
#include <optional>
#include <opencv2/opencv.hpp>

#include "interfaces/msg/fan_armors.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

#include "optimization/ransac.hpp"
#include "debouncer.hpp"

namespace rune
{

    enum class fan_status
    {
        DARK = 0,
        INACTIVATED = 1,
        ACTIVATED = 2
    };

    // 整体步骤
    enum class rune_model_states_code
    {
        CALIBRATION_FAILED_NEED_RESET = -5,
        ANGLE_CALIBRATION_FAILED = -4,
        CIRCLE_CALIBRATION_FAILED = -3,
        CALIBRATION_INACCESSIBLE = -2,
        POINTS_INVALID = -1,
        ZERO = 0,
        POINTS_INVALID_BUT_COMPLEMENTED = 1,
        POINTS_VALID = 2,
        CIRCLE_CALIBRATION_SUCCESSFUL = 3,
        ANGLE_CALIBRATION_SUCCESSFUL = 4
    };

    // 具体细节
    enum class rune_model_error_code
    {
        NORMAL = 0,
        NO_DETECT_RESULT = -1,
        PNP_RESULT_ERROR = -2,
        NO_VALID_INITIALIZATION_FRAME = -3,
        WRONG_LOGICAL_JUDGEMENT = -4,

    };

    enum class rune_model_process
    {
        SLEEP = 0,
        NARMINAL = 1,
        FAILURE = 2
    };

    enum class FanDetectionStatus
    {
        DETECTED = 0, // 当前扇叶被识别到
        NOT_DETECTED, // 当前扇叶未被识别到
    };
    enum class FanCountStatus
    {
        COUNT_INCREASE = 0, // 检测到的扇叶数量增加
        COUNT_DECREASE,     // 检测到的扇叶数量减少
        COUNT_STABLE,       // 检测到的扇叶数量稳定
        COUNT_ONE           // 检测到的扇叶数量只有一个
    };
    struct FanEvent
    {
        FanDetectionStatus detection_status;
        FanCountStatus count_status;
    };

    class Fan : public StateMachine<fan_status>
    {
    public:
        /// 中心点与圆心的角度(0~2pi)弧度
        double angle{};

        // 上一次检测结果
        bool is_detected_last_ = false; // 是否被识别到
        int last_detect_count_ = 0;     // 上一次检测到的总扇叶数量
        FanEvent last_event{FanDetectionStatus::NOT_DETECTED, FanCountStatus::COUNT_ONE};

    private:
        fan_status target_status = fan_status::DARK;

        /// 扇叶的id，第一片亮起的扇叶为0号扇叶,未初始化时为-1.逆时针为正方向
        int id = -1;

        Eigen::Vector3d armor_center{};

        /// 圆心
        Eigen::Vector3d rotation_center{};

        /// 旋转轴，默认为x轴
        Eigen::Vector3d rotate_axis = Eigen::Vector3d::UnitX();

        /// 上次切换状态时的时间戳
        double last_switch_time{};

        /// 为了防止误识别导致的逻辑错误，在扇叶状态切换时进行计时，持续超过一定帧数才认为切换了状态
        int switch_cnt = 0;
        const int switch_cnt_threshold = 0; // 稳定阈值

    public:
        Fan();
        Fan &operator=(const Fan &f);

        fan_status get_target_status() const { return target_status; };

        // 事件生成函数
        FanEvent generate_event(bool is_detected_current, int detect_count_current);
        bool handle_event(FanEvent event);

        // 坐标变换相关
        void rotate(double angle_rad);
        void rotate(double angle_rad, const Eigen::Vector3d &rotate_axis);
        Eigen::Vector3d get_rotation_center() const { return rotation_center; };
        Eigen::Vector3d get_armor_center() const { return armor_center; };
        Eigen::Vector3d get_vector() const { return armor_center - rotation_center; };
        int get_id() const { return id; };
        int get_threshold() const { return switch_cnt_threshold; };
        double get_last_switch_time() const { return last_switch_time; };
        int get_switch_cnt() const { return switch_cnt; };
        void initialize_id(int id) { this->id = id; };
        void initialize_as_inactive()
        {
            this->current_state_ = fan_status::INACTIVATED;
            this->target_status = fan_status::INACTIVATED;
        }
        void set_position(const interfaces::msg::FanArmor &armor);

        bool is_same_fan(Fan &another_fan);
        std::optional<double> fan_angle_diff(Fan &another_fan, double angle_threshold = M_PI / 5);

        void Update_state();
        void Update_history_variance(bool is_detected_current, int detect_count_current);

    private:
        void before_transition(fan_status old_state);
        void after_transition(fan_status old_state);
        void transition_to(fan_status new_state) override;

        // 状态转换规则决策
        fan_status calculate_target_status(FanEvent event);
    };

    class Rune
    {
    private:
        std::array<Fan, 5> fans;                                          // 五个叶片组成的风扇
        rune::optimization::CurveFittingProblem::Model modeling_param_{}; // 模型参数
        EdgeDebouncer edge_debouncer;                                     // 消抖边沿检测
        std::unordered_set<int> detected_fans_id;                         // 识别到的叶片id

        void extract_info(std::vector<Fan> &detect_fans);
        void rotate_fans(double angle_rad)
        {
            for (auto &fan : this->fans)
            {
                fan.rotate(angle_rad);
            }
        }
        void update_fans_status();
        void self_examination();

    public:
        double time_stamp{};
        double start_time_stamp{};
        int dir{};
        double angle_start{};
        int inactive_index{};
        double delta_angle{};
        int current_detect_fans_size{};
        int failed_detect_count = 0;
        int switch_cnt_ = 0;

        /// 是否识别到r标
        bool flag_r_detected = false;

        Rune();
        Rune(const interfaces::msg::FanArmors &msg);
        Rune(const Rune &other);
        ~Rune();
        void clear();
        void detect_result_to_fan(std::vector<Fan> &detect_fans);
        rune_model_states_code no_detect_interpid(rune::optimization::CurveFittingProblem::Model modeling_param_ = std::array<double, 5>{0.470, 1.942, 0, 1.178, 0}, bool fit_valid = false);
        std::pair<double, double> get_angle_data(int id = 0);
        Eigen::Vector3d get_target_position(double angel_rad = 0) const; // 获取目标位置
        double get_last_switch_time() const { return this->fans.at(this->inactive_index).get_last_switch_time(); };
    };

} // namespace rune
#endif