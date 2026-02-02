#ifndef RuneFireController_HPP
#define RuneFireController_HPP

#include <rclcpp/rclcpp.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "interfaces/msg/sendpack.hpp"
#include "interfaces/msg/readpack.hpp"
#include "interfaces/srv/rune_fire.hpp"
#include <Eigen/Dense>
#include <ceres/ceres.h>

#include "ballistic_solver.hpp"
#include "base_class.hpp"

namespace rune
{

    enum class unfire_reason
    {
        FIRE = 0,
        CANNOT_FIRE = 1,
        HAS_FIRED = 2,
    };

    class RuneFireController : public rclcpp::Node
    {
    private:
        BallisticSolver solver_;

        rune::Mode mode = MODE_AUTO;

        // 记录电控是否响应开火请求
        bool is_fire_answered = false;

        // 记录请求开火
        bool rune_request_fire = false;

        bool rune_request_fire_last = false;

        // 记录开火
        bool fire = false;

        // 记录弹速
        float bullet_speed = 30;

        /// 上次发弹的mcu时间，单位s
        double last_shooting_time = 0.0;

        /// 上次向电控发送发弹标志的时间，单位s
        double last_send_rune_fire_time = 0.0;

        /// 用于发弹异常的数据记录（不发弹的原因记录）
        unfire_reason flag_unfire_reason = unfire_reason::CANNOT_FIRE;

        /// 发弹延迟
        double shoot_delay = 0.0;

        /// 最小发弹间隔
        double min_shoot_period = 0.65;

        // 最小切换间隔，事实上是在换扇叶后，电控响应时间，假设这段时间后电控就能从一个扇叶响应到另一个扇叶，跟随时的响应时间可以忽略不计
        double min_switch_period = 0.2;

        // 最大切换间隔
        double max_switch_period = 1.8;

        bool MANUAL_FIRE = false;

        bool DEBUG_MODE = false;

        bool RUNE_CALIB = false;

        // 重力加速度
        double g = 9.7988;

        // 小弹丸质量
        double bullet_m = 3.2e-3;

        // 小弹丸空气阻力系数
        double bullet_k = 7.1655e-5;
        // 发送
        //  能量机关弹道计算系数
        double rune_trajectory_k = 0.087;

        // 能量机关发送偏移量，即与云台数值之间的偏移量
        double send_rune_pitch_offset = 0.15;
        double send_rune_yaw_offset = 0.62;
        // 能量机关发送计算得到的最低的pitch
        double send_rune_low_pitch = 0.0;
        // 接收
        //  能量机关最低点的pitch值
        double raw_rune_low_pitch = 0.0;
        // 能量机关陀螺仪增量误差系数k
        double rune_k = 0.0;
        // 能量机关接收pitch offset
        double receive_rune_pitch_offset = 0.0;
        // 能量机关接收yaw offset
        double receive_rune_yaw_offset = 0.0;
        // roll_offset
        double receive_rune_roll_offset = 0.0;

        interfaces::msg::Sendpack send_pack;
        interfaces::msg::Readpack read_pack;
        visualization_msgs::msg::Marker marker;
        visualization_msgs::msg::MarkerArray marker_array;

        rclcpp::Subscription<interfaces::msg::Readpack>::SharedPtr read_pack_subscription_;
        rclcpp::Publisher<interfaces::msg::Sendpack>::SharedPtr sendpack_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Client<interfaces::srv::RuneFire>::SharedPtr client_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_publisher_;

        // 核心状态
        rclcpp::Time current_pred_time_;
        Eigen::Vector3d predTarget_{};
        std::mutex state_mutex_;
        // 迭代控制状态
        enum class IterState
        {
            IDLE,
            BUSY
        };
        IterState iter_state_ = IterState::IDLE; // 当前状态
        uint8_t active_iter_num_ = 0;            // 已激活的迭代次数
        bool prediction_cycle_active_ = false;   // 需要重新启动循环的标志

        // 拆分后的子函数
        void reset_prediction_cycle();
        void handle_async_response(rclcpp::Client<interfaces::srv::RuneFire>::SharedFuture future,
                                   rclcpp::Time request_time);
        void handle_invalid_response();
        void start_single_iteration();
        // 工具函数
        Eigen::Vector3d process_coordinate_transform(const geometry_msgs::msg::PointStamped &target_point,
                                                     Eigen::Vector3d current_euler_angle,
                                                     rclcpp::Time stamp);
        std::tuple<Eigen::Vector3d, double> calculate_trajectory(const Eigen::Vector3d &target_point,
                                                                 double bullet_speed);
        void make_control_decision(const Eigen::Vector3d &euler_angle,
                                   const Eigen::Vector3d &current_euler_angle,
                                   bool should_fire);
        bool is_angle_safe(const Eigen::Vector3d &angle, const Eigen::Vector3d &gimbal_angle) const
        {
            constexpr double MAX_ANGLE = M_PI / 4;
            return (std::fabs(angle.x() - gimbal_angle.x()) <= MAX_ANGLE) && (std::fabs(angle.y() - gimbal_angle.y()) <= MAX_ANGLE);
        }

        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> shoot_delay_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> min_shoot_period_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> min_switch_period_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> max_switch_period_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> rune_trajectory_k_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> send_rune_pitch_offset_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> send_rune_yaw_offset_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> send_rune_low_pitch_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> DEBUG_MODE_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> RUNE_CALIB_cb_handle_;

    public:
        RuneFireController(const rclcpp::NodeOptions &options);
        ~RuneFireController();

        void load_param();
        void read_callback(const interfaces::msg::Readpack::SharedPtr msg);
        void timer_callback();
        void setRuneFireData();
        void clear();
        void set_send_pack(Eigen::Vector3d euler_angle, rclcpp::Time t);
        void keep_pose();
    };

} // namespace rune

#endif
