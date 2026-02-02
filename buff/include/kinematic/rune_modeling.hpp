#ifndef RuneModeling_HPP
#define RuneModeling_HPP
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "interfaces/msg/rune.hpp"
#include "interfaces/msg/fan_armor.hpp"
#include "interfaces/msg/fan_armors.hpp"
#include "interfaces/srv/rune_fire.hpp"
#include "interfaces/msg/readpack.hpp"

#include "utils/rune_fan_defination.hpp"
#include "optimization/ransac.hpp"
namespace rune
{
    using std::placeholders::_1, std::placeholders::_2;

    class RuneModeling : public rclcpp::Node
    {

    private:
        Mode mode = MODE_AUTO;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        rclcpp::Subscription<interfaces::msg::FanArmors>::SharedPtr fanArmors_sub;
        rclcpp::Subscription<interfaces::msg::Readpack>::SharedPtr readPack_sub;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Service<interfaces::srv::RuneFire>::SharedPtr runeFire_srv_;

        bool flag_all_activate = false;
        bool flag_detect_success = false;
        bool use_sim = false;
        bool RUNE_CALIB = false;
        bool DEBUG_MODE = false;
        bool fit_valid = false;
        double time_start{};
        Eigen::Isometry3d world_to_rune_transform_{}; // 世界坐标系到rune坐标系的变换矩阵
        rune::optimization::CurveFittingProblem::Model modeling_param_{};

        rune::LockFreeCache<interfaces::msg::FanArmors, 50> fan_armor_cache_;
        rune::LockFreeCache<rune::Rune, 800> rune_cache_;

        // 拟合相关变量
        std::mutex fitting_mutex_;
        std::atomic<bool> is_fitting_{false};
        std::future<void> fitting_future_{};
        std::chrono::steady_clock::time_point last_fit_time_;
        rune::optimization::ErrorDrivenRansac<rune::optimization::CurveFittingProblem,
                                              rune::optimization::RandomPointSampler,
                                              rune::optimization::MaxIterationsOrErrorThresholdTermination>
            angle_fitting_;
        rune::optimization::DynamicModelSelector model_selector_;
        rune::optimization::CurveFittingProblem problem_;
        // 最小拟合间隔（单位：毫秒）
        static constexpr int MIN_FIT_INTERVAL = 100;
        double min_error = 10000.0;

        // 最小切换间隔，事实上是在换扇叶后，电控响应时间，假设这段时间后电控就能从一个扇叶响应到另一个扇叶，跟随时的响应时间可以忽略不计
        double min_switch_period = 0.2;

        // 最大切换间隔
        double max_switch_period = 1.8;

        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> receive_rune_pitch_offset_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> receive_rune_yaw_offset_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> receive_rune_roll_offset_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> raw_rune_low_pitch_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> rune_k_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> min_switch_period_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> max_switch_period_cb_handle_;

    private:
        void loadParam();
        void clear();
        void fanArmorsCallback(const interfaces::msg::FanArmors::SharedPtr msg);
        void timerCallback();
        Rune handle_fanArmors(const interfaces::msg::FanArmors &fan_armors, const Rune &last_rune);
        interfaces::msg::FanArmors transform_fanArmors(const interfaces::msg::FanArmors &msg, const Eigen::Isometry3d &T);
        void RuneFireService(const interfaces::srv::RuneFire_Request::SharedPtr request, interfaces::srv::RuneFire_Response::SharedPtr response);

        // 触发角度拟合的接口函数
        void trigger_angle_fitting();

        // 实际拟合任务实现
        void fit_angle_task();

        // 拟合完成后的收尾工作
        void finalize_fitting();
        bool judgeFire(double target_time, double last_fan_switched_time);
        void readPackCallback(const interfaces::msg::Readpack::SharedPtr msg);

    public:
        RuneModeling(const rclcpp::NodeOptions &options);
        ~RuneModeling();
    };

}
#endif