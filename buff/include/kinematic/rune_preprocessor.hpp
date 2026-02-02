#ifndef RunePreprocessor_HPP
#define RunePreprocessor_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "interfaces/msg/rune.hpp"
#include "interfaces/msg/fan_armor.hpp"
#include "interfaces/msg/fan_armors.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include <opencv2/core/eigen.hpp>

#include "pnp_solver.hpp"
#include "base_class.hpp"
#include "optimization/ransac.hpp"

#include "fstream"

namespace rune
{

    struct twopoint
    {
        Eigen::Vector3d p1;
        Eigen::Vector3d p2;
    };
    class RunePreprocessor : public rclcpp::Node
    {
    private:
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_pub;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        rclcpp::Publisher<interfaces::msg::FanArmors>::SharedPtr fan_armor_pub;
        rclcpp::Subscription<interfaces::msg::Rune>::SharedPtr rune_img_process_result_sub;
        std::vector<std::vector<cv::Point2d>> rune_armor_points;
        rclcpp::TimerBase::SharedPtr timer_;

        interfaces::msg::Rune RunePreprocessor_result;
        bool flag_r_detected = false;
        bool flag_all_activate = false;
        bool flag_detect_success = false;
        bool use_sim = false;

        cv::Point2d r_center_pixel{};
        std::string g_camera_param_path;
        std::string g_tune_param_path;
        cv::Mat camera_matrix_;    // 相机内参矩阵
        cv::Mat distortion_coeff_; // 畸变系数

        geometry_msgs::msg::TransformStamped gimbal_tf_msg{};
        geometry_msgs::msg::TransformStamped camera_tf_msg{};
        geometry_msgs::msg::TransformStamped target_tf_msg{};
        geometry_msgs::msg::TransformStamped rune_tf_msg{};

        // PnP计算结果缓存
        LockFreeCache<twopoint, 150> pnp_result_cache_;

        // 拟合相关变量
        std::mutex fitting_mutex_;
        std::atomic<bool> is_fitting_{false};
        std::future<void> fitting_future_{};
        std::chrono::steady_clock::time_point last_fit_time_;
        rune::optimization::ErrorDrivenRansac<rune::optimization::ZConstrainedPlaneFitting,
                                              rune::optimization::RandomPointSampler,
                                              rune::optimization::MaxIterationsOrErrorThresholdTermination>
            plane_ransac;
        rune::optimization::ZConstrainedPlaneFitting problem_;

        // 最小拟合间隔（单位：毫秒）
        static constexpr int MIN_FIT_INTERVAL = 100;
        Eigen::Quaterniond quaternion_from_euler(double yaw, double pitch, double roll);
        // 收到图像处理结果，处理并发布
        void rune_callback(const interfaces::msg::Rune::SharedPtr msg);
        void clear();
        // 消息数据转换为本地格式，发布tf
        bool convertMsgToLocalData(const interfaces::msg::Rune::SharedPtr msg);
        void timer_callback();

        // 触发圆形拟合的接口函数
        void trigger_circle_fitting();

        // 实际拟合任务实现
        void fit_circle_task();

        // 拟合完成后的收尾工作
        void finalize_fitting();

    public:
        std::unique_ptr<PnPSolver> pnp_solver;

        RunePreprocessor(const rclcpp::NodeOptions &options);
        ~RunePreprocessor();

        // 根据fan对象和其他收到的标志位，进行逻辑处理
        void logical_process();
    };
}

#endif