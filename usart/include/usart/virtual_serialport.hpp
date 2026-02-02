#ifndef VIRTUAL_SERIALPORT_HPP
#define VIRTUAL_SERIALPORT_HPP
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/readpack.hpp"
#include "interfaces/msg/sendpack.hpp"
#include <iostream>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h".
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
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

template <typename T>
class PIDController
{
public:
    PIDController() {}
    PIDController(double kp, double ki, double kd)
        : Kp(kp), Ki(ki), Kd(kd) {}
    PIDController &operator=(const PIDController &other)
    {
        this->Kp = other.Kp;
        this->Ki = other.Ki;
        this->Kd = other.Kd;
        return *this;
    }

    T getTargetValue() const
    {
        return target_val;
    }

    void setTargetValue(T target_value)
    {
        target_val = target_value;
    }

    T getControlledValue() const
    {
        return controlled_val;
    }

    void control()
    {
        // 计算误差
        T error = target_val - controlled_val;

        // 检测目标值是否发生突变
        if (std::abs(static_cast<double>(error)) > 24.0)
        {
            // 目标值发生突变,积分项复位
            error_integral = static_cast<T>(0.0);
        }
        else
        {
            error_integral += error;
        }

        // 计算误差导数
        T error_derivative = error - error_prev;
        error_prev = error;

        // 计算 PID 控制输出
        T output = static_cast<T>(Kp * error + Ki * error_integral + Kd * error_derivative);

        // 更新控制值
        controlled_val += output;
    }

private:
    double Kp{}, Ki{}, Kd{};
    T target_val;
    T controlled_val{};
    T error_integral{};
    T error_prev{};
};

class VirtualGimbal
{
public:
    VirtualGimbal()
    {
        // 初始化云台控制器
        current_pose_.header.frame_id = "gimbal_link";
        current_pose_.pose.orientation.w = 1.0;
        target_pose_ = current_pose_;
        yaw_control = PIDController<double>(0.1, 0.0, 0.0);
        pitch_control = PIDController<double>(0.1, 0.0, 0.0);
        tf_msg.header.frame_id = "base_link";
        tf_msg.child_frame_id = "gimbal_link";

        // 创建定时器更新云台姿态
        // timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&VirtualGimbal::updatePose, this));
        // 创建 TF 广播器
        // tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }
    ~VirtualGimbal() {}

    void setTargetPose(const geometry_msgs::msg::PoseStamped &target_pose)
    {
        target_pose_ = target_pose;
        Eigen::Quaterniond q(target_pose_.pose.orientation.w, target_pose_.pose.orientation.x, target_pose_.pose.orientation.y, target_pose_.pose.orientation.z);
        Eigen::Matrix3d R = q.normalized().toRotationMatrix();
        Eigen::Vector3d euler = R.eulerAngles(2, 1, 0);
        yaw_control.setTargetValue(euler[0]);
        pitch_control.setTargetValue(euler[1]);
    }

    void setTargetPose(double yaw, double pitch)
    {
        yaw_control.setTargetValue(yaw);
        pitch_control.setTargetValue(pitch);
    }
    geometry_msgs::msg::PoseStamped getCurrentPose()
    {
        return current_pose_;
    }
    void getCurrentPose(double &yaw, double &pitch, double &roll)
    {
        yaw = this->yaw_control.getControlledValue();
        pitch = this->pitch_control.getControlledValue();
        roll = this->roll;
    }

    geometry_msgs::msg::TransformStamped get_tf_msg()
    {
        tf_msg.header.stamp = rclcpp::Clock().now();
        tf_msg.transform.rotation = current_pose_.pose.orientation;
        return tf_msg;
    }

    void updatePose()
    {
        // 根据 PID 控制器更新云台姿态

        // to be continued
        this->yaw_control.control();
        this->pitch_control.control();
        // 更新当前姿态
        Eigen::Quaterniond q = this->quaternion_from_euler(this->yaw_control.getControlledValue(), -this->pitch_control.getControlledValue(), -this->roll);
        current_pose_.pose.orientation.w = q.w();
        current_pose_.pose.orientation.x = q.x();
        current_pose_.pose.orientation.y = q.y();
        current_pose_.pose.orientation.z = q.z();
        // 发布 TF
    }
    void setCurrentroll(double roll)
    {
        this->roll = roll;
    }

private:
    Eigen::Quaterniond quaternion_from_euler(double yaw, double pitch, double roll)
    {
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
        // 已经搞不懂了，干脆内旋完事了。
        return yawAngle * pitchAngle * rollAngle;
    }

    PIDController<double> yaw_control;
    PIDController<double> pitch_control;
    double roll{};
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::PoseStamped target_pose_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::TransformStamped tf_msg;
};

class VirtualSerial : public rclcpp::Node
{
private:
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_1;
    rclcpp::Subscription<interfaces::msg::Sendpack>::SharedPtr sendpack_sub_;
    rclcpp::Publisher<interfaces::msg::Readpack>::SharedPtr readpack_pub_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> gimbal_yaw_cb_handle_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> gimbal_pitch_cb_handle_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> gimbal_roll_cb_handle_;
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;

    VirtualGimbal gimbal_;
    interfaces::msg::Readpack readpack_msg;
    interfaces::msg::Sendpack sendpack_msg;
    double target_fire_time{};
    Mode mode;
    void update_gimbal()
    {
        this->gimbal_.updatePose();
        this->tf_broadcaster_->sendTransform(this->gimbal_.get_tf_msg());
    }
    void callback_sendpack(const interfaces::msg::Sendpack::SharedPtr msg)
    {
        // 处理接收到的数据
        // 调用 gimbal_ 类的 setTargetPose 方法设置目标姿态
        // std::cout << "Received data: " << msg->pred_yaw << ", " << msg->pred_pitch << std::endl;
        this->gimbal_.setTargetPose(msg->pred_yaw * M_PI / 180, msg->pred_pitch * M_PI / 180);
        switch (this->mode)
        {
        case Mode::MODE_SMALLRUNE:
        case Mode::MODE_BIGRUNE:
            if (msg->rune_fire != sendpack_msg.rune_fire)
            {
                this->target_fire_time = this->get_clock()->now().seconds() + 0.14;
            }

            break;

        default:
            break;
        }
        this->sendpack_msg = *msg;
    }
    void pub_readpack()
    {
        double yaw, pitch, roll;
        this->gimbal_.getCurrentPose(yaw, pitch, roll);
        readpack_msg.pose.ptz_yaw = yaw * 180 / M_PI;
        readpack_msg.pose.ptz_pitch = pitch * 180 / M_PI;
        readpack_msg.pose.ptz_roll = roll * 180 / M_PI;
        readpack_msg.pose.visual_time = this->get_clock()->now().seconds();
        readpack_msg.mode = this->mode;
        readpack_msg.bullet_speed = 27.0;

        switch (this->mode)
        {
        case Mode::MODE_SMALLRUNE:
        case Mode::MODE_BIGRUNE:
            if (fabs(this->get_clock()->now().seconds() - this->target_fire_time) < 0.001)
            {
                readpack_msg.fired = !readpack_msg.fired;
            }

            break;

        default:
            break;
        }

        readpack_pub_->publish(readpack_msg);
    }

public:
    VirtualSerial(std::string node_name) : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "%s init.", node_name.c_str());
        // 创建定时器更新云台姿态
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&VirtualSerial::update_gimbal, this));
        // 创建定时器发布readpack
        timer_1 = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&VirtualSerial::pub_readpack, this));
        // 创建订阅
        sendpack_sub_ = this->create_subscription<interfaces::msg::Sendpack>("Sendpack", 1, std::bind(&VirtualSerial::callback_sendpack, this, std::placeholders::_1));
        // 创建发布
        readpack_pub_ = this->create_publisher<interfaces::msg::Readpack>("Readpack", 1);
        // 创建 TF 广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        this->mode = Mode::MODE_BIGRUNE;

        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        this->declare_parameter("virtual_gimbal_pitch", 0.0);
        this->declare_parameter("virtual_gimbal_yaw", 0.0);
        this->declare_parameter("virtual_gimbal_roll", 0.0);

        // 添加参数回调，动态调参
        auto gimbal_yaw_callback = [this](const rclcpp::Parameter &p)
        {
            this->gimbal_.setTargetPose(p.as_double() * M_PI / 180, this->get_parameter("virtual_gimbal_pitch").as_double() * M_PI / 180);
        };
        auto gimbal_pitch_callback = [this](const rclcpp::Parameter &p)
        {
            this->gimbal_.setTargetPose(this->get_parameter("virtual_gimbal_yaw").as_double() * M_PI / 180, p.as_double() * M_PI / 180);
        };
        auto gimbal_roll_callback = [this](const rclcpp::Parameter &p)
        {
            this->gimbal_.setCurrentroll(p.as_double() * M_PI / 180);
        };
        this->gimbal_yaw_cb_handle_ = param_subscriber_->add_parameter_callback("virtual_gimbal_yaw", gimbal_yaw_callback);
        this->gimbal_pitch_cb_handle_ = param_subscriber_->add_parameter_callback("virtual_gimbal_pitch", gimbal_pitch_callback);
        this->gimbal_roll_cb_handle_ = param_subscriber_->add_parameter_callback("virtual_gimbal_roll", gimbal_roll_callback);
    }
    ~VirtualSerial() {}
};

#endif // virtual_SERIALPORT_HPP