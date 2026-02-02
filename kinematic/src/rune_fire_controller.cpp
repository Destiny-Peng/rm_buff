

#include "rune_fire_controller.hpp"

namespace rune
{
    std::ostream &operator<<(std::ostream &strm, unfire_reason ur)
    {
        switch (ur)
        {
        case unfire_reason::FIRE:
            return strm << "FIRE";
        case unfire_reason::CANNOT_FIRE:
            return strm << "CANNOT_FIRE";
        case unfire_reason::HAS_FIRED:
            return strm << "HAS_FIRED";
        default:
            return strm << "UNKNOWN";
        }
    }

    RuneFireController::RuneFireController(const rclcpp::NodeOptions &options) : Node("rune_fire_controller", options)
    {
        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        load_param();
        read_pack_subscription_ = this->create_subscription<interfaces::msg::Readpack>("Readpack", 1, std::bind(&RuneFireController::read_callback, this, std::placeholders::_1));
        sendpack_publisher_ = this->create_publisher<interfaces::msg::Sendpack>("Sendpack", 1);
        client_ = this->create_client<interfaces::srv::RuneFire>("RuneFire");
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(), std::chrono::seconds(30));
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        visualization_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("predict_target", 1);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&RuneFireController::timer_callback, this));

        marker.header.frame_id = "base_link";
        marker.header.stamp = rclcpp::Clock().now();
        marker.id = 0;
        marker.ns = "fire_controller";
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.a = 1.0;
        // 箭头部分
        RCLCPP_INFO(this->get_logger(), "%s init.", "rune_fire_controller");
        // this->mode = Mode::MODE_SMALLRUNE;
    }

    RuneFireController::~RuneFireController() {}

    void RuneFireController::load_param()
    {
        // 重力加速度
        this->declare_parameter("gravity", 9.7988);
        // 小弹丸质量参数
        this->declare_parameter("bullet_m", 3.2e-3);
        // 小弹丸空气阻力系数
        this->declare_parameter("bullet_k", 7.1655e-5);

        // 能量机关弹道计算系数
        this->declare_parameter("rune_trajectory_k", 0.087);
        // 能量机关发送偏移量，即与云台数值之间的偏移量
        this->declare_parameter("send_rune_pitch_offset", 0.15);
        this->declare_parameter("send_rune_yaw_offset", 0.62);
        // 能量机关发送计算得到的最低的pitch
        this->declare_parameter("send_rune_low_pitch", 0.0);

        // 发弹延迟
        this->declare_parameter("shoot_delay", 0.0);
        // 最小发弹间隔
        this->declare_parameter("min_shoot_period", 0.65);
        // 最小切换间隔
        this->declare_parameter("min_switch_period", 0.2);
        // 最大切换间隔
        this->declare_parameter("max_switch_period", 1.8);
        // 调试模式
        this->declare_parameter("DEBUG_MODE", false);
        // 标定模式
        this->declare_parameter("RUNE_CALIB", false);
        // 手动开火
        this->declare_parameter("MANUAL_FIRE", false);
        // 相机内参路径
        this->send_pack.yaw_resolution = 0.35;
        this->send_pack.pitch_resolution = 0.5;
        this->send_pack.target_found = true;

        this->declare_parameter("car_id", 3);
        this->declare_parameter<double>("D_big", 42.5e-3);
        this->declare_parameter<double>("M_big", 41e-3);

        if (this->get_parameter("car_id").as_int() == 1 || this->get_parameter("car_id").as_int() == 6)
        {
            solver_.setResistanceCoefficient(this->get_parameter("D_big").as_double() * 0.5, this->get_parameter("M_big").as_double());
        }

        // 读取参数
        g = this->get_parameter("gravity").as_double();
        bullet_m = this->get_parameter("bullet_m").as_double();
        bullet_k = this->get_parameter("bullet_k").as_double();

        rune_trajectory_k = this->get_parameter("rune_trajectory_k").as_double();
        send_rune_pitch_offset = this->get_parameter("send_rune_pitch_offset").as_double();
        send_rune_yaw_offset = this->get_parameter("send_rune_yaw_offset").as_double();
        send_rune_low_pitch = this->get_parameter("send_rune_low_pitch").as_double();

        this->shoot_delay = this->get_parameter("shoot_delay").as_double();
        this->min_shoot_period = this->get_parameter("min_shoot_period").as_double();
        this->min_switch_period = this->get_parameter("min_switch_period").as_double();
        this->max_switch_period = this->get_parameter("max_switch_period").as_double();

        DEBUG_MODE = this->get_parameter("DEBUG_MODE").as_bool();
        RUNE_CALIB = this->get_parameter("RUNE_CALIB").as_bool();
        MANUAL_FIRE = this->get_parameter("MANUAL_FIRE").as_bool();

        // 添加参数回调，动态调参
        auto shoot_delay_callback = [this](const rclcpp::Parameter &p)
        {
            this->shoot_delay = p.as_double();
            RCLCPP_INFO(this->get_logger(), "shoot_delay: %f", this->shoot_delay);
        };
        auto min_shoot_period_callback = [this](const rclcpp::Parameter &p)
        {
            this->min_shoot_period = p.as_double();
            RCLCPP_INFO(this->get_logger(), "min_shoot_period: %f", this->min_shoot_period);
        };
        auto min_switch_period_callback = [this](const rclcpp::Parameter &p)
        {
            this->min_switch_period = p.as_double();
            RCLCPP_INFO(this->get_logger(), "min_switch_period: %f", this->min_switch_period);
        };
        auto max_switch_period_callback = [this](const rclcpp::Parameter &p)
        {
            this->max_switch_period = p.as_double();
            RCLCPP_INFO(this->get_logger(), "max_switch_period: %f", this->max_switch_period);
        };
        auto rune_trajectory_k_callback = [this](const rclcpp::Parameter &p)
        {
            rune_trajectory_k = p.as_double();
            RCLCPP_INFO(this->get_logger(), "rune_trajectory_k: %f", rune_trajectory_k);
        };
        auto send_rune_pitch_offset_callback = [this](const rclcpp::Parameter &p)
        {
            send_rune_pitch_offset = p.as_double();
            RCLCPP_INFO(this->get_logger(), "send_rune_pitch_offset: %f", send_rune_pitch_offset);
        };
        auto send_rune_yaw_offset_callback = [this](const rclcpp::Parameter &p)
        {
            send_rune_yaw_offset = p.as_double();
            RCLCPP_INFO(this->get_logger(), "send_rune_yaw_offset: %f", send_rune_yaw_offset);
        };
        auto send_rune_low_pitch_callback = [this](const rclcpp::Parameter &p)
        {
            send_rune_low_pitch = p.as_double();
            RCLCPP_INFO(this->get_logger(), "send_rune_low_pitch: %f", send_rune_low_pitch);
        };

        auto DEBUG_MODE_callback = [this](const rclcpp::Parameter &p)
        {
            DEBUG_MODE = p.as_bool();
            RCLCPP_INFO(this->get_logger(), "DEBUG_MODE: %d", DEBUG_MODE);
        };
        auto RUNE_CALIB_callback = [this](const rclcpp::Parameter &p)
        {
            RUNE_CALIB = p.as_bool();
            RCLCPP_INFO(this->get_logger(), "RUNE_CALIB: %d", RUNE_CALIB);
        };

        // RCLCPP_INFO(this->get_logger(), "path:%s", g_camera_param_path.c_str());

        this->shoot_delay_cb_handle_ = param_subscriber_->add_parameter_callback("shoot_delay", shoot_delay_callback);
        this->min_shoot_period_cb_handle_ = param_subscriber_->add_parameter_callback("min_shoot_period", min_shoot_period_callback);
        this->min_switch_period_cb_handle_ = param_subscriber_->add_parameter_callback("min_switch_period", min_switch_period_callback);
        this->max_switch_period_cb_handle_ = param_subscriber_->add_parameter_callback("max_switch_period", max_switch_period_callback);
        this->rune_trajectory_k_cb_handle_ = param_subscriber_->add_parameter_callback("rune_trajectory_k", rune_trajectory_k_callback);
        this->send_rune_pitch_offset_cb_handle_ = param_subscriber_->add_parameter_callback("send_rune_pitch_offset", send_rune_pitch_offset_callback);
        this->send_rune_yaw_offset_cb_handle_ = param_subscriber_->add_parameter_callback("send_rune_yaw_offset", send_rune_yaw_offset_callback);
        this->send_rune_low_pitch_cb_handle_ = param_subscriber_->add_parameter_callback("send_rune_low_pitch", send_rune_low_pitch_callback);

        this->DEBUG_MODE_cb_handle_ = param_subscriber_->add_parameter_callback("DEBUG_MODE", DEBUG_MODE_callback);
        this->RUNE_CALIB_cb_handle_ = param_subscriber_->add_parameter_callback("RUNE_CALIB", RUNE_CALIB_callback);
    }

    void RuneFireController::read_callback(const interfaces::msg::Readpack::SharedPtr msg)
    {
        if (this->mode != Mode(msg->mode) || msg->rightclick)
        {
            this->clear();
            this->fire = msg->fired;
            this->rune_request_fire_last = msg->request_fire;
        }
        if (this->mode == Mode::MODE_SMALLRUNE || this->mode == Mode::MODE_BIGRUNE)
        {
            this->last_shooting_time = (this->fire != msg->fired ? this->now().seconds() : this->last_shooting_time); // 如果射击状态改变，更新射击时间
            this->fire = msg->fired;
            this->bullet_speed = msg->bullet_speed;
        }
        this->rune_request_fire = msg->request_fire;
        this->mode = Mode(msg->mode);
    }
    void RuneFireController::timer_callback()
    {
        if (this->mode != Mode::MODE_SMALLRUNE && this->mode != Mode::MODE_BIGRUNE)
        {
            return;
        }
        // CASE 1: 启动新预测周期（首次触发）
        if (!prediction_cycle_active_)
        {
            reset_prediction_cycle();        // 初始化时间戳和迭代次数
            prediction_cycle_active_ = true; // 标记周期激活
            start_single_iteration();        // 启动首轮迭代
        }

        this->sendpack_publisher_->publish(this->send_pack);
    }

    // 初始化
    void RuneFireController::reset_prediction_cycle()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        // 1. 取消所有未完成的客户端请求（如果有异步操作）
        client_->prune_pending_requests(); // ROS2 Client 自带方法

        // 2. 重置状态变量
        current_pred_time_ = this->now() + rclcpp::Duration::from_seconds(this->shoot_delay);
        predTarget_ = Eigen::Vector3d::Zero();
        active_iter_num_ = 0;

        // RCLCPP_INFO(this->get_logger(), "预测周期已安全重置");
    }

    void RuneFireController::start_single_iteration()
    {
        // std::cout << "start single iteration" << std::endl;
        if (active_iter_num_ >= 10)
            return; // 安全保护

        auto request = std::make_shared<interfaces::srv::RuneFire::Request>();
        request->time = current_pred_time_;

        this->client_->async_send_request(request, [this](rclcpp::Client<interfaces::srv::RuneFire>::SharedFuture future)
                                          { this->handle_async_response(future, current_pred_time_); });
        RCLCPP_DEBUG(this->get_logger(), "Request sent");
    }

    void RuneFireController::handle_async_response(
        rclcpp::Client<interfaces::srv::RuneFire>::SharedFuture future,
        rclcpp::Time request_time)
    {
        // ...原有弹道解算和决策逻辑...
        try
        {
            auto response = future.get();
            if (response && response->valid)
            {
                // Step 2: 坐标变换
                Eigen::Vector3d current_euler_angle = {0, 0, 0};
                Eigen::Vector3d target_point{};
                try
                {
                    target_point = process_coordinate_transform(
                        response.get()->target_point,
                        current_euler_angle,
                        request_time);
                }
                catch (const std::exception &e)
                {
                    std::cerr << e.what() << '\n';
                    return;
                }
                double error = (target_point - predTarget_).norm();
                // Step 3: 弹道解算
                auto [euler_angle, flight_time] = calculate_trajectory(
                    target_point,
                    bullet_speed);

                // 更新预测时间（加锁！）
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    current_pred_time_ = this->now() + rclcpp::Duration::from_seconds(flight_time + this->shoot_delay);
                    predTarget_ = target_point;
                    active_iter_num_++;
                    // RCLCPP_INFO(this->get_logger(), "Iter %d ,curr_t:%lf", active_iter_num_, current_pred_time_.seconds());
                    // std::cout << "flight_time:" << flight_time << std::endl;
                }

                // 判断是否继续迭代
                if (error > 0.05 && (active_iter_num_ < 10)) // 检查时间差和迭代次数
                {
                    start_single_iteration(); // 触发下一次请求
                }
                else
                {
                    prediction_cycle_active_ = false; // 结束周期
                    // 发布可视化信息
                    marker.pose.position = tf2::toMsg(target_point);
                    marker_array.markers.push_back(marker);
                    this->visualization_publisher_->publish(marker_array);
                    marker_array.markers.clear();

                    // Step 4: 执行决策
                    make_control_decision(
                        euler_angle,
                        current_euler_angle,
                        response.get()->fire);
                }

                RCLCPP_DEBUG(this->get_logger(), "Response processed, ready for next");
            }
            else
            {
                handle_invalid_response();
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Response error: %s", e.what());
            handle_invalid_response();
        }
    }

    void RuneFireController::handle_invalid_response()
    {
        // std::cout << "Invalid response handling" << std::endl;
        keep_pose();
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            prediction_cycle_active_ = false;
        }
        RCLCPP_DEBUG(this->get_logger(), "Invalid response handling");
    }

    Eigen::Vector3d RuneFireController::process_coordinate_transform(
        const geometry_msgs::msg::PointStamped &target_point,
        Eigen::Vector3d current_euler_angle,
        rclcpp::Time stamp)
    {
        try
        {
            const auto transform = tf_buffer_->lookupTransform(
                "base_link", "rune",
                rclcpp::Time(0), std::chrono::milliseconds(30));
            if (rclcpp::Time(transform.header.stamp) < stamp - rclcpp::Duration(std::chrono::seconds(1)))
            {
                // 最近的变换超过了1s
                RCLCPP_WARN(this->get_logger(), "最近的变换超过了1s");
                throw tf2::TransformException("最近的变换超过了1s");
            }

            const Eigen::Isometry3d isometry = tf2::transformToEigen(transform);
            Eigen::Vector3d target_point_;
            tf2::fromMsg(target_point.point, target_point_);
            current_euler_angle[0] = std::atan2(isometry.rotation()(1, 0), isometry.rotation()(0, 0));
            current_euler_angle[1] = std::asin(-isometry.rotation()(2, 0));
            return isometry * target_point_;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "TF error: %s", ex.what());
            throw; // 向上抛出异常
        }
    }

    std::tuple<Eigen::Vector3d, double>
    RuneFireController::calculate_trajectory(
        const Eigen::Vector3d &target_point,
        double bullet_speed)
    {
        Eigen::Vector3d euler_angle;
        double flight_time = 0;
        // auto start_time = std::chrono::steady_clock::now();

        solver_.getResult(target_point, bullet_speed, euler_angle, flight_time);
        // RCLCPP_INFO(this->get_logger(), "解算结果 yaw=%.2f, pitch=%.2f, flight_time=%.2f", euler_angle.x(), euler_angle.y(), flight_time);
        // auto end_time = std::chrono::steady_clock::now();
        // std::cout << "RuneFireService time:" << std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() << std::endl;

        return {euler_angle, flight_time};
    }

    void RuneFireController::make_control_decision(
        const Eigen::Vector3d &euler_angle,
        const Eigen::Vector3d &current_euler_angle,
        bool should_fire)
    {
        // 角度安全校验
        if (!is_angle_safe(euler_angle, current_euler_angle))
        {
            RCLCPP_WARN(this->get_logger(), "危险角度拒绝执行");
            return;
        }

        // 发送控制指令
        set_send_pack(euler_angle, current_pred_time_);

        // 开火指令
        if (should_fire)
        {
            setRuneFireData();
        }
    }

    void RuneFireController::setRuneFireData()
    {
        double clock_now = get_clock()->now().seconds();

        // std::cout << "fire_answered & shoot" << this->is_fire_answered << "," << this->last_shooting_time << std::endl;

        if (MANUAL_FIRE)
        {
            // 手动开火，无任何限制
            if (this->rune_request_fire != this->rune_request_fire_last)
            {
                this->send_pack.rune_fire = !this->send_pack.rune_fire;
                this->last_send_rune_fire_time = clock_now;
                this->rune_request_fire_last = this->rune_request_fire;
                RCLCPP_INFO(this->get_logger(), "manual send");
            }
            return;
        }

        if (clock_now - std::max(this->last_send_rune_fire_time, this->last_shooting_time) > this->min_shoot_period) // 若满足开火条件
        {
            flag_unfire_reason = unfire_reason::FIRE;
            // 开火位反转
            this->send_pack.rune_fire = !this->send_pack.rune_fire;
            this->last_send_rune_fire_time = clock_now;
            RCLCPP_INFO(this->get_logger(), "send");
        }
        else
        {
            flag_unfire_reason = unfire_reason::HAS_FIRED;
            RCLCPP_DEBUG_STREAM(this->get_logger(), "unfire due to" << this->flag_unfire_reason);
        }
    }

    /**
     * @brief 用于重置预测器
     */
    void RuneFireController::clear()
    {
        this->last_shooting_time = 0.0;
        this->last_send_rune_fire_time = 0.0;
        this->is_fire_answered = false;
        this->fire = false;
        this->flag_unfire_reason = unfire_reason::CANNOT_FIRE;
        this->send_pack.rune_fire = false;
    }

    void RuneFireController::set_send_pack(Eigen::Vector3d euler_angle, rclcpp::Time t)
    {
        this->send_pack.mode = this->mode;
        // bag数据测试使用target.clock
        rclcpp::Time clock_now = get_clock()->now();

        double palstance = (euler_angle[1] * 180 / M_PI - this->send_pack.pred_pitch) / (t - clock_now).seconds();
        if (fabs(palstance) < 2 && fabs(palstance) > 1e-3)
        {
            this->send_pack.pitch_palstance = palstance;
        }
        // 计算失败，则不变

        palstance = (euler_angle[0] * 180 / M_PI - this->send_pack.pred_yaw) / (t - clock_now).seconds();
        if (fabs(palstance) < 2 && fabs(palstance) > 1e-3)
        {
            this->send_pack.yaw_palstance = palstance;
        }
        // 计算失败，则不变
        this->send_pack.pred_yaw = euler_angle[0] * 180 / M_PI;
        this->send_pack.pred_pitch = euler_angle[1] * 180 / M_PI;

        // send_pack.pred_yaw += send_rune_yaw_offset;
        // send_pack.pred_pitch += send_rune_pitch_offset + (send_pack.pred_pitch - 4.21971) * rune_trajectory_k - (send_pack.pred_pitch + raw_rune_low_pitch) * rune_k;
    }

    void RuneFireController::keep_pose()
    {
        this->send_pack.pred_yaw = this->read_pack.pose.ptz_yaw;
        this->send_pack.pred_pitch = this->read_pack.pose.ptz_pitch;
    }

} // namespace rune

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rune::RuneFireController)
