#include "rune_parameters.hpp"
#include "rune_modeling.hpp"
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <fstream>

namespace rune
{

    RuneModeling::RuneModeling(const rclcpp::NodeOptions &options) : Node("rune_modeling", options)
    {
        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        loadParam();
        fanArmors_sub = this->create_subscription<interfaces::msg::FanArmors>("FanArmors", 10, std::bind(&RuneModeling::fanArmorsCallback, this, std::placeholders::_1));
        readPack_sub = this->create_subscription<interfaces::msg::Readpack>("Readpack", 10, std::bind(&RuneModeling::readPackCallback, this, std::placeholders::_1));
        if (!RUNE_CALIB)
        {
            timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&RuneModeling::timerCallback, this));
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(), std::chrono::seconds(30));
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }
        runeFire_srv_ = this->create_service<interfaces::srv::RuneFire>("RuneFire", std::bind(&RuneModeling::RuneFireService, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "%s init.", "rune_modeling");

        // Initialize options for ceres solver
    }

    RuneModeling::~RuneModeling() {}

    void RuneModeling::loadParam()
    {
        // 调试模式
        this->declare_parameter("RUNE_CALIB", false);
        this->RUNE_CALIB = this->get_parameter("RUNE_CALIB").as_bool();
        this->declare_parameter("DEBUG_MODE", false);
        this->DEBUG_MODE = this->get_parameter("DEBUG_MODE").as_bool();
        this->declare_parameter("use_sim", false);
        this->use_sim = this->get_parameter("use_sim").as_bool();
        // 最小切换间隔
        this->declare_parameter("min_switch_period", 0.2);
        this->min_switch_period = this->get_parameter("min_switch_period").as_double();
        // 最大切换间隔
        this->declare_parameter("max_switch_period", 1.8);
        this->max_switch_period = this->get_parameter("max_switch_period").as_double();

        // // 能量机关接收pitch offset
        // this->declare_parameter("receive_rune_pitch_offset", 0.0);
        // // 能量机关接收yaw offset
        // this->declare_parameter("receive_rune_yaw_offset", 0.0);
        // // 能量机关接收roll offset
        // this->declare_parameter("receive_rune_roll_offset", 0.0);
        // // 能量机关最低点的pitch值
        // this->declare_parameter("raw_rune_low_pitch", 0.0);
        // // 能量机关陀螺仪增量误差系数k
        // this->declare_parameter("rune_k", 0.0);

        // receive_rune_pitch_offset = this->get_parameter("receive_rune_pitch_offset").as_double();
        // receive_rune_yaw_offset = this->get_parameter("receive_rune_yaw_offset").as_double();
        // receive_rune_roll_offset = this->get_parameter("receive_rune_roll_offset").as_double();
        // raw_rune_low_pitch = this->get_parameter("raw_rune_low_pitch").as_double();
        // rune_k = this->get_parameter("rune_k").as_double();

        // auto receive_rune_pitch_offset_callback = [this](const rclcpp::Parameter &p)
        // {
        //     receive_rune_pitch_offset = p.as_double();
        //     RCLCPP_INFO(this->get_logger(), "receive_rune_pitch_offset: %f", receive_rune_pitch_offset);
        // };
        // auto receive_rune_yaw_offset_callback = [this](const rclcpp::Parameter &p)
        // {
        //     receive_rune_yaw_offset = p.as_double();
        //     RCLCPP_INFO(this->get_logger(), "receive_rune_yaw_offset: %f", receive_rune_yaw_offset);
        // };
        // auto receive_rune_roll_offset_callback = [this](const rclcpp::Parameter &p)
        // {
        //     receive_rune_roll_offset = p.as_double();
        //     RCLCPP_INFO(this->get_logger(), "receive_rune_roll_offset: %f", receive_rune_roll_offset);
        // };
        // auto raw_rune_low_pitch_callback = [this](const rclcpp::Parameter &p)
        // {
        //     raw_rune_low_pitch = p.as_double();
        //     RCLCPP_INFO(this->get_logger(), "raw_rune_low_pitch: %f", raw_rune_low_pitch);
        // };
        // auto rune_k_callback = [this](const rclcpp::Parameter &p)
        // {
        //     rune_k = p.as_double();
        //     RCLCPP_INFO(this->get_logger(), "rune_k: %f", rune_k);
        // };
        // this->raw_rune_low_pitch_cb_handle_ = param_subscriber_->add_parameter_callback("raw_rune_low_pitch", raw_rune_low_pitch_callback);
        // this->rune_k_cb_handle_ = param_subscriber_->add_parameter_callback("rune_k", rune_k_callback);

        // this->receive_rune_pitch_offset_cb_handle_ = param_subscriber_->add_parameter_callback("receive_rune_pitch_offset", receive_rune_pitch_offset_callback);
        // this->receive_rune_yaw_offset_cb_handle_ = param_subscriber_->add_parameter_callback("receive_rune_yaw_offset", receive_rune_yaw_offset_callback);
        // this->receive_rune_roll_offset_cb_handle_ = param_subscriber_->add_parameter_callback("receive_rune_roll_offset", receive_rune_roll_offset_callback);
    }
    void RuneModeling::readPackCallback(const interfaces::msg::Readpack::SharedPtr msg)
    {
        if (this->mode != rune::Mode(msg->mode))
        {
            // 切换模式重置
            this->clear();
        }

        this->mode = rune::Mode(msg->mode);
    }
    void RuneModeling::clear()
    {
        flag_all_activate = false;
        flag_detect_success = false;
        use_sim = false;
        RUNE_CALIB = false;
        DEBUG_MODE = false;
        fit_valid = false;
        time_start = 0;
        fan_armor_cache_.clear();
        rune_cache_.clear();
        is_fitting_ = false;
        // 重置,等待任务完成
        try
        {
            fitting_future_.get();
        }
        catch (const std::exception &e)
        {
            std::cerr << "Caught exception again: " << e.what() << std::endl;
        }
        modeling_param_ = {0.470, 1.942, 0, 1.178, 0};
        min_error = 10000.0;
    }

    interfaces::msg::FanArmors RuneModeling::transform_fanArmors(const interfaces::msg::FanArmors &msg, const Eigen::Isometry3d &T)
    {
        interfaces::msg::FanArmors new_msg{};
        for (auto &armor : msg.fan_armors)
        {
            Eigen::Vector3d armor_pos, r_pos;
            tf2::fromMsg(armor.armor_center.point, armor_pos);
            tf2::fromMsg(armor.r_center.point, r_pos);
            armor_pos = T * armor_pos;
            r_pos = T * r_pos;
            interfaces::msg::FanArmor fan_armor;
            fan_armor.armor_center.point = tf2::toMsg(armor_pos);
            fan_armor.armor_center.header = armor.armor_center.header;
            fan_armor.r_center.point = tf2::toMsg(r_pos);
            fan_armor.r_center.header = armor.r_center.header;
            new_msg.fan_armors.push_back(fan_armor);
        }
        return new_msg;
    }

    void RuneModeling::fanArmorsCallback(const interfaces::msg::FanArmors::SharedPtr msg)
    {
        // 存入buffer
        interfaces::msg::FanArmors new_msg = *msg;
        fan_armor_cache_.push(new_msg);
    }

    void RuneModeling::timerCallback()
    {
        // std::cout << "timerCallback1" << std::endl;
        // ================== 第一阶段：查找是否已经拟合得到目标平面 ==================
        geometry_msgs::msg::TransformStamped t;

        // Look up for the transformation between target_frame and turtle2 frames
        // and send velocity commands for turtle2 to reach target_frame
        try
        {

            t = tf_buffer_->lookupTransform(
                "rune", "base_link",
                rclcpp::Time(0),
                std::chrono::milliseconds(30));
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", "rune", "base_link", ex.what());
            return;
        }
        world_to_rune_transform_ = tf2::transformToEigen(t);

        // ================== 第二阶段：获得快照，对快照中的数据进行处理，处理完成后存入cache ==================
        // std::cout << "timerCallback2" << std::endl;

        auto fan_armors_snap = fan_armor_cache_.snapshot();
        auto rune_snap = rune_cache_.snapshot();

        auto last_fan_time_stamp_ = rclcpp::Time(fan_armors_snap.back()->fan_armors.begin()->armor_center.header.stamp);

        // 获取rune最新时间戳（若无rune则所有fan都需要处理）
        double last_rune_time = rune_snap.empty() ? -INFINITY : rune_snap.back()->time_stamp;

        // 找到第一个时间戳 > last_rune_time + EPSILON的fan消息
        auto it = std::upper_bound(fan_armors_snap.begin(), fan_armors_snap.end(), last_rune_time,
                                   [&](double time, const auto &msg)
                                   {
                                       return time + 1e-6 < rclcpp::Time(msg->fan_armors.begin()->armor_center.header.stamp).seconds();
                                   });
        // 如果it == end，说明所有消息的时间戳都小于等于last_rune_time + EPSILON，则需要看看是否时间戳离现在的时间太远了。
        if (it == fan_armors_snap.end() && !rune_snap.empty())
        {
            if (last_fan_time_stamp_ - get_clock()->now() < rclcpp::Duration::from_seconds(0.025))
            {
                // 不足25ms,认为可以忍受，jump
                return;
            }
            else
            {
                // 说明很久没有更新，需要手动插值以保证数据连续性
                Rune result(*(rune_cache_.get_latest().get()));
                result.no_detect_interpid(this->modeling_param_, this->fit_valid);
                rune_cache_.push(result);
            }
        }

        // 仅遍历[it, end)区间
        for (; it != fan_armors_snap.end(); ++it)
        {
            auto &msg = *it;
            //  保留时间有效性检查
            //  if (rclcpp::Time(msg->fan_armors.begin()->armor_center.header.stamp) - now() < rclcpp::Duration::from_seconds(0.002))
            //  {
            //      continue; // 或break取决于业务逻辑
            //  }

            // 进行坐标系变化
            interfaces::msg::FanArmors new_msg = transform_fanArmors(*msg, world_to_rune_transform_);

            // 首先检查是否为空，为空则初始化
            if (rune_snap.empty())
            {
                try
                {
                    Rune rune(new_msg);
                    rune_cache_.push(rune);
                    rune_snap = rune_cache_.snapshot();
                    continue;
                }
                catch (std::out_of_range &e)
                {
                    // 检测到的数量不为1,不符合初始化的条件
                    return;
                }
            }
            // 处理消息
            try
            {
                auto rune = handle_fanArmors(new_msg, *(rune_cache_.get_latest().get()));
                // out_file << std::to_string(rune.get_angle_data().first) << "," << std::to_string(rune.get_angle_data().second) << std::endl;
                // out_file.close();
                rune_cache_.push(rune);
            }
            catch (const std::logic_error &e)
            {
                // 逻辑错误，可以继续下去
                RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
                // 此时需要插值
                Rune result(*(rune_cache_.get_latest().get()));
                result.no_detect_interpid(this->modeling_param_, this->fit_valid);
                rune_cache_.push(result);
            }
            catch (const std::runtime_error &e)
            {
                // 运行时错误，无法继续，需要重置
                RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
                this->clear();
                return;
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
                return;
            }
        }

        // ================== 第三阶段：检查cache数量，够了就拟合 ==================
        // std::cout << "timerCallback3" << std::endl;

        rune_snap = rune_cache_.snapshot();
        if (rune_snap.size() < 240)
        {
            return;
        }

        // 数量够了，拟合
        // std::cout << "timerCallback4" << std::endl;

        this->trigger_angle_fitting();
    }

    Rune RuneModeling::handle_fanArmors(const interfaces::msg::FanArmors &fan_armors, const Rune &last_rune)
    {
        Rune result(last_rune);
        std::vector<Fan> detect_fans;
        for (auto fan_armor : fan_armors.fan_armors)
        {
            Fan tep_fan;
            tep_fan.set_position(fan_armor);
            detect_fans.push_back(tep_fan);
        }
        result.detect_result_to_fan(detect_fans);
        result.time_stamp = rclcpp::Time(fan_armors.fan_armors.begin()->armor_center.header.stamp).seconds();
        return result;
    }

    void RuneModeling::trigger_angle_fitting()
    {
        // 第一层快速检查（无锁）
        if (is_fitting_.load(std::memory_order_acquire))
        {
            RCLCPP_DEBUG(this->get_logger(), "[Fast Check] Fitting in progress");
            return;
        }

        // 第二层精确检查（带锁）
        std::lock_guard<std::mutex> lock(fitting_mutex_);

        // 双重检查锁定模式
        if (is_fitting_.load(std::memory_order_relaxed))
        {
            RCLCPP_DEBUG(this->get_logger(), "[Accurate Check] Fitting in progress");
            return;
        }

        // 节流控制检查
        const auto now = std::chrono::steady_clock::now();
        if (now - last_fit_time_ < std::chrono::milliseconds(MIN_FIT_INTERVAL))
        {
            RCLCPP_DEBUG(this->get_logger(), "Fitting throttled. Elapsed: %ldms",
                         std::chrono::duration_cast<std::chrono::milliseconds>(
                             now - last_fit_time_)
                             .count());
            return;
        }

        try
        {
            // 标记拟合开始
            is_fitting_.store(true, std::memory_order_release);

            // 启动异步任务
            fitting_future_ = std::async(std::launch::async, [this]()
                                         {
            try {
                fit_angle_task();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(),
                           "Fitting task exception: %s", e.what());
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(),
                           "Unknown exception in fitting task");
            }
            finalize_fitting(); });

            RCLCPP_DEBUG(this->get_logger(),
                         "New fitting task started successfully");
        }
        catch (const std::system_error &e)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to start async task: %s", e.what());
            is_fitting_.store(false, std::memory_order_release);
        }
    }

    void RuneModeling::fit_angle_task()
    {
        // 实际的拟合算法实现
        // RCLCPP_INFO(this->get_logger(), "Starting angle fitting...");

        // 取一份快照
        auto snapshot = rune_cache_.snapshot();

        // 1. 平面拟合阶段
        std::vector<rune::optimization::CurveFittingProblem::DataPoint> data_points;

        for (auto &ptr : snapshot)
        {
            data_points.push_back(ptr->get_angle_data());
        }

        // 执行带约束的平面拟合
        std::vector<rune::optimization::CurveFittingProblem::DataPoint> final_consensus_set;
        auto angle_model = this->angle_fitting_.run(
            data_points,
            problem_,
            rune::optimization::RandomPointSampler{},
            rune::optimization::MaxIterationsOrErrorThresholdTermination(1000, 0.5),
            &final_consensus_set);
        std::cout << "angle_model:" << "a:" << angle_model[0] * angle_model[1] << "w:" << angle_model[1] << "t0:" << angle_model[2] << "b:" << angle_model[3] << "phi:" << angle_model[4] << std::endl;
        // 更新模型参数
        if (angle_model != rune::optimization::CurveFittingProblem::Model())
        {
            // 计算模型参数
            // std::cout << "angle_model:" << "a/w:" << angle_model[0] << "w:" << angle_model[1] << "t0:" << angle_model[2] << "b:" << angle_model[3] << "phi:" << angle_model[4] << std::endl;
            double error = 0.0;
            for (auto &pt : final_consensus_set)
            {
                error += problem_.computeError(pt, angle_model);
            }
            error /= final_consensus_set.size();
            std::cout << "error: " << error << std::endl;
            if (error < min_error)
            {
                min_error = error;
                modeling_param_ = angle_model;
            }
            if (error > 0.025)
            {
                RCLCPP_INFO(this->get_logger(), "Angel fitting failed");
                this->fit_valid = false;
                return;
            }
            this->fit_valid = true;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Angle fitting failed");
            this->fit_valid = false;
            return;
        }

        // RCLCPP_DEBUG(this->get_logger(), "center(%lf,%lf,%lf),vec(%lf,%lf),r=%lf,t=%lf", C_data[0], C_data[1], C_data[2], norm_vector_data[0], norm_vector_data[1], this->rune_model.radius, summary.total_time_in_seconds);

        // RCLCPP_INFO(this->get_logger(), "Angel fitting completed");
    }

    void RuneModeling::finalize_fitting()
    {
        std::lock_guard<std::mutex> lock(fitting_mutex_);

        // 更新状态和时间戳
        is_fitting_.store(false, std::memory_order_release);
        last_fit_time_ = std::chrono::steady_clock::now();

        // 此处添加结果处理逻辑
        // 例如：发布拟合结果、更新模型等
        RCLCPP_DEBUG(this->get_logger(),
                     "Fitting finalized at %ldms",
                     std::chrono::duration_cast<std::chrono::milliseconds>(
                         last_fit_time_.time_since_epoch())
                         .count());
    }
    void RuneModeling::RuneFireService(const interfaces::srv::RuneFire_Request::SharedPtr request, interfaces::srv::RuneFire_Response::SharedPtr response)
    {
        if (!this->fit_valid)
        {
            response->valid = false;
            return;
        }
        response->valid = true;
        double target_time = rclcpp::Time(request->time).seconds();
        auto last_rune = rune_cache_.get_latest();
        // printf("target_time:%lf,%lf\n", target_time, target_time - last_rune->time_stamp);
        auto tep = last_rune->get_angle_data(last_rune->inactive_index);
        double angle_diff = tep.second;
        if (this->mode == Mode::MODE_SMALLRUNE)
        {
            angle_diff = CONST_PALSTANCE * std::copysign(1, last_rune->dir) * (target_time - last_rune->time_stamp);
        }
        else if (this->mode == Mode::MODE_BIGRUNE)
        {
            angle_diff = std::copysign(1, last_rune->dir) * (rune::optimization::CurveFittingProblem::getAngleBig(last_rune->time_stamp - last_rune->start_time_stamp, target_time - last_rune->start_time_stamp, this->modeling_param_));
        }
        // std::cout << "delta_angle: " << angle_diff * 180. / M_PI << ",pred_angle:" << last_rune->get_angle_data().second + last_rune->angle_start + angle_diff << std::endl;
        if (this->RUNE_CALIB)
        {
            angle_diff = 0; // 直瞄
        }

        response->target_point.point = tf2::toMsg(last_rune->get_target_position(angle_diff));
        response->target_point.header.frame_id = "rune";
        response->target_point.header.stamp = request->time;
        response->fire = judgeFire(target_time, last_rune->get_last_switch_time());
    }

    /**
     * @brief 用于判断当前是否要反转开火标志位
     * @param target_time
     * @param last_fan_switched_time
     * @return
     */
    bool RuneModeling::judgeFire(double target_time, double last_fan_switched_time)
    {
        // double clock_now = get_clock()->now().seconds();
        // RCLCPP_INFO(this->get_logger(), "switch:%lf,shoot:%lf", this->rune_model.last_fan_switched_time,this->last_shooting_time);
        // RCLCPP_INFO(this->get_logger(), "last:%lf", this->last_shooting_time);

        if (target_time - last_fan_switched_time > this->max_switch_period)
        {
            RCLCPP_DEBUG_STREAM(this->get_logger(), "cannot fire, time out");
            return false;
        }
        if (target_time - last_fan_switched_time < this->min_switch_period)
        {
            RCLCPP_DEBUG_STREAM(this->get_logger(), "cannot fire, no enough time");
            return false;
        }
        return true;
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rune::RuneModeling)