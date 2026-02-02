#include "utils/rune_fan_defination.hpp"
#include <iostream>
#include <rclcpp/rclcpp.hpp>

namespace rune
{

    std::ostream &operator<<(std::ostream &strm, fan_status fs)
    {
        switch (fs)
        {
        case fan_status::DARK:
            return strm << "DARK";
        case fan_status::INACTIVATED:
            return strm << "INACTIVATED";
        case fan_status::ACTIVATED:
            return strm << "ACTIVATED";
        default:
            return strm << "UNKNOWN";
        }
    }

    Fan::Fan() : StateMachine(fan_status::DARK)
    {
        this->id = -1;
        this->rotation_center = Eigen::Vector3d(0, 0, 0);
        this->angle = 0;
        this->last_switch_time = rclcpp::Clock().now().seconds();

        target_status = fan_status::DARK;
    }

    Fan &Fan::operator=(const Fan &f)
    {
        if (this != &f) // 防止自赋值
        {
            this->id = f.id;
            this->armor_center = f.armor_center;
            this->rotation_center = f.rotation_center;
            this->rotate_axis = f.rotate_axis;
            this->angle = f.angle;
            this->switch_cnt = f.switch_cnt;
            this->last_switch_time = f.last_switch_time;
            this->last_detect_count_ = f.last_detect_count_;
            this->is_detected_last_ = f.is_detected_last_;
            this->target_status = f.target_status;
            this->current_state_ = f.current_state_;
            this->previous_state_ = f.previous_state_;
        }
        return *this;
    }

    void Fan::Update_state()
    {
        // 自动递增计数器（与目标状态无关）
        if (target_status != current_state_)
        {
            // std::cout << id << "switch_cnt: " << switch_cnt << std::endl;
            switch_cnt = std::min(switch_cnt + 1, switch_cnt_threshold + 1);

            // 触发转换条件
            if (switch_cnt >= switch_cnt_threshold)
            {
                transition_to(target_status);
                last_switch_time = rclcpp::Clock().now().seconds();
                // std::cout << id << "switch_cnt_change: " << switch_cnt << std::endl;
                switch_cnt = 0;
            }
        }
        else
        {
            switch_cnt = std::max(0, switch_cnt - 2); // 快速稳定
        }
    }

    void Fan::transition_to(fan_status new_state)
    {
        fan_status tep = current_state_;
        before_transition(tep);
        current_state_ = new_state;
        after_transition(tep);
    }

    void Fan::before_transition(fan_status old_state)
    {
        // std::cout << "Transitioning from: " << static_cast<int>(current_state_) << std::endl;
    }

    void Fan::after_transition(fan_status old_state)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rune_modeling"),
                     "fan %d State changed from %d to %d (switch_cnt=%d)",
                     id,
                     old_state,
                     current_state_,
                     switch_cnt);
    }

    // 核心旋转函数
    void Fan::rotate(double angle_rad, const Eigen::Vector3d &rotate_axis)
    {
        if (this->id == -1)
        {
            // RCLCPP_ERROR(rclcpp::get_logger("rune_modeling"), "uninitialized fan object");
            return;
        }

        // 创建旋转变换
        Eigen::AngleAxisd rotation(angle_rad, rotate_axis.normalized());
        Eigen::Matrix3d rotate_mat = rotation.toRotationMatrix();

        // 对每个特征点进行旋转
        Eigen::Vector3d relative_vec = armor_center - this->rotation_center;
        relative_vec = rotate_mat * relative_vec;
        armor_center = this->rotation_center + relative_vec;
        this->angle += angle_rad;
    }
    void Fan::rotate(double angle_rad)
    {
        this->rotate(angle_rad, this->rotate_axis);
    }
    void Fan::set_position(const interfaces::msg::FanArmor &armor)
    {
        tf2::fromMsg(armor.armor_center.point, this->armor_center);
        tf2::fromMsg(armor.r_center.point, this->rotation_center);
        this->angle = fmod(2 * M_PI + atan2(armor_center.z() - rotation_center.z(), armor_center.y() - rotation_center.y()), 2 * M_PI);
    }

    bool Fan::is_same_fan(Fan &another_fan)
    {
        if ((this->armor_center - another_fan.armor_center).norm() < 0.2)
        {
            return true;
        }
        else
        {
            double delta_angle = fabs(fmod(this->angle - another_fan.angle, 2 * M_PI));
            delta_angle = std::min(delta_angle, 2 * M_PI - delta_angle);
            return delta_angle < M_PI / 10;
        }
    }

    std::optional<double> Fan::fan_angle_diff(Fan &another_fan, double angle_threshold)
    {
        double tep_delta_angle = fmod(this->angle - another_fan.angle, 2 * M_PI);
        if (fabs(tep_delta_angle) > angle_threshold)
        {
            if (fabs(fabs(this->angle) - 2 * M_PI) < angle_threshold)
            {
                tep_delta_angle += (tep_delta_angle < 0) ? 2 * M_PI : -2 * M_PI;
            }
            else if (fabs(fabs(tep_delta_angle) - 2 * M_PI) < angle_threshold)
            {
                tep_delta_angle -= std::copysign(2 * M_PI, tep_delta_angle);
            }
            else
            {
                return std::nullopt;
            }
        }
        return tep_delta_angle;
    }

    // 统一处理事件
    bool Fan::handle_event(FanEvent event)
    {
        // 计算目标状态
        fan_status target = calculate_target_status(event);

        // 触发状态转换
        this->target_status = target;

        // 错误检测逻辑（外部循环中）
        if (target == current_state_)
        {
            // 状态未变化，但可能需记录错误（如 ACTIVATED 状态未被检测到）
            if (current_state_ == fan_status::ACTIVATED && !(event.detection_status == FanDetectionStatus::DETECTED))
            {
                RCLCPP_ERROR(rclcpp::get_logger("rune_modeling"), "active failed: stable but not detected");
                return false;
            }
            else if (event.count_status == FanCountStatus::COUNT_DECREASE)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rune_modeling"), " count decrease,but not one");
                return false;
            }
        }
        return true;
    }

    FanEvent Fan::generate_event(bool is_detected_current, int detect_count_current)
    {
        // 生成基础事件（是否被检测到）
        FanDetectionStatus base_event = is_detected_current ? FanDetectionStatus::DETECTED : FanDetectionStatus::NOT_DETECTED;

        // 生成数量变化事件
        FanCountStatus count_event = [this](int detect_count_current) -> FanCountStatus
        {
            return (detect_count_current > last_detect_count_) ? FanCountStatus::COUNT_INCREASE : (detect_count_current < last_detect_count_ ? FanCountStatus::COUNT_DECREASE : FanCountStatus::COUNT_STABLE);
        }(detect_count_current);

        if (detect_count_current == 1)
        {
            count_event = FanCountStatus::COUNT_ONE;
        }

        return FanEvent{base_event, count_event};
    }

    fan_status Fan::calculate_target_status(FanEvent event)
    {
        const auto current = current_state_;

        switch (current)
        {
        //---------------------------------------------------------------------
        // 当前状态：DARK
        //---------------------------------------------------------------------
        case fan_status::DARK:
            // 规则：若被检测到，则转换到 INACTIVATED；否则保持 DARK
            return event.detection_status == FanDetectionStatus::DETECTED ? fan_status::INACTIVATED : fan_status::DARK;

        //---------------------------------------------------------------------
        // 当前状态：INACTIVATED
        //---------------------------------------------------------------------
        case fan_status::INACTIVATED:
            // 检测数量增加且被检测到 → ACTIVATED
            if (event.count_status == FanCountStatus::COUNT_INCREASE && event.detection_status == FanDetectionStatus::DETECTED)
            {
                return fan_status::ACTIVATED;
            }
            // 检测数量稳定 → 根据是否被检测到决策
            else if (event.count_status == FanCountStatus::COUNT_STABLE)
            {
                // 先确认target和curr是否一致？
                // if (this->target_status == fan_status::ACTIVATED)
                // {
                //     // 说明上一帧检测到了数量增加，这帧稳定，所以继续给ACTIVATED
                //     return this->current_state_;
                // }
                // target不是ACTIVATED，说明正常的稳定，所以是没换扇叶保持INACTIVATED或者换扇叶成为DARK
                return event.detection_status == FanDetectionStatus::DETECTED ? fan_status::INACTIVATED : fan_status::DARK;
            }
            // 检测数量减少 → 根据最终数量决策
            else if (event.count_status == FanCountStatus::COUNT_DECREASE)
            {
                // 未减少到 1 → 保持 INACTIVATED（外部需记录错误）
                return fan_status::INACTIVATED;
            }
            // 减少到 1 → 根据是否被检测到决策
            else if (event.count_status == FanCountStatus::COUNT_ONE)
            {
                return event.detection_status == FanDetectionStatus::DETECTED ? fan_status::INACTIVATED : fan_status::DARK;
            }
            break;

        //---------------------------------------------------------------------
        // 当前状态：ACTIVATED
        //---------------------------------------------------------------------
        case fan_status::ACTIVATED:
            // 检测数量稳定或增加 → 保持 ACTIVATED（外部需检查是否被检测到）
            if (event.count_status == FanCountStatus::COUNT_INCREASE || event.count_status == FanCountStatus::COUNT_STABLE)
            {
                return fan_status::ACTIVATED;
            }
            // 检测数量减少 → 根据最终数量决策
            else
            {
                // 减少到 1 → 根据是否被检测到决策（generate_event已经更新，因此last_detect_count_为现在检测到的数量）
                if (event.count_status == FanCountStatus::COUNT_ONE)
                {
                    return event.detection_status == FanDetectionStatus::DETECTED ? fan_status::INACTIVATED : fan_status::DARK;
                }
                // 未减少到 1 → 保持 ACTIVATED（外部需记录错误）
                else
                {
                    return fan_status::ACTIVATED;
                }
            }
            break;
        }

        // 默认保持当前状态
        return current;
    }

    void Fan::Update_history_variance(bool is_detected_current, int detect_count_current)
    {
        if ((this->current_state_ == this->target_status) ^ (last_detect_count_ == detect_count_current))
        {
            // 两种情况可以更新，状态不变但是数量变了，或者状态变了但是数量没变
            is_detected_last_ = is_detected_current;
            last_detect_count_ = detect_count_current;
        }
    }
    //--------------------------------------------------------------------

    Rune::Rune()
    {
        this->modeling_param_ = {0.470, 1.942, 0, 1.178, 0};
        this->time_stamp = rclcpp::Clock().now().seconds();
        this->dir = 0;
        this->inactive_index = -2;
        this->current_detect_fans_size = 0;
    }

    void Rune::clear()
    {
        this->modeling_param_ = {0.470, 1.942, 0, 1.178, 0};
        this->time_stamp = rclcpp::Clock().now().seconds();
        this->start_time_stamp = this->time_stamp;
        this->dir = 0;
        this->inactive_index = -2;
        this->current_detect_fans_size = 0;
        this->fans = std::array<Fan, 5>{}; // 初始化为空
        this->failed_detect_count = 0;
    }
    Rune::Rune(const interfaces::msg::FanArmors &msg)
    {
        std::cout << "construct" << std::endl;

        // 先检查是否只有一个fan
        if (msg.fan_armors.size() != 1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rune_modeling"), "fan_armors size is %d,not 1", msg.fan_armors.size());
            throw std::out_of_range("fan_armors size is not 1");
        }

        // 第一个扇叶，初始化为待激活扇叶
        this->inactive_index = 0;
        this->fans.at(0).initialize_as_inactive();
        this->fans.at(0).is_detected_last_ = true;
        this->fans.at(0).last_event.detection_status = FanDetectionStatus::DETECTED;

        // 给其他扇叶更新位置
        for (int i = 0; i < this->fans.size(); i++)
        {
            this->fans.at(i).initialize_id(i);
            this->fans.at(i).set_position(msg.fan_armors.at(0));
            this->fans.at(i).rotate(i * 2.0 * M_PI / 5.0);
            this->fans.at(i).last_detect_count_ = 1;
        }
        RCLCPP_INFO(rclcpp::get_logger("rune_modeling"), "initialize from msg");
        this->current_detect_fans_size = msg.fan_armors.size();
        this->time_stamp = rclcpp::Time(msg.fan_armors.at(0).armor_center.header.stamp).seconds();
        this->start_time_stamp = this->time_stamp;
        this->angle_start = std::atan2(msg.fan_armors.at(0).armor_center.point.y, msg.fan_armors.at(0).armor_center.point.x);
        std::cout << "finish construct:" << this->time_stamp << std::endl;
    }
    // copy构造函数
    Rune::Rune(const Rune &other)
    {
        this->time_stamp = other.time_stamp;
        this->start_time_stamp = other.start_time_stamp;
        this->dir = other.dir;
        this->inactive_index = other.inactive_index;
        this->current_detect_fans_size = other.current_detect_fans_size;
        // 深拷贝 fans 数组
        for (size_t i = 0; i < fans.size(); ++i)
        {
            this->fans[i] = other.fans[i]; // 确保 Fan 类有合适的拷贝构造函数
        }
        this->modeling_param_ = other.modeling_param_;

        this->delta_angle = other.delta_angle;
        this->failed_detect_count = other.failed_detect_count;
        this->flag_r_detected = other.flag_r_detected;
    }

    void Rune::extract_info(std::vector<Fan> &detect_fans)
    {
        this->delta_angle = 0;          // 重置变化角度
        this->detected_fans_id.clear(); // 清空上次检测到的扇叶id
        for (auto fan_ : detect_fans)
        {
            for (int i = 0; i < this->fans.size(); i++)
            {
                if (this->fans.at(i).is_same_fan(fan_))
                {
                    // std::cout << "find same fan:" << i << std::endl;
                    // 记录匹配到的index
                    auto insert_result = detected_fans_id.insert(i);
                    if (!insert_result.second)
                    {
                        // 已经有相同的id了，这是一个问题，输出
                        RCLCPP_ERROR(rclcpp::get_logger("rune_modeling"), "same id detected:%d", i);
                        continue;
                    }

                    // 记录变化的angle
                    auto tep_delta_angle = fan_.fan_angle_diff(this->fans.at(i), M_PI / 5);
                    this->delta_angle += tep_delta_angle.value_or(0.0);

                    break;
                }
            }
        }

        if (detected_fans_id.size() != detect_fans.size()) // 检测到的扇叶数量不匹配
        {
            RCLCPP_ERROR(rclcpp::get_logger("rune_modeling"), "fan match failed, indexs.size:%d,detect_fans.size:%d", detected_fans_id.size(), detect_fans.size());
            for (auto fan_ : this->fans)
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rune_modeling"), " " << fan_.get_id() << ":" << fan_.get_vector().transpose() << " " << fan_.angle / M_PI * 180.);
            }

            RCLCPP_ERROR(rclcpp::get_logger("rune_modeling"), "detect_fans");
            for (auto fan_ : detect_fans)
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rune_modeling"), fan_.get_vector().transpose() << " " << fan_.angle / M_PI * 180.);
            }
            throw std::logic_error("fan match failed"); // 抛出异常
        }

        this->detected_fans_id = edge_debouncer.update(this->detected_fans_id);
        this->delta_angle /= this->detected_fans_id.size();
        this->dir += std::copysign(1, this->delta_angle);
    }

    void Rune::update_fans_status()
    {
        int detect_fans_size = this->detected_fans_id.size(); // 减少计算量
        // 遍历所有扇叶进行状态更新
        for (int i = 0; i < this->fans.size(); i++)
        {
            bool is_in = this->detected_fans_id.find(i) != this->detected_fans_id.end();
            // 2. 自动生成事件（内部处理历史状态对比）
            FanEvent event = this->fans.at(i).generate_event(is_in, detect_fans_size);
            // std::cout << "event:" << static_cast<int>(event.detection_status) << " " << static_cast<int>(event.count_status) << std::endl;
            // 3. 统一处理事件
            if (!this->fans.at(i).handle_event(event))
            {
                this->failed_detect_count++;
            }
            // 4. 状态机驱动
            this->fans.at(i).Update_state();
            // 5. 更新扇叶内部变量历史数据
            this->fans.at(i).Update_history_variance(is_in, detect_fans_size);
        }
    }

    void Rune::detect_result_to_fan(std::vector<Fan> &detect_fans)
    {
        int detect_fans_size = detect_fans.size(); // 减少计算量

        this->extract_info(detect_fans); // 提取信息

        this->rotate_fans(this->delta_angle); // 旋转扇叶

        this->update_fans_status(); // 更新状态

        this->self_examination(); // 自检

        // RCLCPP_INFO_STREAM(rclcpp::get_logger("rune_modeling"), this->fans.at(0).get_id() << ":" << this->fans.at(0).get_vector().transpose() << " " << this->fans.at(0).current_state() << " " << this->fans.at(0).get_target_status() << " " << this->fans.at(0).get_switch_cnt() << " " << this->fans.at(0).last_detect_count_ << " " << this->fans.at(0).is_detected_last_ << " "
        //                                                                                   << this->fans.at(1).get_id() << ":" << this->fans.at(1).get_vector().transpose() << " " << this->fans.at(1).current_state() << " " << this->fans.at(1).get_target_status() << " " << this->fans.at(1).get_switch_cnt() << " " << this->fans.at(1).last_detect_count_ << " " << this->fans.at(1).is_detected_last_ << " "
        //                                                                                   << this->fans.at(2).get_id() << ":" << this->fans.at(2).get_vector().transpose() << " " << this->fans.at(2).current_state() << " " << this->fans.at(2).get_target_status() << " " << this->fans.at(2).get_switch_cnt() << " " << this->fans.at(2).last_detect_count_ << " " << this->fans.at(2).is_detected_last_ << " "
        //                                                                                   << this->fans.at(3).get_id() << ":" << this->fans.at(3).get_vector().transpose() << " " << this->fans.at(3).current_state() << " " << this->fans.at(3).get_target_status() << " " << this->fans.at(3).get_switch_cnt() << " " << this->fans.at(3).last_detect_count_ << " " << this->fans.at(3).is_detected_last_ << " "
        //                                                                                   << this->fans.at(4).get_id() << ":" << this->fans.at(4).get_vector().transpose() << " " << this->fans.at(4).current_state() << " " << this->fans.at(4).get_target_status() << " " << this->fans.at(4).get_switch_cnt() << " " << this->fans.at(4).last_detect_count_ << " " << this->fans.at(4).is_detected_last_ << " ");
        // for (auto fan_ : detect_fans)
        // {
        //     RCLCPP_INFO_STREAM(rclcpp::get_logger("rune_modeling"), fan_.get_vector().transpose() << " " << fan_.angle / M_PI * 180.);
        // }

        // 找到了就更新，没找到就保持
        int dis = std::distance(this->fans.begin(), std::find_if(this->fans.begin(), this->fans.end(), [](const Fan &fan_)
                                                                 { return ((fan_.current_state() == fan_status::INACTIVATED) && (fan_.get_target_status() != fan_status::ACTIVATED)); }));
        this->inactive_index = (dis == 5) ? this->inactive_index : dis;
        // RCLCPP_DEBUG(rclcpp::get_logger("rune_modeling"), "inactive_index:%d,size:%d,last_size:%d,switch:%d", this->inactive_index, detect_fans_size, this->last_detect_fans_size, switch_cnt_);

        return;
    }

    void Rune::self_examination()
    {
        double now_time = rclcpp::Clock().now().seconds();
        int cnt = 0;        // 待激活数量计数
        int stable_cnt = 0; // 现在状态和目标状态相等的扇叶数量
        for (auto iter = this->fans.begin(); iter != this->fans.end(); iter++)
        {
            // 检查相邻两扇叶间角度差
            double angle_diff = fabs(fmod(iter->angle - ((iter + 1) == this->fans.end() ? this->fans.begin() : (iter + 1))->angle, 2 * M_PI));
            angle_diff = angle_diff > M_PI ? 2 * M_PI - angle_diff : angle_diff;

            if (fabs(fabs(angle_diff) - 1.256) > 0.1256)
            {
                throw std::range_error("angle diff error" + std::to_string(angle_diff) + "this:" + std::to_string(iter->angle) + "next:" + std::to_string((iter + 1)->angle));
            }

            // 检查扇叶状态
            if (now_time - iter->get_last_switch_time() > 2.6 && iter->current_state() == fan_status::INACTIVATED)
            {
                // 待激活扇叶持续同一状态时间过长。
                // 说明识别有误或者逻辑错误？
                // std::cout << "Fan " << iter->get_id() << " keep inactivated status too long." << std::endl;
                throw std::range_error("Fan keep inactivated status too long." + std::to_string(iter->get_id()) + " " + std::to_string(iter->get_last_switch_time()));
            }

            if (iter->current_state() == fan_status::INACTIVATED && iter->get_target_status() == fan_status::INACTIVATED)
            {
                cnt++;
            }

            if (iter->current_state() == fan_status::DARK && iter->get_target_status() == fan_status::INACTIVATED)
            {
                cnt++;
            }
            if (iter->current_state() == iter->get_target_status())
            {
                stable_cnt++;
            }
        }
        if (cnt != 1)
        {
            // 待激活数量不为1,说明被误识别摧毁了逻辑判断，但是不能直接重开，可能会有误识别导致状态不能及时更新，所以进一步进行判断。
            this->failed_detect_count += 2;
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rune_modeling"), "inactivated != 1, logic errort" << this->fans.at(0).current_state() << "," << this->fans.at(0).get_target_status() << "\t"
                                                                                                      << this->fans.at(1).current_state() << "," << this->fans.at(1).get_target_status() << "\t"
                                                                                                      << this->fans.at(2).current_state() << "," << this->fans.at(2).get_target_status() << "\t"
                                                                                                      << this->fans.at(3).current_state() << "," << this->fans.at(3).get_target_status() << "\t"
                                                                                                      << this->fans.at(4).current_state() << "," << this->fans.at(4).get_target_status());

            // 这里需要进行进一步区分，即切换扇叶时，上一次的发弹正好击打到了新切换的待激活这种情况。
            if (this->fans.at(this->inactive_index).get_target_status() == fan_status::DARK)
            {
                // to be continued
            }
        }

        if (failed_detect_count > 0)
        {
            // 只要有错误次数，就不算有效，此时就不能开火。
            if (failed_detect_count < 10)
            {
                throw std::logic_error("failed detect");
            }
            // 错误次数太多直接重开。
            RCLCPP_ERROR(rclcpp::get_logger("rune_modeling"), "clear and restart");
            throw std::runtime_error("clear and restart");
        }
        if (stable_cnt == 5)
        {
            failed_detect_count = std::max(0, failed_detect_count - 1);
        }
    }

    rune_model_states_code Rune::no_detect_interpid(rune::optimization::CurveFittingProblem::Model modeling_param_, bool fit_valid)
    {
        std::cout << "interpid" << std::endl;
        if (this->inactive_index == -2)
        {
            // 第一帧就是g的，不做任何处理，保持未初始化状态
            return rune_model_states_code::POINTS_INVALID;
        }
        if (!fit_valid)
        {
            // 没有拟合完成，按照上一次更新的角度继续更新
            for (auto fan_ = this->fans.begin(); fan_ != this->fans.end(); ++fan_)
            {
                fan_->rotate(this->delta_angle);
            }
            return rune_model_states_code::POINTS_INVALID_BUT_COMPLEMENTED;
        }
        double time_now = rclcpp::Clock().now().seconds();
        this->delta_angle = std::copysign(1, this->dir) * (rune::optimization::CurveFittingProblem::getAngleBig(this->time_stamp, time_now, modeling_param_));
        for (auto fan_ = this->fans.begin(); fan_ != this->fans.end(); ++fan_)
        {
            fan_->rotate(this->delta_angle);
        }
        this->time_stamp = time_now;
        return rune_model_states_code::POINTS_INVALID_BUT_COMPLEMENTED;
    }

    std::pair<double, double> Rune::get_angle_data(int id)
    {
        return std::make_pair(this->time_stamp - this->start_time_stamp, this->fans.at(id).angle - this->angle_start);
    }

    Eigen::Vector3d Rune::get_target_position(double angel_rad) const
    {
        Fan tep_fan = this->fans.at(this->inactive_index);
        tep_fan.rotate(angel_rad);
        return tep_fan.get_armor_center();
    }

    Rune::~Rune() = default;
}