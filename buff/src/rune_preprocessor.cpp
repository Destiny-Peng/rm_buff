#include "rune_parameters.hpp"
#include "rune_preprocessor.hpp"

namespace rune
{
    Eigen::Quaterniond RunePreprocessor::quaternion_from_euler(double yaw, double pitch, double roll)
    {
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
        // 已经搞不懂了，干脆内旋完事了。
        return yawAngle * pitchAngle * rollAngle;
    }

    RunePreprocessor::RunePreprocessor(const rclcpp::NodeOptions &options) : Node("rune_preprocessor", options)
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(), std::chrono::seconds(30));
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        fan_armor_pub = this->create_publisher<interfaces::msg::FanArmors>("FanArmors", 10);
        this->declare_parameter("use_sim", false);
        this->use_sim = this->get_parameter("use_sim").as_bool();
        if (this->use_sim)
        {
            rune_img_process_result_sub = this->create_subscription<interfaces::msg::Rune>("rune_debug_detect_result", 1, std::bind(&RunePreprocessor::rune_callback, this, std::placeholders::_1));
        }
        else
        {
            rune_img_process_result_sub = this->create_subscription<interfaces::msg::Rune>("Rune", 1, std::bind(&RunePreprocessor::rune_callback, this, std::placeholders::_1));
        }
        timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&RunePreprocessor::trigger_circle_fitting, this));

        this->declare_parameter("g_camera_param_path", "src/config/camera_param5.xml");
        g_camera_param_path = this->get_parameter("g_camera_param_path").as_string();
        cv::FileStorage camera_param_file;
        camera_param_file.open(g_camera_param_path, cv::FileStorage::READ);
        if (!camera_param_file.isOpened())
        {
            RCLCPP_INFO(this->get_logger(), "open camera_param.yaml failed.%s", g_camera_param_path.c_str());
            abort();
        }
        // 内参和畸变矩阵
        camera_param_file["camera_internal_matrix"] >> this->camera_matrix_;
        camera_param_file["distortion_coeff"] >> this->distortion_coeff_;
        this->pnp_solver = std::make_unique<rune::PnPSolver>(this->camera_matrix_, this->distortion_coeff_);
        this->pnp_solver->setObjectPoints("rune", RUNE_OBJECT_POINTS);
        this->pnp_solver->setObjectPoints("rune_interpid", RUNE_OBJECT_POINTS_INTERPID);
        this->pnp_solver->setObjectPoints("rune_with_r", RUNE_OBJECT_POINTS_WITH_R);
        this->pnp_solver->setObjectPoints("rune_with_r_interpid", RUNE_OBJECT_POINTS_WITH_R_INTERPID);

        this->declare_parameter("g_tune_param_path", "src/config/tune_param5.xml");
        g_tune_param_path = this->get_parameter("g_tune_param_path").as_string();
        cv::FileStorage tune_param_file;
        tune_param_file.open(g_tune_param_path, cv::FileStorage::READ);
        if (!tune_param_file.isOpened())
        {
            RCLCPP_INFO(this->get_logger(), "open tune_param.yaml failed.%s", g_tune_param_path.c_str());
            abort();
        }
        double x_offset, y_offset, z_offset;
        tune_param_file["X_OFFSET"] >> x_offset;
        tune_param_file["Y_OFFSET"] >> y_offset;
        tune_param_file["Z_OFFSET"] >> z_offset;

        tf_pub = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        gimbal_tf_msg.header.frame_id = "base_link";
        gimbal_tf_msg.child_frame_id = "gimbal";

        static_tf_pub = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        camera_tf_msg.header.frame_id = "gimbal";
        camera_tf_msg.child_frame_id = "camera";
        camera_tf_msg.transform.translation.x = z_offset / 1000.;
        camera_tf_msg.transform.translation.y = -x_offset / 1000.;
        camera_tf_msg.transform.translation.z = -y_offset / 1000.;
        // 都转成m tf中是坐标系变换，因此我们应该给出camera在gimbal坐标系下，gimbal原点指向camera原点的向量。
        camera_tf_msg.transform.rotation.w = 0.5;
        camera_tf_msg.transform.rotation.x = -0.5;
        camera_tf_msg.transform.rotation.y = 0.5;
        camera_tf_msg.transform.rotation.z = -0.5;
        static_tf_pub->sendTransform(camera_tf_msg);

        target_tf_msg.header.frame_id = "camera";
        target_tf_msg.child_frame_id = "target";

        rune_tf_msg.header.frame_id = "base_link";
        rune_tf_msg.child_frame_id = "rune";
    }

    RunePreprocessor::~RunePreprocessor() {}

    /**
     * @brief 清空数据，每次进入能量机关前，进行清空
     */
    void RunePreprocessor::clear()
    {
    }

    void RunePreprocessor::rune_callback(const interfaces::msg::Rune::SharedPtr msg)
    {
        // 将消息中的格式转换为可读取格式
        this->convertMsgToLocalData(msg);

        interfaces::msg::FanArmors fan_armors_;
        // 使用pnp对每个扇叶进行求解
        for (auto points : this->rune_armor_points)
        {
            interfaces::msg::FanArmor fan_armor_;
            cv::Mat r_vec(3, 1, CV_64F), t_vec(3, 1, CV_64F);
            if (points.size() == 5)
            {
                // this->pnp_solver->solvePnP(points, r_vec, t_vec, "rune_with_r");
                // 插值生成虚拟的边中点和中心点
                // 计算每条边的中点
                std::vector<cv::Point2f> new_points(points.begin(), points.end());
                cv::Point2f object_center(0, 0);
                for (size_t i = 0; i < 4; ++i)
                {
                    const cv::Point2f &p1 = points[i];
                    const cv::Point2f &p2 = points[(i + 1) % 4];
                    cv::Point2f midpoint = (p1 + p2) * 0.5;
                    new_points.push_back(midpoint);
                    object_center += p1;
                }
                // 计算物体的中点（所有顶点的平均值）
                object_center *= 1.0 / 4;
                new_points.push_back(object_center);
                this->pnp_solver->solvePnP(new_points, r_vec, t_vec, "rune_with_r_interpid");
            }
            else
            {
                // this->pnp_solver->solvePnP(points, r_vec, t_vec, "rune");
                // 插值生成虚拟的边中点和中心点
                // 计算每条边的中点
                std::vector<cv::Point2f> new_points(points.begin(), points.end());
                cv::Point2f object_center(0, 0);
                for (size_t i = 0; i < 4; ++i)
                {
                    const cv::Point2f &p1 = points[i];
                    const cv::Point2f &p2 = points[(i + 1) % 4];
                    cv::Point2f midpoint = (p1 + p2) * 0.5;
                    new_points.push_back(midpoint);
                    object_center += p1;
                }
                // 计算物体的中点（所有顶点的平均值）
                object_center *= 1.0 / 4;
                new_points.push_back(object_center);
                this->pnp_solver->solvePnP(new_points, r_vec, t_vec, "rune_interpid");
            }
            // std::cout << t_vec << std::endl;

            // 发布target的tf
            Eigen::Vector3d r_vec_eigen;
            cv::cv2eigen(r_vec, r_vec_eigen);
            Eigen::Quaterniond q(Eigen::AngleAxisd(r_vec_eigen.norm(), r_vec_eigen.normalized()));
            Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
            T.rotate(q);
            T.pretranslate(Eigen::Vector3d(t_vec.at<double>(0) / 1000., t_vec.at<double>(1) / 1000., t_vec.at<double>(2) / 1000.));
            target_tf_msg.transform = tf2::eigenToTransform(T).transform;

            target_tf_msg.header.stamp = msg->stamp;
            this->tf_pub->sendTransform(target_tf_msg);

            // 将结果存起来

            geometry_msgs::msg::TransformStamped t;

            // Look up for the transformation between target_frame and turtle2 frames
            // and send velocity commands for turtle2 to reach target_frame
            try
            {
                t = tf_buffer_->lookupTransform(
                    "base_link", "target",
                    msg->stamp,
                    std::chrono::microseconds(30));
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", "base_link", "camera", ex.what());
                return;
            }
            Eigen::Isometry3d isometry = tf2::transformToEigen(t);

            twopoint two_points;
            two_points.p1 = isometry * Eigen::Vector3d(0, 0, 0);
            two_points.p2 = isometry * Eigen::Vector3d(0, -STANDARD_RUNE_RADIUS, 0);

            fan_armor_.armor_center.point = tf2::toMsg(two_points.p1);
            fan_armor_.armor_center.header.stamp = msg->stamp;
            fan_armor_.armor_center.header.frame_id = "base_link";

            fan_armor_.r_center.point = tf2::toMsg(two_points.p2);
            fan_armor_.r_center.header.stamp = msg->stamp;
            fan_armor_.r_center.header.frame_id = "base_link";

            fan_armors_.fan_armors.push_back(fan_armor_);
            this->pnp_result_cache_.push(two_points); // 将结果存起来
        }
        // 所有点都处理完了，发布
        this->fan_armor_pub->publish(fan_armors_);
    }

    bool RunePreprocessor::convertMsgToLocalData(const interfaces::msg::Rune::SharedPtr msg)
    {
        this->rune_armor_points.clear();
        this->rune_armor_points.resize(msg->vvpoint.size());
        for (int i = 0; i < msg->vvpoint.size(); ++i)
        {
            this->rune_armor_points[i].resize(msg->vvpoint[i].vpoint.size());
            for (int j = 0; j < msg->vvpoint[i].vpoint.size(); ++j)
            {
                cv::Point2d pt;
                pt.x = msg->vvpoint[i].vpoint[j].x;
                pt.y = msg->vvpoint[i].vpoint[j].y;
                this->rune_armor_points[i][j] = pt;
            }
            // if (msg->r_center.x > 0)
            // {
            //     this->rune_armor_points[i].push_back(cv::Point2d(msg->r_center.x, msg->r_center.y));
            // }
        }

        // if (this->circle_calibration_state <= rune_model_states_code::ZERO)
        // {
        //     this->energy_yaw = msg->energy_yaw;
        // }
        Eigen::Quaterniond q = this->quaternion_from_euler(msg->pose.ptz_yaw * M_PI / 180., -msg->pose.ptz_pitch * M_PI / 180., -msg->pose.ptz_roll * M_PI / 180.);
        gimbal_tf_msg.transform.rotation.w = q.w();
        gimbal_tf_msg.transform.rotation.x = q.x();
        gimbal_tf_msg.transform.rotation.y = q.y();
        gimbal_tf_msg.transform.rotation.z = q.z();
        gimbal_tf_msg.header.stamp = msg->stamp;
        this->tf_pub->sendTransform(gimbal_tf_msg);

        // this->pose.ptz_pitch += receive_rune_pitch_offset + (this->pose.ptz_pitch - raw_rune_low_pitch) * rune_k;
        // this->pose.ptz_roll += receive_rune_roll_offset;
        // this->pose.ptz_yaw += receive_rune_yaw_offset;

        this->r_center_pixel.x = msg->r_center.x;
        this->r_center_pixel.y = msg->r_center.y;
    }

    void RunePreprocessor::trigger_circle_fitting()
    {
        // 第一层快速检查（无锁）
        if (is_fitting_.load(std::memory_order_acquire))
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "[Fast Check] Fitting in progress");
            return;
        }

        // 第二层精确检查（带锁）
        std::lock_guard<std::mutex> lock(fitting_mutex_);

        // 双重检查锁定模式
        if (is_fitting_.load(std::memory_order_relaxed))
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "[Accurate Check] Fitting in progress");
            return;
        }

        // 节流控制检查
        const auto now = std::chrono::steady_clock::now();
        if (now - last_fit_time_ < std::chrono::milliseconds(MIN_FIT_INTERVAL))
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "Fitting throttled. Elapsed: %ldms",
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
                fit_circle_task();
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

    void RunePreprocessor::fit_circle_task()
    {
        // 取一份快照
        std::vector<std::shared_ptr<twopoint>> snapshot = pnp_result_cache_.snapshot();

        if (snapshot.size() < 80)
        {
            std::__throw_runtime_error("Too few points for circle fitting");
        }

        // 实际的拟合算法实现
        // RCLCPP_INFO(this->get_logger(), "Starting circle fitting...");

        // 1. 平面拟合阶段
        std::vector<Eigen::Vector3d> armor_points, r_center_points;
        for (auto &ptr : snapshot)
        {
            armor_points.push_back(ptr->p1);
            r_center_points.push_back(ptr->p2);
        }

        // 执行带约束的平面拟合
        auto constrained_model = this->plane_ransac.run(
            armor_points,
            problem_,
            rune::optimization::RandomPointSampler{},
            rune::optimization::MaxIterationsOrErrorThresholdTermination(200, 5.0));
        if (constrained_model == rune::optimization::ZConstrainedPlaneFitting::Model())
        {
            RCLCPP_INFO(this->get_logger(), "Plane fitting failed");
            return;
        }

        // ================== 第二阶段：坐标投影 ==================
        Eigen::Vector3d normal(constrained_model[0], constrained_model[1], 0.0);
        Eigen::Vector3d plane_center = Eigen::Vector3d::Zero();
        // for (size_t i = 0; i < r_center_points.size(); i++)
        // {
        //     plane_center += r_center_points[i];
        // }
        // plane_center /= r_center_points.size();

        plane_center = std::accumulate(r_center_points.begin(), r_center_points.end(), plane_center);
        plane_center /= static_cast<double>(r_center_points.size());
        // 构建正交基底
        Eigen::Vector3d local_x = normal.unitOrthogonal().normalized();
        Eigen::Vector3d local_y = normal.cross(local_x).normalized();

        // 投影到二维平面坐标系
        std::vector<Eigen::Vector2d> local_points;
        for (const auto &pt : armor_points)
        {
            Eigen::Vector3d vec = pt - plane_center;
            local_points.emplace_back(
                vec.dot(local_x), // x'坐标
                vec.dot(local_y)  // y'坐标
            );
        }

        // 平面原点
        // 平面参数改用角度表示，确保单位法向量
        double theta_initial = atan2(constrained_model[1], constrained_model[0]);
        // 使用三目运算符限制角度,将角度限制在 [-pi/2, pi/2]
        theta_initial = (theta_initial > M_PI / 2) ? (theta_initial - M_PI) : (theta_initial < -M_PI / 2) ? (theta_initial + M_PI)
                                                                                                          : theta_initial;

        // ================== 第三阶段：二维圆拟合 ==================
        // 初始化参数（转换历史值到局部坐标系）
        double center_2d[2] = {0, 0};
        double radius = STANDARD_RUNE_RADIUS;

        // 构建优化问题
        ceres::Problem problem;
        for (const auto &local_pt : local_points)
        {
            ceres::CostFunction *cost_function =
                new ceres::AutoDiffCostFunction<rune::optimization::Circle2DFittingCost, 1, 2, 1>(
                    new rune::optimization::Circle2DFittingCost(local_pt));
            problem.AddResidualBlock(cost_function, nullptr, center_2d, &radius);
        }

        // 设置参数约束
        constexpr double kMaxRadius = 1.5; // 根据实际场景调整
        problem.SetParameterLowerBound(center_2d, 0, -kMaxRadius);
        problem.SetParameterUpperBound(center_2d, 0, kMaxRadius);
        problem.SetParameterLowerBound(center_2d, 1, -kMaxRadius);
        problem.SetParameterUpperBound(center_2d, 1, kMaxRadius);
        problem.SetParameterLowerBound(&radius, 0, 0.60);
        problem.SetParameterUpperBound(&radius, 0, 0.80);

        // 5. 构建优化问题
        // 初始化参数
        // double theta = theta_initial;
        // double d = constrained_model[3];
        // double center[3] = {plane_center.x(), plane_center.y(), plane_center.z()};
        // double radius = STANDARD_RUNE_RADIUS;
        // double initial_angle_weight = 0.3 * std::sqrt(armor_points.size()); // 基于数据量自适应

        // // 创建优化问题
        // ceres::Problem problem;
        // for (const auto &pt : armor_points)
        // {
        //     ceres::CostFunction *cost_function =
        //         new ceres::AutoDiffCostFunction<rune::optimization::ConstrainedCircle3DFittingCost, 2, 1, 1, 3, 1>(
        //             new rune::optimization::ConstrainedCircle3DFittingCost(pt));

        //     problem.AddResidualBlock(
        //         cost_function,
        //         new ceres::HuberLoss(0.5), // 鲁棒损失函数
        //         &theta, &d, center, &radius);
        // }

        // // 添加角度正则化约束
        // problem.AddResidualBlock(
        //     new ceres::AutoDiffCostFunction<rune::optimization::AngleRegularizationCost, 1, 1>(
        //         new rune::optimization::AngleRegularizationCost(theta_initial)),
        //     new ceres::ScaledLoss( // 动态加权损失函数
        //         new ceres::CauchyLoss(0.1),
        //         initial_angle_weight, // 初始权重系数，建议0.1-0.5
        //         ceres::TAKE_OWNERSHIP),
        //     &theta);

        // // 设置参数边界
        // constexpr double kMaxAngleDeviation = 2.0 * M_PI / 180; // 允许±2度偏移
        // problem.SetParameterLowerBound(&theta, 0, theta_initial - kMaxAngleDeviation);
        // problem.SetParameterUpperBound(&theta, 0, theta_initial + kMaxAngleDeviation);
        // ================== 第四阶段：执行优化 ==================
        ceres::Solver::Summary summary;
        ceres::Solver::Options options;
        ceres::Solve(options, &problem, &summary);

        // ================== 第五阶段：结果转换 ==================
        if (summary.IsSolutionUsable())
        {
            // 将二维结果转换回三维全局坐标系
            Eigen::Vector3d optimized_local_center =
                center_2d[0] * local_x +
                center_2d[1] * local_y + plane_center;
            Eigen::Isometry3d trans = Eigen::Isometry3d::Identity();
            trans.pretranslate(optimized_local_center);
            trans.rotate(Eigen::AngleAxisd(theta_initial, Eigen::Vector3d::UnitZ()));
            // 从能量机关平面到世界坐标系的变换
            rune_tf_msg.transform = tf2::eigenToTransform(trans).transform;
            rune_tf_msg.header.stamp = this->now();
            this->tf_pub->sendTransform(rune_tf_msg);

            RCLCPP_DEBUG_STREAM(this->get_logger(), "Optimized center: " << optimized_local_center.transpose() * 1000. << " Radius: %f mm" << radius * 1000);
        }

        // // 维护角度历史状态
        // static std::deque<double> angle_history;
        // angle_history.push_back(theta);
        // if (angle_history.size() > 5)
        //     angle_history.pop_front();
        // double filtered_theta{};
        // // 使用中值滤波
        // if (angle_history.size() >= 3)
        // {

        //     std::nth_element(angle_history.begin(),
        //                      angle_history.begin() + angle_history.size() / 2,
        //                      angle_history.end());
        //     filtered_theta = angle_history[angle_history.size() / 2];
        // }
        // else
        // {
        //     filtered_theta = theta;
        // }
        // std::cout << summary.FullReport() << std::endl;

        // if (summary.IsSolutionUsable()) // 检验本次拟合是否准确
        // {
        //     // this->rune_model.center = this->rune_model.center * 0.9 + Eigen::Vector3d(C_data[0], C_data[1], C_data[2]) * 1000.0 * 0.1;
        //     // this->rune_model.radius = (fabs(r) * 0.15 + STANDARD_RUNE_RADIUS * 0.85) * 1000.0;
        //     // 转换回全局坐标系
        //     Eigen::Vector3d final_normal(cos(theta), sin(theta), 0);
        //     Eigen::Vector3d final_center(center[0], center[1], center[2]);
        //     // RCLCPP_INFO_STREAM(this->get_logger(), "plane_center: " << final_center.transpose() << "norm:" << normal.transpose());
        //     // 计算夹角
        //     double diff_theta = std::fabs(acos(final_normal.dot(Eigen::Vector3d(cos(0.21), sin(0.21), 0))));
        //     diff_theta = std::min(diff_theta, M_PI - diff_theta);

        //     // RCLCPP_INFO_STREAM(this->get_logger(), "final norm:" << final_normal.transpose() << "angle diff: " << diff_theta * 180 / M_PI);

        //     // 计算圆心的误差
        //     Eigen::Vector3d final_center_error = final_center * 1000. - Eigen::Vector3d(6433.3, 0.0, 1075);

        //     // std::ofstream out_file;
        //     // out_file.open("output.csv", std::ios::app);
        //     // out_file << std::to_string(final_center_error.x()) << "," << std::to_string(final_center_error.y()) << "," << std::to_string(diff_theta) << std::endl;
        //     // out_file.close();

        //     // 更新模型参数

        //     // RCLCPP_DEBUG(this->get_logger(), "center(%lf,%lf,%lf),vec(%lf,%lf),r=%lf,t=%lf", C_data[0], C_data[1], C_data[2], norm_vector_data[0], norm_vector_data[1], this->rune_model.radius, summary.total_time_in_seconds);
        // }
        // else
        // {
        //     // RCLCPP_WARN(this->get_logger(), "calibrate failed! center(%lf,%lf,%lf),vec(%lf,%lf),r=%lf,t=%lf,cost=%lf", C_data[0], C_data[1], C_data[2], norm_vector_data[0], norm_vector_data[1], this->rune_model.radius, summary.total_time_in_seconds, summary.final_cost);
        // }

        RCLCPP_INFO(this->get_logger(), "Circle fitting completed");
    }

    void RunePreprocessor::finalize_fitting()
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
}
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rune::RunePreprocessor)
