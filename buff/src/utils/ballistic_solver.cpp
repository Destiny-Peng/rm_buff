#include "ballistic_solver.hpp"

namespace rune
{

    bullet::bullet()
    {
        y_temp_.resize(n);
        dydt_.resize(n);
        target_y_.resize(n);
        y_.resize(n);
        flight_time = 0;
        marker.scale.x = 16.8e-3;
        marker.scale.y = 16.8e-3;
        marker.scale.z = 16.8e-3;
        marker.color.r = 1;
        marker.color.a = 1;
        marker.header.frame_id = "base_link";
        marker.ns = "bullet";
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = geometry_msgs::msg::Pose();
    }

    bullet::bullet(std::chrono::system_clock::time_point time, double theta, double yaw_, double v0, double x0, double y0)
    {
        y_temp_.resize(n);
        dydt_.resize(n);
        target_y_.resize(n);
        y_.resize(n);
        y_[0] = x0;
        y_[1] = v0 * cos(theta);
        y_[2] = y0;
        y_[3] = v0 * sin(theta);
        flight_time = 0;
        time_stamp = time;
        yaw = yaw_;
        marker.scale.x = 16.8e-3;
        marker.scale.y = 16.8e-3;
        marker.scale.z = 16.8e-3;
        marker.color.r = 1;
        marker.color.a = 1;
        marker.header.frame_id = "base_link";
        marker.ns = "bullet";
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = geometry_msgs::msg::Pose();
    }

    bullet::~bullet() {}

    void bullet::f(const std::vector<double> &y, std::vector<double> &dydt)
    {
        double v = sqrt(y[1] * y[1] + y[3] * y[3]);
        double D = D_m * v * v;
        double cos_theta = y[1] / v;
        double sin_theta = y[3] / v;

        dydt[0] = y[1];
        dydt[1] = -D * cos_theta;
        dydt[2] = y[3];
        dydt[3] = -D * sin_theta - g;
    }

    void bullet::rungeKutta4(double h_, double h2_, double h3_, double h6_)
    {
        f(y_, dydt_);

        for (int i = 0; i < n; ++i)
        {
            y_temp_[i] = y_[i] + h2_ * dydt_[i];
            y_[i] += h6_ * dydt_[i];
        }

        f(y_temp_, dydt_);

        for (int i = 0; i < n; ++i)
        {
            y_temp_[i] = y_[i] + h2_ * dydt_[i];
            y_[i] += h3_ * dydt_[i];
        }

        f(y_temp_, dydt_);

        for (int i = 0; i < n; ++i)
        {
            y_temp_[i] = y_[i] + h_ * dydt_[i];
            y_[i] += h3_ * dydt_[i];
        }

        f(y_temp_, dydt_);

        for (int i = 0; i < n; ++i)
        {
            y_[i] += h6_ * dydt_[i];
        }

        flight_time += h_;
        time_stamp += std::chrono::microseconds(static_cast<int64_t>(h_ * 1e6));
    }

    void bullet::calc_a_step(double h_)
    {
        this->rungeKutta4(h_, h_ / 2., h_ / 3., h_ / 6.);
    }

    void bullet::calc_a_step()
    {
        double h = std::chrono::duration<double>(std::chrono::system_clock::now() - time_stamp).count();
        this->rungeKutta4(h, h / 2., h / 3., h / 6.);
    }

    void bullet::calc_a_step(double h_, double yaw_)
    {
        this->yaw = yaw_;
        this->rungeKutta4(h_, h_ / 2., h_ / 3., h_ / 6.);
    }

    void bullet::get_state_vec(std::vector<double> &y)
    {
        y = this->y_;
    }

    Eigen::Vector3d bullet::get_state_vec_rotated()
    {
        return Eigen::Vector3d(y_[0] * cos(yaw), y_[0] * sin(yaw), y_[2]);
    }

    double bullet::get_flight_time()
    {
        return flight_time;
    }

    double bullet::get_time_stamp()
    {
        return std::chrono::duration<double>(time_stamp.time_since_epoch()).count();
    }

    visualization_msgs::msg::Marker bullet::get_visualize_msg(int index, builtin_interfaces::msg::Time stamp_)
    {
        marker.header.stamp = stamp_;
        marker.id = index;
        marker.pose.position.x = y_[0] * cos(yaw);
        marker.pose.position.y = y_[0] * sin(yaw);
        marker.pose.position.z = y_[2];
        return marker;
    }

    BallisticSolver::BallisticSolver()
    {
        options.linear_solver_type = ceres::DENSE_QR;
        options.initial_trust_region_radius = 5e4;
        options.function_tolerance = 0.2;
        // options.minimizer_progress_to_stdout = true;
        f_ = [this](const Eigen::Vector4f &y, Eigen::Vector4f &dydt)
        {
            float v = hypot(y[1], y[3]);
            float D = D_m_ * v * v;
            float inv_v = 1.0f / v;

            dydt[0] = y[1];
            dydt[1] = -D * y[1] * inv_v;
            dydt[2] = y[3];
            dydt[3] = -D * y[3] * inv_v - g;
        };
    }

    BallisticSolver::~BallisticSolver() {}

    void BallisticSolver::setResistanceCoefficient(double radius, double mass)
    {
        R_ = radius;
        m_ = mass;
        A_ = M_PI * R_ * R_;
        lambda_ = Cd * A_ * rho0 * 0.5;
        D_m_ = lambda_ / m_;
    }

    void BallisticSolver::getResult(const Eigen::Vector3d &target, double velocity,
                                    Eigen::Vector3d &euler_angles, double &flight_time)
    {
        target_x_ = hypot(target[0], target[1]);
        target_y_ = target[2];
        v0_ = velocity;

        euler_angles[1] = solveGunElevation();
        euler_angles[0] = atan2(target[1], target[0]);
        flight_time = this->flight_time_;
    }

    double BallisticSolver::calculateInitialTheta()
    {
        double dis = std::sqrt(target_x_ * target_x_ + target_y_ * target_y_);
        double phi = std::atan2(target_y_, target_x_);
        double alpha = std::asin((target_y_ + g * target_x_ * target_x_ / (v0_ * v0_)) / dis);
        double theta1 = (phi + alpha) / 2;
        double theta2 = M_PI - (phi + alpha) / 2;
        double theta = (theta1 + M_PI / 2 > theta2 + M_PI / 2) ? theta2 : theta1;

        if (fabs(theta) > M_PI / 2)
        {
            throw std::out_of_range("theta out of range" + std::to_string(theta));
        }

        return theta;
    }

    double BallisticSolver::solveGunElevation()
    {
        double theta0 = calculateInitialTheta();
        ceres::Problem problem;
        problem.AddParameterBlock(&theta0, 1);
        ceres::CostFunction *cost_function =
            new ceres::NumericDiffCostFunction<BallisticCostFunctor, ceres::FORWARD, 1, 1>(
                new BallisticCostFunctor(this));
        problem.AddResidualBlock(cost_function, nullptr, &theta0);

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        return theta0;
    }

    Eigen::Vector4f BallisticSolver::odesolve(const Eigen::Vector4f &y0, const Eigen::Vector4f &yTarget)
    {
        // auto start = std::chrono::high_resolution_clock::now();
        int n = 4;

        Eigen::Vector4f y_eigen = y0;

        std::array<Eigen::Vector4f, 6> ks_rkf_eigen = {
            Eigen::Vector4f::Zero(n),
            Eigen::Vector4f::Zero(n),
            Eigen::Vector4f::Zero(n),
            Eigen::Vector4f::Zero(n),
            Eigen::Vector4f::Zero(n),
            Eigen::Vector4f::Zero(n)};

        Eigen::Vector4f y_temp_eigen = Eigen::Vector4f::Zero(n);
        this->flight_time_ = 0.0;
        int iter = 0;
        bool converged = false;
        h_ = h;

        while (!converged && iter < maxIter)
        {
            rungeKuttaFehlberg45(f_, this->flight_time_, y_eigen, ks_rkf_eigen, y_temp_eigen, h_, 1e-2);
            iter++;

            // 检查是否应该结束迭代，指y已经小于目标值
            if (y_eigen[0] > yTarget[0])
            {
                converged = true;
            }
        }

        return y_eigen;
    }

    void BallisticSolver::rungeKuttaFehlberg45(odeFunc f, float &t, Eigen::Vector4f &y, std::array<Eigen::Vector4f, 6> &ks, Eigen::Vector4f &ytemp, float &h, const float h_max)
    {

        // 中间步骤系数
        constexpr float b21 = 1.0 / 4.0;
        constexpr float b31 = 3.0 / 8.0, b32 = 9.0 / 32.0;
        constexpr float b41 = 1932.0 / 2197.0, b42 = -7200.0 / 2197.0, b43 = 7296.0 / 2197.0;
        constexpr float b51 = 439.0 / 216.0, b52 = -8.0, b53 = 3680.0 / 513.0, b54 = -845.0 / 4104.0;
        constexpr float b61 = -8.0 / 27.0, b62 = 2.0, b63 = -3544.0 / 2565.0, b64 = 1859.0 / 4104.0, b65 = -11.0 / 40.0;

        // 解权重
        constexpr float c5[] = {16.0 / 135.0, 0, 6656.0 / 12825.0, 28561.0 / 56430.0, -9.0 / 50.0, 2.0 / 55.0};
        constexpr float e[] = {1.0f / 360.0f, 0, -128.0f / 4275.0f, -2197.0f / 75240.0f, 1.0f / 50.0f, 2.0f / 55.0f};
        constexpr float q_coff = 0.84f * 0.316227766f;
        // std::power(tol,0.25f)=0.316227766f

        /****** 分步计算 ​******/
        // k1 (存储在ks[0]和ks[6])
        f(y, ks[0]);

        // k2 (存储在ks[1]和ks[7])
        ytemp = y + h * b21 * ks[0];
        f(ytemp, ks[1]);

        // k3 (存储在ks[2]和ks[8])
        ytemp = y + h * (b31 * ks[0] + b32 * ks[1]);
        f(ytemp, ks[2]);

        // k4 (存储在ks[3]和ks[9])
        ytemp = y + h * (b41 * ks[0] + b42 * ks[1] + b43 * ks[2]);
        f(ytemp, ks[3]);

        // k5 (存储在ks[4]和ks[10])
        ytemp = y + h * (b51 * ks[0] + b52 * ks[1] + b53 * ks[2] + b54 * ks[3]);
        f(ytemp, ks[4]);

        // k6 (存储在ks[5]和ks[11])
        ytemp = y + h * (b61 * ks[0] + b62 * ks[1] + b63 * ks[2] + b64 * ks[3] + b65 * ks[4]);
        f(ytemp, ks[5]);

        /****** 误差估计 ​******/
        Eigen::Vector4f error_vec = h * (e[0] * ks[0] + e[2] * ks[2] + e[3] * ks[3] +
                                         e[4] * ks[4] + e[5] * ks[5]);

        float R = error_vec.norm();

        /****** 步长调整 ​******/
        // float q = 0.84f * std::pow(tol / R, 0.25f);
        float q = q_coff / std::sqrt(std::sqrt(R));
        h *= std::clamp(q, 0.2f, 4.0f); // 限制步长变化率
        h = std::min(h, h_max);         // 限制最大步长
        /****** 接受五阶解 ​******/
        if (R <= tol)
        {
            y += h * (c5[0] * ks[0] + c5[2] * ks[2] + c5[3] * ks[3] +
                      c5[4] * ks[4] + c5[5] * ks[5]);
            t += h;
        }
        // 否则保持y不变，h已被缩小
    }

}