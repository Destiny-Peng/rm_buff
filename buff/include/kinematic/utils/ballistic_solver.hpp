#ifndef BALLISTIC_SOLVER_HPP
#define BALLISTIC_SOLVER_HPP
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <string>
#include <chrono>

namespace rune
{
    using odeFunc = std::function<void(const Eigen::Vector4f &, Eigen::Vector4f &)>;
    // 常量定义
    constexpr double g = 9.7988;                   // 当地的重力加速度
    constexpr double rho0 = 1.204;                 // 空气密度(海平面)
    constexpr double Cd = 0.5;                     // drag coefficient
    constexpr double R = 16.8e-3 / 2.0;            // 半径
    constexpr double A = M_PI * R * R;             // 横截面积
    constexpr double m = 3.2e-3;                   // 质量
    constexpr double lambda = Cd * A * rho0 * 0.5; // 空气阻力系数
    constexpr double D_m = lambda / m;             // 空气阻力系数与质量的比值
    constexpr double h = 0.002;                    // 步长
    constexpr double h2 = h / 2.0;                 // 步长的二分之一，提前算好减少计算量。
    constexpr double h3 = h / 3.0;                 // 步长的三分之一，提前算好减少计算量。
    constexpr double h6 = h / 6.0;                 // 步长的六分之一，提前算好减少计算量。
    constexpr int maxIter = 1e4;                   // 最大迭代次数
    constexpr double tol = 1e-2;                   // 收敛阈值
    const int n = 4;                               // 状态变量的维度

    class bullet
    {
    private:
        std::chrono::system_clock::time_point time_stamp = std::chrono::system_clock::now();
        double flight_time;
        double yaw;
        visualization_msgs::msg::Marker marker;
        std::vector<double> y_;
        std::vector<double> y_temp_;
        std::vector<double> dydt_;
        std::vector<double> target_y_;

        void f(const std::vector<double> &y, std::vector<double> &dydt);
        void rungeKutta4(double h_ = 0.1, double h2_ = 0.05, double h3_ = 0.0333, double h6_ = 0.0167);

    public:
        bullet();
        bullet(std::chrono::system_clock::time_point time, double theta, double yaw_, double v0, double x0, double y0);
        ~bullet();

        void calc_a_step(double h_);
        void calc_a_step();
        void calc_a_step(double h_, double yaw_);
        void get_state_vec(std::vector<double> &y);
        Eigen::Vector3d get_state_vec_rotated();
        double get_flight_time();
        double get_time_stamp();
        visualization_msgs::msg::Marker get_visualize_msg(int index, builtin_interfaces::msg::Time stamp_);
    };

    class BallisticSolver
    {

    public:
        double v0_ = 27;
        double target_x_ = -1.0;
        double target_y_ = -1.0;
        float flight_time_ = 0.0;
        ceres::Solver::Options options;

        BallisticSolver();
        ~BallisticSolver();

        void setResistanceCoefficient(double radius, double mass);
        void getResult(const Eigen::Vector3d &target, double velocity, Eigen::Vector3d &euler_angle, double &flight_time);
        Eigen::Vector4f odesolve(const Eigen::Vector4f &y0, const Eigen::Vector4f &yTarget);

    private:
        double R_ = R;
        double m_ = m;
        double A_ = A;
        double lambda_ = lambda;
        double D_m_ = D_m;
        float h_ = h;
        odeFunc f_;
        double calculateInitialTheta();
        double solveGunElevation();
        void rungeKuttaFehlberg45(rune::odeFunc f, float &t, Eigen::Vector4f &y, std::array<Eigen::Vector4f, 6> &ks, Eigen::Vector4f &ytemp, float &h, const float h_max);
    };
    struct BallisticCostFunctor
    {
        BallisticCostFunctor(BallisticSolver *solver) : solver_(solver) {}

        bool operator()(const double *const theta0, double *residuals) const
        {
            const double v0 = solver_->v0_;
            const double target_x = solver_->target_x_;
            const double target_y = solver_->target_y_;

            Eigen::Vector4f y0_ = Eigen::Vector4f({0, v0 * cos(*theta0), 0, v0 * sin(*theta0)});
            Eigen::Vector4f yTarget_ = Eigen::Vector4f({target_x, 0, target_y, 0});
            Eigen::Vector4f y_ = Eigen::Vector4f::Zero();
            y_ = this->solver_->odesolve(y0_, yTarget_);
            float x_error = y_[0] - target_x;
            float y_error = y_[2] - target_y;
            residuals[0] = x_error * x_error + y_error * y_error;
            // std::cout << "y_: " << y_.transpose() << std::endl;
            // std::cout << "yT: " << yTarget_.transpose() << std::endl;

            return true;
        }

        BallisticSolver *solver_;
    };
}

#endif