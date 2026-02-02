#ifndef PROBLEM_HPP
#define PROBLEM_HPP

#include <vector>
#include <array>
#include <ceres/ceres.h>
#include <algorithm>

namespace rune::optimization
{
    // *****************************
    //         核心类型定义
    // *****************************
    enum class Convexity
    {
        CONVEX,
        CONCAVE
    };

    /**
     * @brief 惩罚项，让拟合的参数更加贴近预设的参数
     */
    class CostFunctor1 : public ceres::SizedCostFunction<1, 5>
    {
    public:
        CostFunctor1(double truth_, int id_) : truth(truth_), id(id_) {}
        virtual ~CostFunctor1() {};
        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
        {
            double pre = parameters[0][id];
            residuals[0] = pre - truth;
            if (jacobians != nullptr)
            {
                if (jacobians[0] != nullptr)
                {
                    for (int i = 0; i < 5; ++i)
                    {
                        if (i == id)
                        {
                            jacobians[0][i] = 1;
                        }
                        else
                        {
                            jacobians[0][i] = 0;
                        }
                    }
                }
            }
            return true;
        }
        double truth;
        int id;
    };

    /**
     * @brief 拟合项
     */
    class CostFunctor2 : public ceres::SizedCostFunction<1, 5>
    {
    public:
        CostFunctor2(double t_, double y_) : t(t_), y(y_) {}
        virtual ~CostFunctor2() {};
        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
        {
            double a = parameters[0][0];
            double w = parameters[0][1];
            double t0 = parameters[0][2];
            double b = parameters[0][3];
            double c = parameters[0][4];
            double cs = cos(w * (t + t0));
            double sn = sin(w * (t + t0));
            residuals[0] = -a * cs + b * t + c - y + a + b - 2.09;
            if (jacobians != nullptr)
            {
                if (jacobians[0] != nullptr)
                {
                    jacobians[0][0] = -cs + 1;
                    jacobians[0][1] = a * (t + t0) * sn;
                    jacobians[0][2] = a * w * sn;
                    jacobians[0][3] = t + 1;
                    jacobians[0][4] = 1;
                }
            }
            return true;
        }
        double t, y;
    };

    // *****************************
    //         优化问题抽象
    // *****************************
    struct CurveFittingProblem
    {
        using Model = std::array<double, 5>;
        // a/w,w,t0,b,c
        using DataPoint = std::pair<double, double>;

        Convexity convexity;
        Model params = {0.470, 1.942, 0, 1.178, 0};
        explicit CurveFittingProblem(Convexity c) : convexity(c) {}
        CurveFittingProblem() : convexity(Convexity::CONVEX) {}

        static constexpr int min_samples_required() { return 100; }
        static constexpr double inlier_threshold() { return 0.189; }

        void setParams(const Model &p) { params = p; }

        Model fitModel(const std::vector<DataPoint> &samples) const
        {
            return optimize(samples, params, convexity);
        }

        double computeError(const DataPoint &pt, const Model &model) const
        {
            return std::abs(pt.second - getAngleBig(pt.first, model));
        }

        double computeRelError(const DataPoint &pt, const Model &model) const
        {
            return std::abs(pt.second - getAngleBig(pt.first, model)) / std::abs(pt.second);
        }

        /**
         * @brief 得到大符角度，这里是定积分，从0到time。
         * @param[in] time          时间
         * @param[in] params        参数
         * @return double
         */
        static inline double getAngleBig(double time, const std::array<double, 5> &params)
        {
            return -params[0] * std::cos(params[1] * (time + params[2])) + params[3] * time + params[4];
        }
        /**
         * @brief 得到大符角度，这里是定积分，从t1到t2。
         * @param[in] time          时间
         * @param[in] params        参数
         * @return double
         */
        static inline double getAngleBig(double t1, double t2, const std::array<double, 5> &params)
        {
            // return -params[0] * std::cos(params[1] * (t2 + params[2])) + params[3] * t2 - (-params[0] * std::cos(params[1] * (t1 + params[2])) + params[3] * t1);
            // 整理得
            return -params[0] * std::cos(params[1] * (t2 + params[2])) + params[3] * (t2 - t1) + params[0] * std::cos(params[1] * (t1 + params[2]));
        }

    private:
        Model optimize(const std::vector<DataPoint> &points,
                       const Model &initial_params,
                       Convexity convexity) const
        {
            Model params = initial_params;
            ceres::Problem problem;

            // 添加数据点约束
            for (const auto &pt : points)
            {
                ceres::CostFunction *costFunction = new CostFunctor2(pt.first, pt.second);
                ceres::LossFunction *lossFunction = new ceres::SoftLOneLoss(0.1);
                problem.AddResidualBlock(costFunction, lossFunction, params.begin());
            }

            // 参数约束逻辑
            std::array<double, 3> omega;
            if (points.size() < 100)
            {
                if (convexity == Convexity::CONCAVE)
                {
                    problem.SetParameterUpperBound(params.begin(), 2, -2.8);
                    problem.SetParameterLowerBound(params.begin(), 2, -4);
                }
                else
                {
                    problem.SetParameterUpperBound(params.begin(), 2, -1.1);
                    problem.SetParameterLowerBound(params.begin(), 2, -2.3);
                }
                omega = {10., 1., 1.};
            }
            else
            {
                omega = {60., 50., 50.};
            }

            // 添加参数约束
            auto addParamConstraint = [&](int index, double omega_val)
            {
                ceres::CostFunction *cf = new CostFunctor1(params[index], index);
                ceres::LossFunction *lf = new ceres::ScaledLoss(
                    new ceres::HuberLoss(0.1), omega_val, ceres::TAKE_OWNERSHIP);
                problem.AddResidualBlock(cf, lf, params.begin());
            };

            addParamConstraint(0, omega[0]);
            addParamConstraint(1, omega[1]);
            addParamConstraint(3, omega[2]);

            // 求解优化问题
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 50;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            // std::cout << summary.BriefReport() << "\n";

            return params;
        }
    };

    /**
     * @brief 平面拟合问题策略
     */
    struct PlaneFittingProblem
    {
        using DataPoint = Eigen::Vector3d;
        using Model = std::pair<Eigen::Vector3d, Eigen::Vector3d>; // (法向量, 中心点)

        int min_samples_required() const { return 3; }

        Model fitModel(const std::vector<DataPoint> &samples) const
        {
            Eigen::Vector3d center = Eigen::Vector3d::Zero();
            for (const auto &pt : samples)
                center += pt;
            center /= samples.size();

            Eigen::MatrixXd centered(samples.size(), 3);
            for (size_t i = 0; i < samples.size(); ++i)
                centered.row(i) = samples[i] - center;

            Eigen::Matrix3d cov = (centered.adjoint() * centered) / (samples.size() - 1);
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullV);

            return {svd.matrixV().col(2), center};
        }

        double computeError(const DataPoint &pt, const Model &model) const
        {
            const auto &[normal, center] = model;
            Eigen::Vector3d vec = pt - center;
            return std::abs(vec.dot(normal)) / normal.norm();
        }

        double inlier_threshold() const { return 0.1; }
    };

    /**
     * @brief 带Z轴约束的平面拟合策略
     * @note 强制平面法向量z分量为零，即平面方程为ax + by + d = 0
     */
    class ZConstrainedPlaneFitting
    {
    public:
        using DataPoint = Eigen::Vector3d;
        using Model = Eigen::Vector4d;                             // [nx, ny, 0, d] (归一化法向量 + 截距项)
        static constexpr int min_samples_required() { return 50; } // 两点确定XY平面直线
        static constexpr double inlier_threshold() { return 0.5; }

        Model fitModel(const std::vector<DataPoint> &samples) const
        {
            // 构建约束方程组：nx*x + ny*y + d = 0
            Eigen::MatrixXd A(samples.size(), 3);
            double x_sum = 0.0, y_sum = 0.0;
            for (size_t i = 0; i < samples.size(); ++i)
            {
                A(i, 0) = samples[i].x();
                A(i, 1) = samples[i].y();
                A(i, 2) = 1.0;
                x_sum += samples[i].x();
                y_sum += samples[i].y();
            }

            // SVD求解最小二乘问题
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
            Eigen::Vector3d solution = svd.matrixV().col(2);

            // 归一化法向量
            Eigen::Vector3d normal(solution[0], solution[1], 0.0);
            double norm = normal.norm();
            if (norm < 1e-6)
                return Eigen::Vector4d::Zero();

            normal /= norm;
            double d = -solution[0] * x_sum - solution[1] * y_sum;
            d /= samples.size();
            return Eigen::Vector4d(normal.x(), normal.y(), 0.0, d);
        }

        double computeError(const DataPoint &pt, const Model &model) const
        {
            return std::abs(model[0] * pt.x() + model[1] * pt.y() + model[3]);
        }
    };
}
#endif