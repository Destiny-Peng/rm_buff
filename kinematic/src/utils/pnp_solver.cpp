#include "utils/pnp_solver.hpp"

namespace rune
{
    PnPSolver::PnPSolver(const cv::Mat &camera_matrix,
                         const cv::Mat &distortion_coefficients,
                         cv::SolvePnPMethod method)
        : camera_matrix_(camera_matrix), distortion_coefficients_(distortion_coefficients), method_(method) {}

    void PnPSolver::setObjectPoints(const std::string &coord_frame_name,
                                    const std::vector<cv::Point3d> &object_points) noexcept
    {
        object_points_map_[coord_frame_name] = object_points;
    }

    float PnPSolver::calculateDistanceToCenter(const cv::Point2f &image_point) const noexcept
    {
        float cx = camera_matrix_.at<double>(0, 2);
        float cy = camera_matrix_.at<double>(1, 2);
        return cv::norm(image_point - cv::Point2f(cx, cy));
    }

    Eigen::VectorXd PnPSolver::getPose(const cv::Mat &rvec, const cv::Mat &tvec) noexcept
    {
        auto pose = Eigen::VectorXd(6);
        cv::Mat rmat;
        cv::Rodrigues(rvec, rmat);

        float yaw, pitch, roll;
        yaw = std::asin(-rmat.at<double>(2, 0));
        pitch = std::atan2(rmat.at<double>(2, 1), rmat.at<double>(2, 2));
        roll = std::atan2(rmat.at<double>(1, 0), rmat.at<double>(0, 0));

        pose << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2), yaw, pitch, -roll;
        return pose;
    }

}