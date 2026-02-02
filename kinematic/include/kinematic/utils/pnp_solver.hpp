#ifndef PNP_SOLVER_HPP
#define PNP_SOLVER_HPP
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "base_class.hpp"

namespace rune
{
    // Rune object points
    // r_tag, bottom_left, top_left, top_right, bottom_right
    const std::vector<cv::Point3d> RUNE_OBJECT_POINTS = {cv::Point3d(-160, 154, 0), cv::Point3d(160, 154, 0), cv::Point3d(186, -154, 0), cv::Point3d(-186, -154, 0)};
    const std::vector<cv::Point3d> RUNE_OBJECT_POINTS_INTERPID = {cv::Point3d(-160, 154, 0), cv::Point3d(160, 154, 0), cv::Point3d(186, -154, 0), cv::Point3d(-186, -154, 0),
                                                                  cv::Point3d(0, 154, 0), cv::Point3d(173, 0, 0), cv::Point3d(0, -154, 0), cv::Point3d(-173, 0, 0), cv::Point3d(0, 0, 0)};
    const std::vector<cv::Point3d> RUNE_OBJECT_POINTS_WITH_R = {cv::Point3d(-160, 154, 0), cv::Point3d(160, 154, 0), cv::Point3d(186, -154, 0), cv::Point3d(-186, -154, 0), cv::Point3d(0, -700, 0)};
    const std::vector<cv::Point3d> RUNE_OBJECT_POINTS_WITH_R_INTERPID = {cv::Point3d(-160, 154, 0), cv::Point3d(160, 154, 0), cv::Point3d(186, -154, 0), cv::Point3d(-186, -154, 0), cv::Point3d(0, -700, 0),
                                                                         cv::Point3d(0, 154, 0), cv::Point3d(173, 0, 0), cv::Point3d(0, -154, 0), cv::Point3d(-173, 0, 0), cv::Point3d(0, 0, 0)};
    // coord with origin at the center of the rune
    // const std::vector<cv::Point3d> RUNE_OBJECT_POINTS = {cv::Point3d(-160, 854, 0), cv::Point3d(160, 854, 0), cv::Point3d(186, 546, 0), cv::Point3d(-186, 546, 0)};
    // const std::vector<cv::Point3d> RUNE_OBJECT_POINTS_WITH_R = {cv::Point3d(-160, 854, 0), cv::Point3d(160, 854, 0), cv::Point3d(186, 546, 0), cv::Point3d(-186, 546, 0), cv::Point3d(0, 0, 0)};
    class PnPSolver
    {
    public:
        PnPSolver(const cv::Mat &camera_matrix,
                  const cv::Mat &distortion_coefficients,
                  cv::SolvePnPMethod method = cv::SOLVEPNP_ITERATIVE);

        // Set an object coord system
        void setObjectPoints(const std::string &coord_frame_name,
                             const std::vector<cv::Point3d> &object_points) noexcept;

        // Get 3d position of the object coord system using PnP algorithm
        template <class InputArray>
        bool solvePnP(const InputArray &image_points,
                      cv::Mat &rvec,
                      cv::Mat &tvec,
                      const std::string &coord_frame_name)
        {
            if (auto it = object_points_map_.find(coord_frame_name); it != object_points_map_.end())
            {
                auto object_points = it->second;
                return cv::solvePnP(object_points,
                                    image_points,
                                    camera_matrix_,
                                    distortion_coefficients_,
                                    rvec,
                                    tvec,
                                    false,
                                    method_);
            }
            else
            {
                return false;
            }
        }

        // Calculate the distance between armor center and image center
        float calculateDistanceToCenter(const cv::Point2f &image_point) const noexcept;

        static Eigen::VectorXd getPose(const cv::Mat &rvec, const cv::Mat &tvec) noexcept;

    private:
        std::unordered_map<std::string, std::vector<cv::Point3d>> object_points_map_;
        cv::Mat camera_matrix_;
        cv::Mat distortion_coefficients_;
        cv::SolvePnPMethod method_ = cv::SOLVEPNP_ITERATIVE;
    };
}
#endif