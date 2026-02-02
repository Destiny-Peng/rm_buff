#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <interfaces/msg/rune.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <termios.h>
#include <unistd.h>
#include <cstdint>
#include <cmath>
#include <random>
#include <vector>
#include <string>
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "interfaces/msg/target.hpp"
#include "interfaces/msg/readpack.hpp"
#include "interfaces/msg/rune.hpp"
#include "interfaces/msg/pose.hpp"
#include "ballistic_solver.hpp"
#include "rune_parameters.hpp"
#include "base_class.hpp"
namespace rune
{
  std::default_random_engine random_engine(time(0));
  double a = 0.913;
  double w = 1.942;
  Eigen::Vector3d virtual_rune_center(7400, 500.0, STANDARD_HEIGHT * 1000.); // 单位mm
  double energy_yaw = 0.0;
  cv::Mat camera_matrix;
  cv::Mat distortion_coefficients;

  double X_OFFSET = 0.0;
  double Y_OFFSET = 0.0;
  double Z_OFFSET = 0.0;

  class VirtualRune
  {
  private:
    Eigen::Matrix<double, 3, 4> single_fan_world{};
    Eigen::Matrix<double, 2, 4> single_fan_camera{};
    std::vector<cv::Point3f> points_base_world;
    std::vector<cv::Point2f> points_base_camera;
    std::vector<int> fans_status_;
    int inactive_index;
    double angle;
    double angle_start_time;
    double status_time_stamp;
    int rand_cnt;
    int active_cnt{};

    Eigen::Matrix<double, 3, 4> get_fan_3d(double angle)
    {
      return (Eigen::AngleAxisd(energy_yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()).toRotationMatrix() * this->single_fan_world);
    }

    Eigen::Vector3d get_fan_center_3d(const Eigen::Matrix<double, 3, 4> &fan)
    {
      return (fan.col(0) + fan.col(1) + fan.col(2) + fan.col(3)) / 4. + virtual_rune_center;
    }

    void rotate_points(double angle_rad)
    {
      for (size_t i = 0; i < 5; i++)
      {
        if (this->fans_status_[i])
        {
          Eigen::Matrix<double, 3, 4> result = this->get_fan_3d(this->angle + i * 2 * M_PI / 5);
          // std::cout << "r:" << result << std::endl;
          for (size_t j = 0; j < result.cols(); j++)
          {
            points_base_world.push_back([&result, j]()
                                        { return cv::Point3f(static_cast<float>(result(0, j) + virtual_rune_center[0]),
                                                             static_cast<float>(result(1, j) + virtual_rune_center[1]),
                                                             static_cast<float>(result(2, j) + virtual_rune_center[2])); }());
          }
        }
      }
      return;
    }

    double piv_angle_generate(double time, double a, double omiga)
    {
      // std::cout << "delta_t:" << time << std::endl;
      return -a / omiga * cos(omiga * time) + (2.09 - a) * time;
    }
    double const_angle_generate(double time)
    {
      return CV_PI / 3 * time;
    }

    void reset_status()
    {
      this->fans_status_ = std::vector<int>(5, 0);
      std::uniform_int_distribution<int> distribution(0, 4);
      int index = distribution(random_engine);
      this->inactive_index = (this->inactive_index == index ? (index + 1) % 5 : index); // make sure the inactive index is different from the last one.
      this->fans_status_[inactive_index] = -1;
      this->active_cnt = 0;
    }

    void clear()
    {
      this->points_base_world.clear();
      this->points_base_camera.clear();
      this->reset_status();
    }

  public:
    VirtualRune(/* args */)
    {
      this->angle_start_time = rclcpp::Clock().now().seconds();
      this->angle = 0;
      this->status_time_stamp = rclcpp::Clock().now().seconds();
      this->rand_cnt = 0;
      this->single_fan_world << 0.0, 0.0, 0.0, 0.0,
          160., -160., -186., 186.,
          154 + STANDARD_RUNE_RADIUS * 1000., 154 + STANDARD_RUNE_RADIUS * 1000., -154 + STANDARD_RUNE_RADIUS * 1000., -154 + STANDARD_RUNE_RADIUS * 1000.;
      this->reset_status();
    }
    ~VirtualRune() {}

    bool judge_hit(const Eigen::Vector3d &bullet_pos)
    {
      // to simply the judgement, just divide to three step.
      //  1. check the x axis,if the bullet is in the range of the rune, then go to the next step, otherwise return false.
      if (fabs(bullet_pos[0] - virtual_rune_center[0] / 1000.) > 0.7)
      {
        return false;
      }
      Eigen::Vector3d center = this->get_fan_center_3d(this->get_fan_3d(this->angle + this->inactive_index * 2 * M_PI / 5)) / 1000.;
      RCLCPP_DEBUG(rclcpp::get_logger("virtual_rune"), "center:%f %f %f", center[0], center[1], center[2]);
      // 2. check the y and z axis,if the bullet is in the range of the armor center, if true, return true, otherwise go to the next step.
      double norm = (bullet_pos - center).norm();
      if (norm < 0.15 && fabs(bullet_pos[0] - center[0]) < 0.05)
      {
        // hit
        fans_status_[inactive_index] = 10 - static_cast<int>(norm / 0.015);

        if (this->active_cnt == 4)
        {
          std::cout << "score:" << fans_status_[0] + fans_status_[1] + fans_status_[2] + fans_status_[3] + fans_status_[4] << std::endl;
          // the last one is hit, so we need to sleep 1 second and then reset the rune.
          rclcpp::sleep_for(std::chrono::seconds(1));
          std::uniform_real_distribution<double> dis_a(0, 0.266);
          a = dis_a(random_engine) + 0.78;
          std::uniform_real_distribution<double> dis_w(0, 0.117);
          w = dis_w(random_engine) + 1.884;
          std::cout << "a:" << a << " w:" << w << std::endl;
          this->reset_status();
          this->active_cnt = 0;
          return true;
        }

        std::uniform_int_distribution<int> distribution(0, 4);
        this->inactive_index = distribution(random_engine);
        while (fans_status_[inactive_index])
        {
          inactive_index = (inactive_index + 1) % 5;
        }
        this->fans_status_[inactive_index] = -1;
        this->status_time_stamp = rclcpp::Clock().now().seconds();
        this->active_cnt++;
        std::cout << "hit" << std::endl;
        return true;
      }
      // 3. check the y and z axis,if the bullet hit other armor center, if true, reset the rune, and this step always return false.
      for (size_t i = 0; i < 4; i++)
      {
        // check the other 4 armor.
        center = this->get_fan_center_3d(this->get_fan_3d(this->angle + (this->inactive_index + i + 1) % 5 * 2 * M_PI / 5)) / 1000.;
        double norm = (bullet_pos - center).norm();
        if (norm < 0.15 && fabs(bullet_pos[0] - center[0]) < 0.05)
        {
          // hit, so we need to reset the rune.
          std::cout << "wrong hit" << std::endl;
          this->reset_status();
          break;
        }
      }
      return false;
    }
    void update_callback()
    {

      if (rclcpp::Clock().now().seconds() - this->status_time_stamp > 2.5)
      {
        // RCLCPP_INFO(this->get_logger(), "time: %lf,%d", time, static_cast<int>(time / 2.5));
        this->status_time_stamp = rclcpp::Clock().now().seconds();
        this->reset_status();
      }

      // 大符
      this->angle = piv_angle_generate(rclcpp::Clock().now().seconds() - this->angle_start_time, a, w);
      // 小符
      //  this->angle = const_angle_generate(rclcpp::Clock().now().seconds() - this->angle_start_time);

      // 静止
      // this->angle = 0;
      // std::cout << "angle:" << (this->angle + this->inactive_index * 2 * M_PI / 5) * 180 / M_PI << std::endl;
    }
    interfaces::msg::Rune get_msg(Eigen::Matrix3d R)
    {
      rotate_points(this->angle);
      cv::Mat r_mat, r_vec;
      cv::eigen2cv(R, r_mat);
      cv::Rodrigues(r_mat, r_vec);
      this->points_base_world.push_back(cv::Point3f(virtual_rune_center[0], virtual_rune_center[1], virtual_rune_center[2]));
      cv::projectPoints(this->points_base_world, r_vec, cv::Mat::zeros(3, 1, CV_64F), camera_matrix, distortion_coefficients, this->points_base_camera);
      interfaces::msg::Rune msg;
      msg.r_center.x = this->points_base_camera.end()->x;
      msg.r_center.y = this->points_base_camera.end()->y;
      msg.r_center.z = 0;
      this->points_base_camera.erase(this->points_base_camera.end());

      std::vector<interfaces::msg::Point> tep_vpoint;
      interfaces::msg::Point temp_p;
      for (auto point : this->points_base_camera)
      {
        temp_p.x = point.x;
        temp_p.y = point.y;
        tep_vpoint.push_back(temp_p);
        if (tep_vpoint.size() == 4)
        {
          interfaces::msg::VPoint tep_vpoint_msg;
          tep_vpoint_msg.set__vpoint(tep_vpoint);
          msg.vvpoint.push_back(tep_vpoint_msg);
          tep_vpoint.clear();
        }
      }
      std::cout << std::endl;
      msg.flag_detect_success = true;
      msg.flag_all_activate = false;
      return msg;
    }
    interfaces::msg::Rune get_msg(const interfaces::msg::Readpack &readpack_msg, double noise_average, double noise_standard_deviation)
    {
      rotate_points(this->angle);
      cv::Mat r_mat, r_vec;
      Eigen::Vector3d euler_angle_a(-readpack_msg.pose.ptz_roll * CV_PI / 180.0, -readpack_msg.pose.ptz_pitch * CV_PI / 180.0, (readpack_msg.pose.ptz_yaw - 0.0) * CV_PI / 180.0);
      RCLCPP_DEBUG(rclcpp::get_logger("virtual_rune"), "euler_angle_a: %lf %lf %lf", euler_angle_a[0], euler_angle_a[1], euler_angle_a[2]);
      Eigen::Matrix3d rotate_mat_world_to_camera;
      rotate_mat_world_to_camera = Eigen::AngleAxisd(euler_angle_a[2], Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(euler_angle_a[1], Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(euler_angle_a[0], Eigen::Vector3d::UnitX());

      // 然后绕z轴和x轴分别转-90,得到y轴朝下相机坐标系
      Eigen::Vector3d euler_angle_b(-M_PI_2, 0, -M_PI_2);
      Eigen::Matrix3d rotate_mat_world_to_camera_b;
      rotate_mat_world_to_camera_b = Eigen::AngleAxisd(euler_angle_b[2], Eigen::Vector3d::UnitZ()) *
                                     Eigen::AngleAxisd(euler_angle_b[1], Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(euler_angle_b[0], Eigen::Vector3d::UnitX());

      // 然后只需要将两个旋转矩阵按照顺序右乘即可得到相机坐标系到世界坐标系的旋转矩阵。
      rotate_mat_world_to_camera = rotate_mat_world_to_camera * rotate_mat_world_to_camera_b;
      cv::eigen2cv(Eigen::Matrix3d(rotate_mat_world_to_camera.transpose()), r_mat);
      cv::Rodrigues(r_mat, r_vec);
      cv::Mat t_vec = cv::Mat::zeros(3, 1, CV_64F);
      cv::eigen2cv(Eigen::Vector3d(-X_OFFSET, -Y_OFFSET, -Z_OFFSET), t_vec);
      this->points_base_world.push_back(cv::Point3f(virtual_rune_center[0], virtual_rune_center[1], virtual_rune_center[2]));
      cv::projectPoints(this->points_base_world, r_vec, t_vec, camera_matrix, distortion_coefficients, this->points_base_camera);

      // add noise
      if (noise_average > 0.0 || noise_standard_deviation > 0.0)
      {
        std::normal_distribution<double> n(noise_average, noise_standard_deviation * noise_standard_deviation);
        for (auto &point : this->points_base_camera)
        {
          point.x += n(random_engine);
          point.y += n(random_engine);
        }
      }

      interfaces::msg::Rune msg;
      msg.r_center.x = this->points_base_camera.back().x;
      msg.r_center.y = this->points_base_camera.back().y;
      msg.r_center.z = 0;
      // std::cout << this->points_base_camera.back() << std::endl;
      this->points_base_camera.pop_back();

      std::vector<interfaces::msg::Point> tep_vpoint;
      interfaces::msg::Point temp_p;
      for (auto point : this->points_base_camera)
      {
        temp_p.x = point.x;
        temp_p.y = point.y;
        tep_vpoint.push_back(temp_p);
        if (tep_vpoint.size() == 4)
        {
          interfaces::msg::VPoint tep_vpoint_msg;
          tep_vpoint_msg.set__vpoint(tep_vpoint);
          msg.vvpoint.push_back(tep_vpoint_msg);
          tep_vpoint.clear();
        }
      }
      // std::cout << "size:" << this->points_base_camera.size() << std::endl;
      msg.flag_detect_success = true;
      msg.flag_all_activate = false;
      msg.pose = readpack_msg.pose;
      msg.energy_yaw = 0.0;
      msg.stamp = rclcpp::Clock().now();
      msg.visual_time = rclcpp::Clock().now().seconds();
      this->points_base_camera.clear();
      this->points_base_world.clear();
      return msg;
    }
    visualization_msgs::msg::MarkerArray get_rune_visualize_msg()
    {
      visualization_msgs::msg::MarkerArray msg;
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = rclcpp::Clock().now();
      marker.ns = "virtual_rune";
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = STANDARD_RUNE_RADIUS;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.color.a = 1.0;
      // 箭头部分
      marker.type = visualization_msgs::msg::Marker::ARROW;
      for (size_t i = 0; i < 5; i++)
      {
        marker.id = i;

        // 计算旋转四元数
        Eigen::Quaterniond orientation;
        orientation.setFromTwoVectors(Eigen::Vector3d::UnitX(), Eigen::AngleAxisd(energy_yaw, Eigen::Vector3d::UnitZ()) * Eigen::Vector3d(0, cos(M_PI_2 + angle + i * M_PI * 2 / 5.), sin(M_PI_2 + angle + i * M_PI * 2 / 5.)));

        marker.pose = tf2::toMsg(Eigen::Isometry3d(orientation));
        marker.pose.position.x = virtual_rune_center[0] / 1000.;
        marker.pose.position.y = virtual_rune_center[1] / 1000.;
        marker.pose.position.z = virtual_rune_center[2] / 1000.;
        switch (this->fans_status_[i])
        {
        case 0:
          marker.color.r = 0.0;
          marker.color.g = 1.0;
          marker.color.b = 0.0;
          break;
        case -1:
          marker.color.r = 1.0;
          marker.color.g = 0.0;
          marker.color.b = 0.0;
          break;
        default:
          marker.color.r = 0.0;
          marker.color.g = 0.0;
          marker.color.b = 1.0;
          break;
        }
        msg.markers.push_back(marker);
      }
      // 文字部分
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.scale.z = 0.5;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      for (size_t i = 0; i < 5; i++)
      {
        marker.id = i + 5;
        // marker.text = std::to_string(i);
        marker.text = std::to_string(this->fans_status_[i]);
        Eigen::Vector3d center = this->get_fan_center_3d(this->get_fan_3d(this->angle + i * 2 * M_PI / 5)) / 1000.;
        marker.pose.position.x = center.x();
        marker.pose.position.y = center.y();
        marker.pose.position.z = center.z();
        msg.markers.push_back(marker);
      }
      return msg;
    }
  };

  class Rune_img_publisher : public rclcpp::Node
  {
  public:
    cv::Mat img;
    sensor_msgs::msg::Image msg;
    Rune_img_publisher(std::string name) : Node(name)
    {
      img_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("rune_debug_image", 1);
      img = cv::imread("/home/jacy/project/workspace/herocv-2024-ver-ros/src/buff/src/t.png");
      // cv::resize(img, img, cv::Size(640, 480));
      auto msg_ptr = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
      msg = *msg_ptr.get();
      timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&Rune_img_publisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      RCLCPP_INFO(this->get_logger(), "Publishing");
      img_publisher_->publish(this->msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher_;
  };

  class Rune_detect_result_publisher : public rclcpp::Node
  {
  public:
    Rune_detect_result_publisher(std::string name) : Node(name)
    {
      rune_publisher_ = this->create_publisher<interfaces::msg::Rune>("rune_debug_detect_result", 1);
      timer_ = this->create_wall_timer(std::chrono::milliseconds(12), std::bind(&Rune_detect_result_publisher::timer_callback, this));
      rune_update_timer_ = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&Rune_detect_result_publisher::rune_update_callback, this));
      calc_timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&Rune_detect_result_publisher::calc_bullet_callback, this));
      readpack_subscriber_ = this->create_subscription<interfaces::msg::Readpack>("Readpack", 1, std::bind(&Rune_detect_result_publisher::readpack_callback, this, std::placeholders::_1));
      GT_fan_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("GT_fan", 1);
      bullet_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("bullet", 1);
      param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
      this->points_base.clear();

      this->declare_parameter("g_camera_param_path", "src/config/camera_param9.xml");
      cv::FileStorage camera_param_file;
      camera_param_file.open(this->get_parameter("g_camera_param_path").as_string(), cv::FileStorage::READ);
      if (!camera_param_file.isOpened())
      {
        RCLCPP_INFO(this->get_logger(), "open camera_param.yaml failed.");
        abort();
      }
      camera_param_file["camera_internal_matrix"] >> camera_matrix;
      camera_param_file["distortion_coeff"] >> distortion_coefficients;

      this->declare_parameter("g_tune_param_path", "src/config/tune_param5.xml");
      std::string g_tune_param_path = this->get_parameter("g_tune_param_path").as_string();
      cv::FileStorage tune_param_file;
      tune_param_file.open(g_tune_param_path, cv::FileStorage::READ);
      if (!tune_param_file.isOpened())
      {
        RCLCPP_INFO(this->get_logger(), "open tune_param.yaml failed.%s", g_tune_param_path.c_str());
        abort();
      }
      tune_param_file["X_OFFSET"] >> X_OFFSET;
      tune_param_file["Y_OFFSET"] >> Y_OFFSET;
      tune_param_file["Z_OFFSET"] >> Z_OFFSET;

      this->declare_parameter("judge_hit", true);
      this->declare_parameter("noise_average", 0.0);
      this->declare_parameter("noise_standard_deviation", 0.0);
      this->declare_parameter("energy_yaw", 0.0);
      this->declare_parameter("center_x", 7.4);
      this->declare_parameter("center_y", 0.5);
      this->declare_parameter("center_z", 1.075);
      this->declare_parameter("car_id", 3);
      this->declare_parameter("a", 0.913);
      this->declare_parameter("w", 1.942);
      std::cout << this->get_parameter("car_id").as_int() << std::endl;
      this->judge_hit = this->get_parameter("judge_hit").as_bool();
      auto judge_hit_callback = [this](const rclcpp::Parameter &p)
      {
        this->judge_hit = p.as_bool();
      };
      this->noise_average = this->get_parameter("noise_average").as_double();
      auto noise_average_callback = [this](const rclcpp::Parameter &p)
      {
        this->noise_average = p.as_double();
      };
      this->noise_standard_deviation = this->get_parameter("noise_standard_deviation").as_double();
      auto noise_standard_deviation_callback = [this](const rclcpp::Parameter &p)
      {
        this->noise_standard_deviation = p.as_double();
      };
      energy_yaw = this->get_parameter("energy_yaw").as_double();
      auto energy_yaw_callback = [this](const rclcpp::Parameter &p)
      {
        energy_yaw = p.as_double();
      };
      a = this->get_parameter("a").as_double();
      auto a_callback = [this](const rclcpp::Parameter &p)
      {
        std::cout << "get a" << p.as_double() << std::endl;
        a = p.as_double();
      };
      w = this->get_parameter("w").as_double();
      auto w_callback = [this](const rclcpp::Parameter &p)
      {
        std::cout << "get w" << p.as_double() << std::endl;
        w = p.as_double();
      };
      this->judge_hit_cb_handle_ = param_subscriber_->add_parameter_callback("judge_hit", judge_hit_callback);
      this->noise_average_cb_handle_ = param_subscriber_->add_parameter_callback("noise_average", noise_average_callback);
      this->noise_standard_deviation_cb_handle_ = param_subscriber_->add_parameter_callback("noise_standard_deviation", noise_standard_deviation_callback);
      this->energy_yaw_cb_handle_ = param_subscriber_->add_parameter_callback("energy_yaw", energy_yaw_callback);
      this->a_cb_handle_ = param_subscriber_->add_parameter_callback("a", a_callback);
      this->w_cb_handle_ = param_subscriber_->add_parameter_callback("w", w_callback);
      virtual_rune_center = Eigen::Vector3d(this->get_parameter("center_x").as_double(), this->get_parameter("center_y").as_double(), this->get_parameter("center_z").as_double()) * 1000.;
      std::cout << "virtual_rune_center: " << virtual_rune_center.transpose() << std::endl;
    }

    void rotate_points(double angle_rad, std::vector<Eigen::Vector2d> &points)
    {
      points.clear();
      Eigen::Matrix2d rotation_matrix;
      rotation_matrix << cos(angle_rad), -sin(angle_rad), sin(angle_rad), cos(angle_rad);
      for (auto &point : this->points_base)
      {
        Eigen::Vector2d vec(point - this->center);
        // 像素坐标系x向右y向下，所以需要先镜像翻转一下。
        vec.y() = -vec.y();
        Eigen::Vector2d result = rotation_matrix * vec + Eigen::Vector2d(this->center.x(), 480 - this->center.y());
        // 这里需要再镜像翻转一次，因为是像素坐标系。
        result.y() = 480 - result.y();
        points.push_back(result);
        // std::cout << "point: " << result.transpose() << std::endl;
      }
    }
    interfaces::msg::Rune get_msg(std::vector<Eigen::Vector2d> &points)
    {
      interfaces::msg::Rune msg;
      std::vector<interfaces::msg::Point> tep_vpoint;
      interfaces::msg::Point temp_p;
      for (auto &point : points)
      {
        temp_p.x = point.x();
        temp_p.y = point.y();
        tep_vpoint.push_back(temp_p);
      }
      interfaces::msg::VPoint tep_vpoint_msg;
      tep_vpoint_msg.set__vpoint(tep_vpoint);
      msg.vvpoint.push_back(tep_vpoint_msg);
      msg.flag_detect_success = true;
      msg.flag_all_activate = false;
      msg.energy_yaw = 2.6;
      msg.pose.ptz_yaw = 2.67;
      msg.pose.ptz_pitch = 5.56;
      msg.pose.ptz_roll = 0.27;
      msg.pose.visual_time = this->get_clock()->now().seconds();
      msg.r_center.x = -1;
      msg.r_center.y = -1;
      msg.r_center.z = 0;
      return msg;
    }

  private:
    void rune_update_callback()
    {
      if (this->last_msg.mode == Mode::MODE_SMALLRUNE || this->last_msg.mode == Mode::MODE_BIGRUNE)
      {
        this->v_rune.update_callback();
      }
    }
    void timer_callback()
    {
      if (this->last_msg.mode == Mode::MODE_SMALLRUNE || this->last_msg.mode == Mode::MODE_BIGRUNE)
      {
        // this->v_rune.update_callback();

        interfaces::msg::Rune msg = this->v_rune.get_msg(this->last_msg, this->noise_average, this->noise_standard_deviation);
        this->rune_publisher_->publish(msg);
        this->GT_fan_publisher->publish(this->v_rune.get_rune_visualize_msg());
      }
      // RCLCPP_INFO(this->get_logger(), "Publishing");
    }

    void readpack_callback(const interfaces::msg::Readpack::SharedPtr msg)
    {
      // judge fire
      if (this->last_msg.mode != Mode(msg->mode) || msg->rightclick)
      {
        this->clear();
      }
      if (this->last_msg.mode == Mode::MODE_SMALLRUNE || this->last_msg.mode == Mode::MODE_BIGRUNE)
      {
        if (msg->fired != this->last_msg.fired)
        {
          // fire
          this->bullets_list.push_back(bullet(std::chrono::system_clock::now(), msg->pose.ptz_pitch * M_PI / 180, msg->pose.ptz_yaw * M_PI / 180, msg->bullet_speed, 0, 0));
        }
      }
      this->last_msg = *msg;

      // add the bullet into the list
    }

    void calc_bullet_callback()
    {
      // calculate all the bullet in the list
      for (auto bullet_iterator = this->bullets_list.begin(); bullet_iterator != this->bullets_list.end(); ++bullet_iterator)
      {
        bullet_iterator->calc_a_step();
        Eigen::Vector3d pos = bullet_iterator->get_state_vec_rotated();
        if (pos[0] > 7.5)
        {
          // std::cout << pos.transpose() << std::endl;
          // bullet fly out of the field
          this->bullets_list.erase(bullet_iterator);
          // Check if the list is empty after the erase operation
          if (bullets_list.empty())
          {
            break;
          }
          continue;
        }

        // judge if there is a bullet hit the target
        if (this->judge_hit)
        {
          if (this->v_rune.judge_hit(pos))
          {
            this->bullets_list.erase(bullet_iterator);
          }
        }
        // Check if the list is empty after the erase operation
        if (bullets_list.empty())
        {
          break;
        }
      }
      if (!this->bullets_list.empty())
      {
        this->bullet_publisher->publish(this->get_bullet_visualize_msg());
      }
    }

    visualization_msgs::msg::MarkerArray get_bullet_visualize_msg()
    {
      visualization_msgs::msg::MarkerArray msg;
      builtin_interfaces::msg::Time stamp_ = rclcpp::Clock().now();
      for (size_t i = 0; i < this->bullets_list.size(); i++)
      {
        msg.markers.push_back(this->bullets_list.at(i).get_visualize_msg(i, stamp_));
      }
      return msg;
    }

    void clear()
    {

      this->bullets_list.clear();
      this->points_base.clear();
      this->center = Eigen::Vector2d(0, 0);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr rune_update_timer_;
    rclcpp::TimerBase::SharedPtr calc_timer_;
    rclcpp::Publisher<interfaces::msg::Rune>::SharedPtr rune_publisher_;
    rclcpp::Subscription<interfaces::msg::Readpack>::SharedPtr readpack_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr GT_fan_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bullet_publisher;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> judge_hit_cb_handle_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> noise_average_cb_handle_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> noise_standard_deviation_cb_handle_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> energy_yaw_cb_handle_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> a_cb_handle_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> w_cb_handle_;
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    interfaces::msg::Readpack last_msg;
    VirtualRune v_rune;

    std::vector<bullet> bullets_list;
    std::vector<Eigen::Vector2d> points_base;
    Eigen::Vector2d center;

    bool judge_hit = true;
    double noise_average = 0.0;
    double noise_standard_deviation = 0.0;
  };
}
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rune::Rune_detect_result_publisher>("virtual_rune");
  // auto node = std::make_shared<Rune_img_publisher>("virtual_rune");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
