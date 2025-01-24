#include "rclcpp/rclcpp.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/pcl_config.h> 

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp> // удалить, нужен для эксперементов



/*

cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg_ptr, "bgr8");

параметры найти нужный
    "mono8"
    "bgr8"
    "bgra8"
    "rgb8"
    "rgba8"
    "mono16"

camera_instrinsics - это
Mat cameraMatrix: Матрица камеры, которая содержит параметры фокусного расстояния и координаты главной точки. Обычно она выглядит так:
[fx  0  cx]
[0  fy  cy]
[0  0  1]
где (fx, fy) — фокусные расстояния по осям x и y, а (cx, cy) — координаты главной точки.


distortion_coefficients
Mat distCoeffs: Вектор коэффициентов искажения, который включает параметры,
описывающие искажения объектива. Обычно это 5 или 8 значений, в зависимости от модели искажения.

*/
/*
%YAML 1.2
---
CalibrationDate: "2023-07-11 17:13:16.592488" 
CameraName: CF_short
CameraInfoTopicName: sensing/camera/CF_short/camera_optical_link/camera_info
ImageSize: [ 1920, 1080 ] 
IntrinsicMatrix: [1034.73762,    0.     ,  934.71608,
            0.     , 1046.20661,  488.41446,
            0.     ,    0.     ,    1.     ]
DistortionModel: "plumb_bob" 
Distortion: [-0.304284, 0.075382, -0.002566, -0.001795, 0.000000]
PixelSize: 3.0
ROI: [0.0, 0.0, 1.0, 1.0]
*/
namespace std {
  template <>
  class hash<cv::Point>{
  public :
    size_t operator()(const cv::Point &pixel_cloud ) const
    {
      return hash<std::string>()( std::to_string(pixel_cloud.x) + "|" + std::to_string(pixel_cloud.y) );
    }
  };
};


class CloudFusion : public rclcpp::Node
{
    public:
      CloudFusion() : Node("fusion_node")
    {
      InitCalibration();

      tf_buffer   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

      image_sub = this->create_subscription<sensor_msgs::msg::Image>(
    "/CF", rclcpp::QoS{1} , std::bind(&CloudFusion::callbackImage, this, std::placeholders::_1));

      cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/lidar_top/points", rclcpp::QoS{1} , std::bind(&CloudFusion::callbackCloud, this, std::placeholders::_1));

      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("CloudFusion_pub", 10);
    }

    private:

      void InitCalibration();

      tf2::Transform  FindTransform(const std::string &in_target_frame, const std::string &in_source_frame);
      Eigen::Vector3d TransformPoint(const pcl::PointXYZ &point);
    
      void callbackCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg_ptr);
      void callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr       image_msg_ptr);


      rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  cloud_sub;
      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr        image_sub;

      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr     publisher_;

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr                          cloud_rgb;

      cv::Mat                             current_frame;
      cv::Mat                             camera_instrinsics;
      cv::Mat                             distortion_coefficients;
      cv::Size                            image_size;

      cv::Mat                             in_image_q;  

      tf2::Transform                      camera_lidar_tf_;

      std::unique_ptr<tf2_ros::Buffer>            tf_buffer;
      std::shared_ptr<tf2_ros::TransformListener> transform_listener{nullptr};


      bool                                camera_info_ok;
      bool                                camera_lidar_tf_ok;
      std::string                         image_frame_id;


      Eigen::Vector3d                      transform_test;
      Eigen::Quaterniond                   quaternion_test;
      Eigen::Matrix<double, 3, 4>          projection_matrix_;
      Eigen::Matrix4d                      lidar_to_camera_matrix_;
      Eigen::Affine3d                      affine_lidar_to_camera_matrix;
      Eigen::Matrix<double, 3, 4>          lidar_to_camera_projection_matrix_;

};
