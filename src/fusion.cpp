#include "lidar_rgb_fusion/fusion.hpp"

//Функция калибровок камеры
//калибровки камеры CF для Весты взяты из Calibration Data
void CloudFusion::InitCalibration()
{
  camera_lidar_tf_ok = false;
  camera_info_ok = false;
  image_frame_id = "";

  image_size.height = 1024;
  image_size.width =  1280;

  camera_instrinsics = cv::Mat(3, 3, CV_64F);
  camera_instrinsics.at<double>(0, 0) = 2365.5; //fx  2495.5
  camera_instrinsics.at<double>(0, 1) = 0.0;
  camera_instrinsics.at<double>(0, 2) = 400.5;  //cx
  camera_instrinsics.at<double>(1, 0) = 0.0;
  camera_instrinsics.at<double>(1, 1) = 2394.5;  //fy 2504.5
  camera_instrinsics.at<double>(1, 2) = 658.5;   //cy 493.5  690.5
  camera_instrinsics.at<double>(2, 0) = 0.0;
  camera_instrinsics.at<double>(2, 1) = 0.0;
  camera_instrinsics.at<double>(2, 2) = 1.0;

  distortion_coefficients = cv::Mat(1, 5, CV_64F);
  distortion_coefficients.at<double>(0) =  -0.3787;
  distortion_coefficients.at<double>(1) =  0.11;
  distortion_coefficients.at<double>(2) =  0.0;
  distortion_coefficients.at<double>(3) =  0.0;
  distortion_coefficients.at<double>(4) =  0.0;


  projection_matrix_.setZero();
  projection_matrix_(0, 0) = camera_instrinsics.at<double>(0, 0);
  projection_matrix_(0, 2) = camera_instrinsics.at<double>(0, 2);
  projection_matrix_(1, 1) = camera_instrinsics.at<double>(1, 1);
  projection_matrix_(1, 2) = camera_instrinsics.at<double>(1, 2);
  projection_matrix_(2, 2) = 1;

  camera_info_ok = true;
}

//Получаем изображение с камеры и настраиваем его с помощью заданных калибровок
void CloudFusion::callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr image_msg_ptr)
{
  if (!camera_info_ok)
  {
    std::cout << "Waiting for Intrinsics to be available." << std::endl;
    return;
  }

  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg_ptr, "bgr8");
  in_image_q = cv_image->image;   

  cv::flip(in_image_q, in_image_q, 1); // разворот изображение 

  cv::undistort(in_image_q, current_frame, camera_instrinsics, distortion_coefficients); // калибровка

  image_frame_id = image_msg_ptr->header.frame_id;

/*
//Провека что мы получаем
  cv::namedWindow("Example", cv::WINDOW_AUTOSIZE);
  cv::imshow("Example", current_frame);
  cv::waitKey(1);
*/

}


//проецируем точки лидара на положение камеры
tf2::Transform CloudFusion::FindTransform(const std::string &in_target_frame, const std::string &in_source_frame)
{
  geometry_msgs::msg::TransformStamped t;

  tf2::Transform transform;

  camera_lidar_tf_ok = false;

  try
  {
  std::cout << "Lookup Transform source--> " <<  in_source_frame << " || target---> " << in_target_frame << std::endl;
  t = tf_buffer->lookupTransform(in_target_frame, in_source_frame, tf2::TimePointZero);

  transform_test = Eigen::Vector3d(t.transform.translation.x ,t.transform.translation.y , t.transform.translation.z);
  quaternion_test = Eigen::Quaterniond(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w);

  affine_lidar_to_camera_matrix.translation() = transform_test;
  affine_lidar_to_camera_matrix.linear() = quaternion_test.toRotationMatrix();

  lidar_to_camera_matrix_ = Eigen::Matrix4d::Identity();
  lidar_to_camera_matrix_ = affine_lidar_to_camera_matrix.matrix();

  lidar_to_camera_projection_matrix_ = projection_matrix_ * lidar_to_camera_matrix_.block<4, 4>(0, 0);

  transform.setOrigin(tf2::Vector3(t.transform.translation.x, 
                                   t.transform.translation.y, 
                                   t.transform.translation.z));

  std::cout << "ПОсмотрим на вращение -> " << t.transform.rotation.x << " " << t.transform.rotation.y 
            << " " << t.transform.rotation.z << " " << t.transform.rotation.w << std::endl;

  std::cout << "ПОсмотрим на tf -> " << t.transform.translation.x << " " << t.transform.translation.y 
            << " " << t.transform.translation.z << std::endl;  
  
  camera_lidar_tf_ok = true;
  }
  catch (const tf2::LookupException &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "LookupException: %s", ex.what());
  }

  return transform;
 }
 

Eigen::Vector3d CloudFusion::TransformPoint(const pcl::PointXYZ &point)
{

  Eigen::Vector4d P4d(point.x , point.y, point.z, 1.0);

  Eigen::Vector3d point3d_transformed_camera = lidar_to_camera_projection_matrix_ * P4d;

  return Eigen::Vector3d(point3d_transformed_camera[0] / point3d_transformed_camera[2],
                         point3d_transformed_camera[1] / point3d_transformed_camera[2],
                         point3d_transformed_camera[2]);
}


void CloudFusion::callbackCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg_ptr)
{

  if (current_frame.empty() || image_frame_id == "")
  {
    std::cout << " Waiting for Image frame to be available." << std::endl;
    return;
  }

  if (!camera_lidar_tf_ok)
  {
    camera_lidar_tf_ = FindTransform(image_frame_id, cloud_msg_ptr->header.frame_id);
  }

  if (!camera_info_ok || !camera_lidar_tf_ok)
  {
    std::cout << " Waiting for Camera-Lidar TF and Intrinsics to be available." << std::endl;
    return;
  }


  pcl::PointCloud<pcl::PointXYZ>::Ptr         in_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr      out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud_msg_ptr, *in_cloud);

  std::unordered_map<cv::Point, pcl::PointXYZ> projection_map;

  std::vector<pcl::PointXYZ> cam_cloud(in_cloud->points.size());

  for (size_t i = 0; i < in_cloud->points.size(); i++)
  {

  Eigen::Vector3d point2d_transformed_camera = TransformPoint((*in_cloud)[i]);

  double u = point2d_transformed_camera[0];
  double v = point2d_transformed_camera[1];

  pcl::PointXYZRGB                colored_3d_point;
       
    if ((u >= 0) && (u < image_size.width)
      && (v >= 0) && (v < image_size.height)
      && point2d_transformed_camera[2] < 0)
    {
      cv::Vec3b rgb_pixel = current_frame.at<cv::Vec3b>(v, u);

      colored_3d_point.x = in_cloud->points[i].x;
      colored_3d_point.y = in_cloud->points[i].y;
      colored_3d_point.z = in_cloud->points[i].z;
      colored_3d_point.r = rgb_pixel[2];
      colored_3d_point.g = rgb_pixel[1];
      colored_3d_point.b = rgb_pixel[0];
      out_cloud->points.push_back(colored_3d_point);

    }
    else
    {
      colored_3d_point.x = in_cloud->points[i].x;
      colored_3d_point.y = in_cloud->points[i].y;
      colored_3d_point.z = in_cloud->points[i].z;
      colored_3d_point.r = 65;
      colored_3d_point.g = 105;
      colored_3d_point.b = 225;
      out_cloud->points.push_back(colored_3d_point);
    }
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*out_cloud, cloud_msg);
  cloud_msg.header = cloud_msg_ptr->header;
  publisher_->publish(cloud_msg);
  out_cloud->points.clear();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudFusion>());
  rclcpp::shutdown();
  return 0;
}

/*
Оптический центр (или главная точка) — это точка в оптической системе, через которую проходят лучи света, не претерпевая отклонений.
 В контексте камеры, оптический центр соответствует точке на сенсоре, где изображение фокусируется.

    Координаты оптического центра: В матрице камеры (camera matrix) оптический центр обычно обозначается как cx​ и cy​. 
    Эти значения представляют собой координаты оптического центра на изображении, выраженные в пикселях.
     Например, если у вас есть изображение размером 640x480 пикселей, то оптический центр может находиться примерно в точке (320, 240).

    Влияние на искажения: Если оптический центр не совпадает с центром сенсора, это может привести к искажению изображения,
     особенно по краям. Коррекция этих искажений часто включает в себя правильное определение оптического центра.
*/

/*
Фокусное расстояние (focal length) — это расстояние от оптического центра до фокуса, где собираются лучи света, проходящие через объектив. 
Оно измеряется в миллиметрах (мм) и является критически важным параметром, определяющим угол зрения и масштаб изображения.

    Определение: Фокусное расстояние определяет, насколько "увеличенным" или "уменьшенным" будет изображение.
     Объектив с коротким фокусным расстоянием (например, 18 мм) будет иметь широкий угол обзора и будет подходить для съемки пейзажей,
      в то время как объектив с длинным фокусным расстоянием (например, 200 мм) будет иметь узкий угол обзора и будет идеален для съемки 
      удаленных объектов (например, дикой природы).

    Влияние на глубину резкости: 
    Фокусное расстояние также влияет на глубину резкости, то есть на диапазон расстояний, 
    в котором объекты будут выглядеть четкими. Объективы с длинным фокусным расстоянием имеют меньшую глубину резкости, 
    что позволяет выделить объект на фоне размытого фона.

*/