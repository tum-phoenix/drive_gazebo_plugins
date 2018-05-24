#include <drive_gazebo_plugins/sign_label_plugin.h>

// used in attempts to visualize label output
//#include <sensor_msgs/fill_image.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <sensor_msgs/image_encodings.h>

bool getScreenspaceCoords(Ogre::AxisAlignedBox* AABB, Ogre::Camera* camera, Ogre::Vector2& result_top, Ogre::Vector2& result_bottom)
{
  Ogre::Vector3 point_top = (AABB->getCorner(Ogre::AxisAlignedBox::FAR_LEFT_TOP) +
                             AABB->getCorner(Ogre::AxisAlignedBox::FAR_RIGHT_TOP)) / 2;
  Ogre::Vector3 point_bottom = (AABB->getCorner(Ogre::AxisAlignedBox::NEAR_RIGHT_BOTTOM) +
                                AABB->getCorner(Ogre::AxisAlignedBox::NEAR_LEFT_BOTTOM)) / 2;

   // Is the camera facing that point? If not, return false
   Ogre::Plane cameraPlane = Ogre::Plane(Ogre::Vector3(camera->getDerivedOrientation().zAxis()), camera->getDerivedPosition());
   if(cameraPlane.getSide(point_top) != Ogre::Plane::NEGATIVE_SIDE || cameraPlane.getSide(point_bottom) != Ogre::Plane::NEGATIVE_SIDE)
      return false;

   // Transform the 3D point into screen space
   point_top = camera->getProjectionMatrix() * (camera->getViewMatrix() * point_top);
   point_bottom = camera->getProjectionMatrix() * (camera->getViewMatrix() * point_bottom);

   // Transform from coordinate space [-1, 1] to [0, 1] and update in-value
   result_top.x = (point_top.x / 2) + 0.5f;
   result_top.y = 1 - ((point_top.y / 2) + 0.5f);

   result_bottom.x = (point_bottom.x / 2) + 0.5f;
   result_bottom.y = 1 - ((point_bottom.y / 2) + 0.5f);

   return true;
}

namespace gazebo {

using namespace common;
using namespace math;

SignLabelPlugin::SignLabelPlugin():
  image_counter_(0),
  output_folder_("/tmp/driving"),
  sign_pixel_area_threshold_(10),
  signs_in_scene_()
{

}

SignLabelPlugin::~SignLabelPlugin()
{
  csv_file_.close();
}

void SignLabelPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  CameraPlugin::Load(_parent, _sdf);
  // copying from CameraPlugin into GazeboRosCameraUtils
  this->parentSensor_ = this->parentSensor;
  this->width_ = this->width;
  this->height_ = this->height;
  this->depth_ = this->depth;
  this->format_ = this->format;

  this->image_connect_count_ = boost::shared_ptr<int>(new int);
  *this->image_connect_count_ = 0;
  this->image_connect_count_lock_ = boost::shared_ptr<boost::mutex>(new boost::mutex);
  this->was_active_ = boost::shared_ptr<bool>(new bool);
  *this->was_active_ = false;

  LoadImpl(_parent, _sdf);
  GazeboRosCameraUtils::Load(_parent, _sdf);

  if (!_sdf->HasElement("sign_area_threshold"))
  {
      ROS_WARN_STREAM("Unable to get parameter 'sign_area_threshold', using default "<<sign_pixel_area_threshold_);
  }
  else if (!_sdf->GetElement("sign_area_threshold")->GetValue()->Get<int>(sign_pixel_area_threshold_)) {
    ROS_WARN_STREAM("Unable to cast parameter 'sign_area_threshold' to int, using default "<<sign_pixel_area_threshold_);
  }

  if (!_sdf->HasElement("output_folder"))
  {
      ROS_WARN_STREAM("Unable to get parameter 'output_folder', using default "<<output_folder_);
  }
  else output_folder_ = _sdf->GetElement("output_folder")->GetValue()->GetAsString();
  if (!(output_folder_.size() >= 1 && output_folder_.compare(output_folder_.size() - 1, 1, "/") == 1))
    output_folder_ += "/";
  ROS_INFO_STREAM("Output folder set to "<<output_folder_);

  csv_file_.open(output_folder_+"ground_truth.csv");
  csv_file_ << "Filename,Width,Height,Roi.X1,Roi.Y1,Roi.X2,Roi.Y2,ClassId \n";

  Json::Reader reader;
  if (!_sdf->HasElement("signs_in_world"))
  {
      ROS_ERROR_STREAM("Unable to get parameter 'signs_in_world', shutting down");
      ros::shutdown();
  }
  else if (!reader.parse(_sdf->GetElement("signs_in_world")->GetValue()->GetAsString(), signs_in_scene_)) {
    ROS_ERROR_STREAM("Unable to parse parameter 'signs_in_world', shutting down");
    ros::shutdown();
  }

  ROS_INFO("Succesfully loaded TrafficSignLabeler plugin!");
}

void SignLabelPlugin::LoadImpl(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  this->camera_ = this->CameraPlugin::camera;
}

void SignLabelPlugin::OnNewFrame(const unsigned char *_image, unsigned int _width, unsigned int _height, unsigned int _depth, const std::__cxx11::string &_format)
{
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

#if (GAZEBO_MAJOR_VERSION > 6)
  this->sensor_update_time_ = this->parentSensor_->LastUpdateTime();
#else
  this->sensor_update_time_ = this->parentSensor_->GetLastUpdateTime();
#endif

  if (!this->parentSensor->IsActive())
  {
    if ((*this->image_connect_count_) > 0)
      // do this first so there's chance for sensor to run 1 frame after activate
      this->parentSensor->SetActive(true);
  }

  // all attempts to display the images segfault :(
//  {
//    cv::namedWindow("Signs found in image", CV_WINDOW_NORMAL);
//    boost::mutex::scoped_lock lock(this->lock_);
//    sensor_msgs::Image copy_msg;
//    copy_msg.header.frame_id = this->frame_name_;
//    copy_msg.header.stamp.sec = this->sensor_update_time_.sec;
//    copy_msg.header.stamp.nsec = this->sensor_update_time_.nsec;
//    ROS_INFO_STREAM("type "<<this->type_<<" skip: "<<this->skip_<<" width "<<this->width_<<" height: "<<this->height_);
//    fillImage(copy_msg, this->type_, this->height_, this->width_,
//              this->skip_*this->width_, reinterpret_cast<const void*>(_image));
//    ROS_INFO("After filling");
//    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(copy_msg, sensor_msgs::image_encodings::BGR8);
//    ROS_INFO("After copying");
//  }

  bool sign_found = false;
  std::string filename = output_folder_ + std::to_string(image_counter_) + ".bmp";

//  std::vector<std::string> traffic_sign_names_;
//  traffic_sign_names_.push_back("Sign/-13");
//  traffic_sign_names_.push_back("Sign/-18");
//  traffic_sign_names_.push_back("Sign/-23");
//  traffic_sign_names_.push_back("Sign/-33");
//  traffic_sign_names_.push_back("Sign/-43");
//  traffic_sign_names_.push_back("Sign/-55");
  // get all visible objects
  gazebo::rendering::ScenePtr scene = CameraPlugin::parentSensor->Camera()->GetScene();
  for (const std::string& object_name : signs_in_scene_.getMemberNames()) {
    if (CameraPlugin::parentSensor->Camera()->IsVisible(object_name)) {
      rendering::VisualPtr visual = scene->GetVisual(object_name);
      ignition::math::Box bbox = visual->BoundingBox();
      Ogre::AxisAlignedBox box;
      box.setMinimum(bbox.Min().X(), bbox.Min().Y(), bbox.Min().Z());
      box.setMaximum(bbox.Max().X(), bbox.Max().Y(), bbox.Max().Z());
      box.transformAffine(visual->GetSceneNode()->_getFullTransform());
      Ogre::Vector2 top_point, bottom_point;
      cv::Point2d bottom_pixel, top_pixel;
      if (getScreenspaceCoords(&box, this->parentSensor->Camera()->OgreCamera(), top_point, bottom_point)) {
          bottom_pixel.x = bottom_point.x*_width;
          bottom_pixel.y = bottom_point.y*_height;
          top_pixel.x = top_point.x*_width;
          top_pixel.y = top_point.y*_height;
      }

      cv::Rect image_region(bottom_pixel, top_pixel);
      ROS_DEBUG_STREAM("Object "<<object_name<<" top pixel "<<top_pixel<<" bottom pixel: "<<bottom_pixel);

      if (image_region.size().area() > sign_pixel_area_threshold_) {
        csv_file_ << std::to_string(image_counter_) + ".bmp" << "," << _width << ","<< _height << ","<<
                     (int)bottom_pixel.x << ","<< (int)bottom_pixel.y << "," << (int)top_pixel.x << "," << (int)top_pixel.y <<
                     "," << signs_in_scene_[object_name] << "\n";
        sign_found = true;
      }
    }
  }

  if (sign_found) {
    this->parentSensor->Camera()->SaveFrame(_image, _width, _height, _depth, format, filename);
    image_counter_++;
  }

  this->PutCameraData(_image);
  this->PublishCameraInfo();

  // all attempts to display segfault :(
//  cv::imshow("Signs found in image", camera_image);
//  cv::waitKey(1);
}

void SignLabelPlugin::OnNewImageFrame(const unsigned char *_image, unsigned int _width, unsigned int _height,
                                                  unsigned int _depth, const std::__cxx11::string &_format)
{
   OnNewFrame(_image, _width, _height, _depth, _format);
}

}
