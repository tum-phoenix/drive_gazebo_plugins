#ifndef GAZEBO_GENERATOR_PLUGIN_H
#define GAZEBO_GENERATOR_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo_plugins/gazebo_ros_camera_utils.h>
#include <gazebo/common/common.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/Scene.hh>

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <jsoncpp/json/json.h>

namespace gazebo
{
  class SignLabelPlugin : public CameraPlugin, GazeboRosCameraUtils
  {
    public:
    SignLabelPlugin();
    ~SignLabelPlugin();

    /// \brief Load the plugin
    /// \param take in SDF root element
    void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
    void LoadImpl(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    protected:
    /// \brief Update the controller
    virtual void OnNewFrame(const unsigned char *_image,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth, const std::string &_format);

    /// \brief Update the controller
    virtual void OnNewImageFrame(const unsigned char *_image,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth, const std::string &_format);

    std::ofstream csv_file_;
    int image_counter_;
    std::string output_folder_;
    int sign_pixel_area_threshold_;
    Json::Value signs_in_scene_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(SignLabelPlugin)
}
#endif
