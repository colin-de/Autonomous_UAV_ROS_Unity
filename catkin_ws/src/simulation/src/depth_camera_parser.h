#pragma once

#include <unordered_map>
#include <ros/ros.h>
#include "rgb_camera_parser.h"

class DepthCameraParser : public RGBCameraParser {
public:
  DepthCameraParser() = default;

protected: 

  virtual void ImageToRos(const ImageData& img, 
                          const std::string& frame, 
                          sensor_msgs::Image* img_msg) const override {
    
    img_msg->width = img.width;
    img_msg->height = img.height;
    img_msg->encoding = "16UC1";
    img_msg->header.frame_id = frame;
    img_msg->step = img.width * 2;

    const int pixels = img.width * img.height;
    img_msg->data = std::vector<uint8_t>(img.width * img.height * 2);

    auto img_data_ptr = img.data.get();
    
    float max_range_millimeters = GetMaxRange() * 1000;

    for(int i = 0; i < pixels; i++) {
      uint8_t r = img_data_ptr[3*i];
      uint8_t g = img_data_ptr[3*i+1];
      uint8_t b = img_data_ptr[3*i+2];
      float depth = 1.0 * static_cast<float>(r)/255.0f + static_cast<float>(g)/255.0f * 1.0f/255.0f + static_cast<float>(b)/255.0f * 1.0f/65025.0f;

      if (depth > 0.99f) {
        depth = 0.0f;
      }

      uint16_t int_millimeters = (uint16_t) (depth * max_range_millimeters);

      img_msg->data[2*i] = (int_millimeters << 8) >> 8;
      img_msg->data[2*i+1] = int_millimeters >> 8;
    }
  }
};