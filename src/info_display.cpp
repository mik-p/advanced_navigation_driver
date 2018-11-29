#include <fstream>

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv/cv.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

const float WIDTH = 640;
const float HEIGHT = 480;

const cv::Scalar RED(0, 0, 255, 255);
const cv::Scalar GREEN(0, 255, 0, 255);

string flag_to_string(const int flag) {
  if(flag == 0) {
    return "NO";
  } else {
    return "YES";
  }
}

cv::Scalar flag_to_color(const int flag) {
  if(flag == 0) {
    return RED;
  } else {
    return GREEN;
  }
}

typedef enum
{
    gnss_fix_none,
    gnss_fix_2d,
    gnss_fix_3d,
    gnss_fix_sbas,
    gnss_fix_differential,
    gnss_fix_omnistar,
    gnss_fix_rtk_float,
    gnss_fix_rtk_fixed
} gnss_fix_type_e;

string gnss_fix_to_string(const int fix) {
  switch(fix) {
    case gnss_fix_none:
      return "gnss_fix_none";
    case gnss_fix_2d:
      return "gnss_fix_2d";
    case gnss_fix_3d:
      return "gnss_fix_3d";
    case gnss_fix_sbas:
      return "gnss_fix_sbas";
    case gnss_fix_differential:
      return "gnss_fix_differential";
    case gnss_fix_omnistar:
      return "gnss_fix_omnistar";
    case gnss_fix_rtk_float:
      return "gnss_fix_rtk_float";
    case gnss_fix_rtk_fixed:
      return "gnss_fix_rtk_fixed";
    default:
      return "UNKNOWN";
  }
}

cv::Scalar gnss_fix_to_color(const int fix) {
  if(fix < gnss_fix_3d) {
    return RED;
  } else {
    return GREEN;
  }
}

float deg(const float rad) {
  return rad * 180.0 / M_PI;
}

cv::Scalar err_to_rgb(const float error, const float max_error) {
  float rel_error = MIN(error, max_error) / max_error;
  return cv::Scalar(0, 255*(1-rel_error), 255*rel_error, 255);
}

void publish_info_panel(image_transport::Publisher &display_pub, geometry_msgs::Vector3Stamped pose_errors_msg,
    float std_deviation_threshold, const int gnss_fix_type, const int heading_initialised, const int dual_antenna_heading_active) {
  cv::Mat panel(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(255,255,255,255));

  float roll_error = deg(pose_errors_msg.vector.x);
  float pitch_error = deg(pose_errors_msg.vector.y);

  float max_err = 3.0 * std_deviation_threshold;

  const float font_scale = 2.5;
  const float font_thickness = 3;
  const float space_top = 50;
  const float space_lines = 40;
  const float space_stats = 50;

  stringstream roll_text;
  roll_text << std::setprecision(3) << roll_error << "deg";
  cv::putText(panel, "Roll std. dev [deg]:", cv::Point2f(30, space_top), cv::FONT_HERSHEY_PLAIN, font_scale, err_to_rgb(roll_error, max_err), font_thickness);
  cv::putText(panel, roll_text.str(), cv::Point2f(30, space_top+space_lines), cv::FONT_HERSHEY_PLAIN, font_scale, err_to_rgb(roll_error, max_err), font_thickness);

  stringstream pitch_text;
  pitch_text << std::setprecision(3) << pitch_error << "deg";
  cv::putText(panel, "Pitch std. dev [deg]:", cv::Point2f(30, space_top+space_lines+space_stats), cv::FONT_HERSHEY_PLAIN, font_scale, err_to_rgb(pitch_error, max_err), font_thickness);
  cv::putText(panel, pitch_text.str(), cv::Point2f(30, space_top+2*space_lines+space_stats), cv::FONT_HERSHEY_PLAIN, font_scale, err_to_rgb(pitch_error, max_err), font_thickness);

  cv::putText(panel, "Fix:", cv::Point2f(30, space_top+2*space_lines+2*space_stats), cv::FONT_HERSHEY_PLAIN, font_scale, gnss_fix_to_color(gnss_fix_type), font_thickness);
  cv::putText(panel, gnss_fix_to_string(gnss_fix_type), cv::Point2f(30, space_top+3*space_lines+2*space_stats), cv::FONT_HERSHEY_PLAIN, font_scale, gnss_fix_to_color(gnss_fix_type), font_thickness);

  cv::putText(panel, "Heading set:", cv::Point2f(30, space_top+3*space_lines+3*space_stats), cv::FONT_HERSHEY_PLAIN, font_scale, flag_to_color(heading_initialised), font_thickness);
  cv::putText(panel, flag_to_string(heading_initialised), cv::Point2f(30, space_top+4*space_lines+3*space_stats), cv::FONT_HERSHEY_PLAIN, font_scale, flag_to_color(heading_initialised), font_thickness);

  cv::putText(panel, "Dual antenna active:", cv::Point2f(30, space_top+4*space_lines+4*space_stats), cv::FONT_HERSHEY_PLAIN, font_scale, flag_to_color(dual_antenna_heading_active), font_thickness);
  cv::putText(panel, flag_to_string(dual_antenna_heading_active), cv::Point2f(30, space_top+5*space_lines+4*space_stats), cv::FONT_HERSHEY_PLAIN, font_scale, flag_to_color(dual_antenna_heading_active), font_thickness);

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", panel).toImageMsg();

  display_pub.publish(msg);
}

void publish_info_panel_failure(image_transport::Publisher &display_pub) {
  cv::Mat panel(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(255,255,255,255));

  cv::putText(panel, "IMU failure!", cv::Point2f(30, 100), cv::FONT_HERSHEY_PLAIN, 4, cv::Scalar(0,0,255,255), 6);

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", panel).toImageMsg();

  display_pub.publish(msg);
}
