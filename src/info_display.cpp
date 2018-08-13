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

float deg(const float rad) {
  return rad * 180.0 / M_PI;
}

cv::Scalar err_to_rgb(const float error, const float max_error) {
  float rel_error = MIN(error, max_error) / max_error;
  return cv::Scalar(0, 255*(1-rel_error), 255*rel_error, 255);
}

void publish_info_panel(image_transport::Publisher &display_pub, geometry_msgs::Vector3Stamped pose_errors_msg,
    float std_deviation_threshold) {
  cv::Mat panel(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(255,255,255,255));

  float roll_error = deg(pose_errors_msg.vector.x);
  float pitch_error = deg(pose_errors_msg.vector.y);

  float max_err = 3.0 * std_deviation_threshold;

  stringstream roll_text;
  roll_text << std::setprecision(3) << roll_error << "deg";
  cv::putText(panel, "Roll std. dev [deg]:", cv::Point2f(30, 100), cv::FONT_HERSHEY_PLAIN, 3, err_to_rgb(roll_error, max_err), 4);
  cv::putText(panel, roll_text.str(), cv::Point2f(30, 160), cv::FONT_HERSHEY_PLAIN, 3, err_to_rgb(roll_error, max_err), 4);

  stringstream pitch_text;
  pitch_text << std::setprecision(3) << pitch_error << "deg";
  cv::putText(panel, "Pitch std. dev [deg]:", cv::Point2f(30, 250), cv::FONT_HERSHEY_PLAIN, 3, err_to_rgb(pitch_error, max_err), 4);
  cv::putText(panel, pitch_text.str(), cv::Point2f(30, 310), cv::FONT_HERSHEY_PLAIN, 3, err_to_rgb(pitch_error, max_err), 4);

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", panel).toImageMsg();

  display_pub.publish(msg);
}
