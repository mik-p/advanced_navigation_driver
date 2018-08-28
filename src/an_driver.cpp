/****************************************************************/
/*                                                              */
/*          Advanced Navigation Packet Protocol Library         */
/*        ROS Driver, Packet to Published Message Example       */
/*          Copyright 2017, Advanced Navigation Pty Ltd         */
/*                                                              */
/****************************************************************/
/*
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include <fstream>

#include "rs232/rs232.h"
#include "an_packet_protocol.h"
#include "spatial_packets.h"

#include <sensor_msgs/TimeReference.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <image_transport/image_transport.h>

#include <eigen3/Eigen/Eigen>

using namespace std;

#define RADIANS_TO_DEGREES (180.0/M_PI)

void load_system_status(const system_state_packet_t &system_state_packet,
    diagnostic_msgs::DiagnosticStatus &system_status_msg) {
  system_status_msg.message = "";
  system_status_msg.level = 0; // default OK state
  if (system_state_packet.system_status.b.system_failure) {
      system_status_msg.level = 2; // ERROR state
      system_status_msg.message = system_status_msg.message + "0. System Failure! ";
  }
  if (system_state_packet.system_status.b.accelerometer_sensor_failure) {
      system_status_msg.level = 2; // ERROR state
      system_status_msg.message = system_status_msg.message + "1. Accelerometer Sensor Failure! ";
  }
  if (system_state_packet.system_status.b.gyroscope_sensor_failure) {
      system_status_msg.level = 2; // ERROR state
      system_status_msg.message = system_status_msg.message + "2. Gyroscope Sensor Failure! ";
  }
  if (system_state_packet.system_status.b.magnetometer_sensor_failure) {
      system_status_msg.level = 2; // ERROR state
      system_status_msg.message = system_status_msg.message + "3. Magnetometer Sensor Failure! ";
  }
  if (system_state_packet.system_status.b.pressure_sensor_failure) {
      system_status_msg.level = 2; // ERROR state
      system_status_msg.message = system_status_msg.message + "4. Pressure Sensor Failure! ";
  }
  if (system_state_packet.system_status.b.gnss_failure) {
      system_status_msg.level = 2; // ERROR state
      system_status_msg.message = system_status_msg.message + "5. GNSS Failure! ";
  }
  if (system_state_packet.system_status.b.accelerometer_over_range) {
      system_status_msg.level = 2; // ERROR state
      system_status_msg.message = system_status_msg.message + "6. Accelerometer Over Range! ";
  }
  if (system_state_packet.system_status.b.gyroscope_over_range) {
      system_status_msg.level = 2; // ERROR state
      system_status_msg.message = system_status_msg.message + "7. Gyroscope Over Range! ";
  }
  if (system_state_packet.system_status.b.magnetometer_over_range) {
      system_status_msg.level = 2; // ERROR state
      system_status_msg.message = system_status_msg.message + "8. Magnetometer Over Range! ";
  }
  if (system_state_packet.system_status.b.pressure_over_range) {
      system_status_msg.level = 2; // ERROR state
      system_status_msg.message = system_status_msg.message + "9. Pressure Over Range! ";
  }
  if (system_state_packet.system_status.b.minimum_temperature_alarm) {
      system_status_msg.level = 2; // ERROR state
      system_status_msg.message = system_status_msg.message + "10. Minimum Temperature Alarm! ";
  }
  if (system_state_packet.system_status.b.maximum_temperature_alarm) {
      system_status_msg.level = 2; // ERROR state
      system_status_msg.message = system_status_msg.message + "11. Maximum Temperature Alarm! ";
  }
  if (system_state_packet.system_status.b.low_voltage_alarm) {
      system_status_msg.level = 2; // ERROR state
      system_status_msg.message = system_status_msg.message + "12. Low Voltage Alarm! ";
  }
  if (system_state_packet.system_status.b.high_voltage_alarm) {
      system_status_msg.level = 2; // ERROR state
      system_status_msg.message = system_status_msg.message + "13. High Voltage Alarm! ";
  }
  if (system_state_packet.system_status.b.gnss_antenna_disconnected) {
      system_status_msg.level = 2; // ERROR state
      system_status_msg.message = system_status_msg.message + "14. GNSS Antenna Disconnected! ";
  }
  if (system_state_packet.system_status.b.serial_port_overflow_alarm) {
      system_status_msg.level = 2; // ERROR state
      system_status_msg.message = system_status_msg.message + "15. Data Output Overflow Alarm! ";
  }
}

void load_filter_status(const system_state_packet_t &system_state_packet,
    diagnostic_msgs::DiagnosticStatus &filter_status_msg) {
  filter_status_msg.message = "";
  filter_status_msg.level = 0; // default OK state
  if (system_state_packet.filter_status.b.orientation_filter_initialised) {
      filter_status_msg.message = filter_status_msg.message + "0. Orientation Filter Initialised. ";
  }
  else {
      filter_status_msg.level = 1; // WARN state
      filter_status_msg.message = filter_status_msg.message + "0. Orientation Filter NOT Initialised. ";
  }
  if (system_state_packet.filter_status.b.ins_filter_initialised) {
      filter_status_msg.message = filter_status_msg.message + "1. Navigation Filter Initialised. ";
  }
  else {
      filter_status_msg.level = 1; // WARN state
      filter_status_msg.message = filter_status_msg.message + "1. Navigation Filter NOT Initialised. ";
  }
  if (system_state_packet.filter_status.b.heading_initialised) {
      filter_status_msg.message = filter_status_msg.message + "2. Heading Initialised. ";
  }
  else {
      filter_status_msg.level = 1; // WARN state
      filter_status_msg.message = filter_status_msg.message + "2. Heading NOT Initialised. ";
  }
  if (system_state_packet.filter_status.b.utc_time_initialised) {
      filter_status_msg.message = filter_status_msg.message + "3. UTC Time Initialised. ";
  }
  else {
      filter_status_msg.level = 1; // WARN state
      filter_status_msg.message = filter_status_msg.message + "3. UTC Time NOT Initialised. ";
  }
  if (system_state_packet.filter_status.b.event1_flag) {
      filter_status_msg.level = 1; // WARN state
      filter_status_msg.message = filter_status_msg.message + "7. Event 1 Occured. ";
  }
  else {
      filter_status_msg.message = filter_status_msg.message + "7. Event 1 NOT Occured. ";
  }
  if (system_state_packet.filter_status.b.event2_flag) {
      filter_status_msg.level = 1; // WARN state
      filter_status_msg.message = filter_status_msg.message + "8. Event 2 Occured. ";
  }
  else {
      filter_status_msg.message = filter_status_msg.message + "8. Event 2 NOT Occured. ";
  }
  if (system_state_packet.filter_status.b.internal_gnss_enabled) {
      filter_status_msg.message = filter_status_msg.message + "9. Internal GNSS Enabled. ";
  }
  else {
      filter_status_msg.level = 1; // WARN state
      filter_status_msg.message = filter_status_msg.message + "9. Internal GNSS NOT Enabled. ";
  }
  if (system_state_packet.filter_status.b.magnetic_heading_enabled) {
      filter_status_msg.message = filter_status_msg.message + "10. Magnetic Heading Active. ";
  }
  else {
      filter_status_msg.level = 1; // WARN state
      filter_status_msg.message = filter_status_msg.message + "10. Magnetic Heading NOT Active. ";
  }
  if (system_state_packet.filter_status.b.velocity_heading_enabled) {
      filter_status_msg.message = filter_status_msg.message + "11. Velocity Heading Enabled. ";
  }
  else {
      filter_status_msg.level = 1; // WARN state
      filter_status_msg.message = filter_status_msg.message + "11. Velocity Heading NOT Enabled. ";
  }
  if (system_state_packet.filter_status.b.atmospheric_altitude_enabled) {
      filter_status_msg.message = filter_status_msg.message + "12. Atmospheric Altitude Enabled. ";
  }
  else {
      filter_status_msg.message = filter_status_msg.message + "12. Atmospheric Altitude NOT Enabled. ";
      filter_status_msg.level = 1; // WARN state
  }
  if (system_state_packet.filter_status.b.external_position_active) {
      filter_status_msg.message = filter_status_msg.message + "13. External Position Active. ";
  }
  else {
      filter_status_msg.level = 1; // WARN state
      filter_status_msg.message = filter_status_msg.message + "13. External Position NOT Active. ";
  }
  if (system_state_packet.filter_status.b.external_velocity_active) {
      filter_status_msg.message = filter_status_msg.message + "14. External Velocity Active. ";
  }
  else {
      filter_status_msg.level = 1; // WARN state
      filter_status_msg.message = filter_status_msg.message + "14. External Velocity NOT Active. ";
  }
  if (system_state_packet.filter_status.b.external_heading_active) {
      filter_status_msg.message = filter_status_msg.message + "15. External Heading Active. ";
  }
  else {
      filter_status_msg.level = 1; // WARN state
      filter_status_msg.message = filter_status_msg.message + "15. External Heading NOT Active. ";
  }
}

Eigen::Matrix3f rollM(const float roll) {
  Eigen::Matrix3f Rx;
  float cr = cos(roll);
  float sr = sin(roll);
  Rx << 1.0, 0.0, 0.0,
        0.0,  cr, -sr,
        0.0,  sr,  cr;
  return Rx;
}

Eigen::Matrix3f pitchM(const float pitch) {
  Eigen::Matrix3f Ry;
  float cp = cos(pitch);
  float sp = sin(pitch);
  Ry <<  cp, 0.0,  sp,
        0.0, 1.0, 0.0,
        -sp, 0.0,  cp;
  return Ry;
}

Eigen::Matrix3f yawM(const float yaw) {
  Eigen::Matrix3f Rz;
  float cy = cos(yaw);
  float sy = sin(yaw);
  Rz <<  cy, -sy, 0.0,
         sy,  cy, 0.0,
        0.0, 0.0, 1.0;
  return Rz;
}

void load_orientation(const float *rpy, geometry_msgs::Quaternion &orientation, bool should_discard_heading) {
  // Convert roll, pitch, yaw from radians to quaternion format //
  float phi = rpy[0] / 2.0f;
  float theta = rpy[1] / 2.0f;
  float psi = should_discard_heading ? 0.0 : rpy[2] / 2.0f;

  float sin_phi = sinf(phi);
  float cos_phi = cosf(phi);
  float sin_theta = sinf(theta);
  float cos_theta = cosf(theta);
  float sin_psi = sinf(psi);
  float cos_psi = cosf(psi);
  orientation.x = -cos_phi * sin_theta * sin_psi + sin_phi * cos_theta * cos_psi;
  orientation.y = cos_phi * sin_theta * cos_psi + sin_phi * cos_theta * sin_psi;
  orientation.z = cos_phi * cos_theta * sin_psi - sin_phi * sin_theta * cos_psi;
  orientation.w = cos_phi * cos_theta * cos_psi + sin_phi * sin_theta * sin_psi;

  /*cerr << orientation << endl;

  Eigen::Quaternionf XYZ_q(rollM(rpy[0]) * pitchM(rpy[1]) * yawM(rpy[2]));
  cerr << XYZ_q.x() << " " << XYZ_q.y() << " " << XYZ_q.z() << " " << XYZ_q.w() << endl;

  Eigen::Quaternionf ZYX_q(yawM(rpy[2]) * pitchM(rpy[1]) * rollM(rpy[0]));
  cerr << ZYX_q.x() << " " << ZYX_q.y() << " " << ZYX_q.z() << " " << ZYX_q.w() << endl;*/
}

void publish_info_panel(image_transport::Publisher &display_pub, geometry_msgs::Vector3Stamped pose_errors_msg, float std_deviation_threshold);

class JsonGenerator {
public:
  JsonGenerator(void) : isOpened(false), isFirst(true) {
  }

  void openFile(const string &filename) {
    file.open(filename.c_str());
    file << endl << "[" << endl;
    isOpened = true;
  }

  void genDict(const float timeStamp,
      const geometry_msgs::Quaternion &quaternion,
      const geometry_msgs::Vector3 eulerStdDev) {
    if(isOpened) {
      if(!isFirst) {
        file << ", " << endl;
      }
      isFirst = false;
      file << "{ \"type\":\"quat_data\", \"timeStamp\":" << timeStamp << ", ";
      file << "\"quaternion\":[" << quaternion.x << ", " << quaternion.y << ", " << quaternion.z << ", " << quaternion.w << "], ";
      file << "\"error\":[" << eulerStdDev.x << ", " << eulerStdDev.y << ", " << eulerStdDev.z << "] }";
    }
  }

  void close(void) {
    if(isOpened) {
      file << "]" << endl;
      file.close();
    }
  }

private:
  ofstream file;
  bool isOpened, isFirst;
};

JsonGenerator generator;

void shutdownHandler(int sig) {
  generator.close();
  ros::shutdown();
}

// SpatialDual publish NEMEA timestamps within current hour to Velodyne
float fixHourOverflow(const float hourTimestamp) {
  static int hours = 0;
  static float last_timestamp = -1.0;
  if(last_timestamp > hourTimestamp) {
    hours++;
    ROS_INFO_STREAM("Hour overflow, hours: " << hours);
  }
  last_timestamp = hourTimestamp;
  return last_timestamp + 60*60*hours;
}


class ANPPLogger {

public:
  bool open(const string &filename) {
    logFile.open(filename.c_str(), ios::out | ios::binary);
    return logFile.is_open();
  }

  bool write(const an_packet_t &packet) {
    if(!logFile.is_open()) {
      return false;
    }
    logFile.write((char*)&packet.header, sizeof(uint8_t) * AN_PACKET_HEADER_SIZE);
    logFile.write((char*)&packet.data, packet.length * sizeof(uint8_t));
    return true;
  }

private:
  ofstream logFile;

} anpp_logger;

int main(int argc, char *argv[]) {
  // Set up ROS node //
  ros::init(argc, argv, "an_device_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  signal(SIGINT, shutdownHandler);
  ros::NodeHandle pnh("~");

  printf("\nYour Advanced Navigation ROS driver is currently running\nPress Ctrl-C to interrupt\n");

  // Set up the COM port
  std::string com_port;
  int baud_rate;
  float std_deviation_threshold;
  std::string output_filename, output_binary_log;
  bool should_discard_heading;

  pnh.param<std::string>("uart_port", com_port, "/dev/ttyUSB0");
  pnh.param<int>("uart_baud_rate", baud_rate, 115200);
  pnh.param<float>("std_deviation_threshold", std_deviation_threshold, 0.6);
  pnh.param<std::string>("output_file", output_filename, "");
  pnh.param<std::string>("output_binary_log", output_binary_log, "");
  pnh.param<bool>("discard_heading", should_discard_heading, true);

  // Initialise Publishers and Topics //
  ros::Publisher orientation_pub = nh.advertise<geometry_msgs::PoseStamped>("imu_pose", 10);
  ros::Publisher orientation_err_pub = nh.advertise<geometry_msgs::Vector3Stamped>("imu_pose_errors", 10);
  ros::Publisher timeref_pub = nh.advertise<sensor_msgs::TimeReference>("imu_timeref", 10);
  ros::Publisher system_status_pub = nh.advertise<diagnostic_msgs::DiagnosticStatus>("imu_status", 10);
  ros::Publisher filter_status_pub = nh.advertise<diagnostic_msgs::DiagnosticStatus>("imu_filter_status", 10);
  image_transport::ImageTransport it(nh);
  image_transport::Publisher display_pub = it.advertise("info_display", 10);

  geometry_msgs::PoseStamped orientation_msg;
  orientation_msg.header.frame_id = "imu_base";

  geometry_msgs::Vector3Stamped orientation_errors_msg;
  orientation_errors_msg.header.frame_id = "imu_base";

  sensor_msgs::TimeReference time_ref;
  time_ref.header.frame_id = "imu_base";

  tf::TransformBroadcaster tf_broadcaster;
  tf::StampedTransform tf_msg;
  tf_msg.frame_id_ = "imu_base";
  tf_msg.child_frame_id_ = "imu";

  diagnostic_msgs::DiagnosticStatus system_status_msg;
  system_status_msg.level = 0; // default OK state
  system_status_msg.name = "System Status";
  system_status_msg.message = "";

  diagnostic_msgs::DiagnosticStatus filter_status_msg;
  filter_status_msg.level = 0; // default OK state
  filter_status_msg.name = "Filter Status";
  filter_status_msg.message = "";

  // get data from com port //
  an_decoder_t an_decoder;
  an_packet_t *an_packet;
  system_state_packet_t system_state_packet;
  euler_orientation_standard_deviation_packet_t euler_orientation_standard_deviation_packet;
  running_time_packet_t time_packet;
  int bytes_received;

  if (OpenComport(const_cast<char*>(com_port.c_str()), baud_rate)) {
    printf("Could not open serial port: %s \n", com_port.c_str());
    exit(EXIT_FAILURE);
  }

  an_decoder_initialise(&an_decoder);

  if(!output_filename.empty()) {
    generator.openFile(output_filename);
    ROS_INFO_STREAM("OUTPUT_FILE: " << output_filename);
  }

  if(!output_binary_log.empty()) {
    anpp_logger.open(output_binary_log);
    ROS_INFO_STREAM("Binary log saved to: " << output_binary_log);
  }

  // Loop continuously, polling for packets
  while (ros::ok()) {
    ros::spinOnce();
    if ((bytes_received = PollComport(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) > 0) {
      // increment the decode buffer length by the number of bytes received //
      an_decoder_increment(&an_decoder, bytes_received);

      // decode all the packets in the buffer //
      while ((an_packet = an_packet_decode(&an_decoder)) != NULL) {

        ros::Time ros_now = ros::Time::now();
        //cerr << "Packet came, ID: " << (int)an_packet->id << endl;

        // system state packet //
        if (an_packet->id == packet_id_system_state) {
          if (decode_system_state_packet(&system_state_packet, an_packet) == 0) {
            // System status
            load_system_status(system_state_packet, system_status_msg);
            system_status_pub.publish(system_status_msg);

            // Filter status
            load_filter_status(system_state_packet, filter_status_msg);
            filter_status_pub.publish(filter_status_msg);

            // IMU
            orientation_msg.header.stamp = ros_now;
            load_orientation(system_state_packet.orientation, orientation_msg.pose.orientation, should_discard_heading);
            orientation_pub.publish(orientation_msg);

            tf_msg.stamp_ = ros_now;
            tf_msg.setOrigin(tf::Vector3(
                orientation_msg.pose.position.x, orientation_msg.pose.position.y, orientation_msg.pose.position.z));
            tf_msg.setRotation(tf::Quaternion(
                orientation_msg.pose.orientation.x, orientation_msg.pose.orientation.y, orientation_msg.pose.orientation.z,
                orientation_msg.pose.orientation.w));
            tf_broadcaster.sendTransform(tf_msg);

            // SpatialDual publish NEMEA timestamps within current hour to Velodyne
            float hourTimestamp = system_state_packet.unix_time_seconds % 3600 + system_state_packet.microseconds*1e-6;
            float timestamp = fixHourOverflow(hourTimestamp);
            time_ref.header.stamp = ros_now;
            time_ref.time_ref = ros::Time(timestamp);
            timeref_pub.publish(time_ref);

            generator.genDict(timestamp, orientation_msg.pose.orientation, orientation_errors_msg.vector);
          }
        }

        if (an_packet->id == packet_id_euler_orientation_standard_deviation) {
          if (decode_euler_orientation_standard_deviation_packet(&euler_orientation_standard_deviation_packet, an_packet) == 0) {
            // IMU error
            orientation_errors_msg.header.stamp = ros_now;
            orientation_errors_msg.vector.x = euler_orientation_standard_deviation_packet.standard_deviation[0];
            orientation_errors_msg.vector.y = euler_orientation_standard_deviation_packet.standard_deviation[1];
            orientation_errors_msg.vector.z = euler_orientation_standard_deviation_packet.standard_deviation[2];
            orientation_err_pub.publish(orientation_errors_msg);

            publish_info_panel(display_pub, orientation_errors_msg, std_deviation_threshold);
          }
        }

        anpp_logger.write(*an_packet);

        an_packet_free(&an_packet);
      }
    }
  }

}

