/**
 * MIT License
 *
 * Copyright (c) 2017 an-scott
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <eigen3/Eigen/Eigen>

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

namespace an_driver
{

class ANDriver
{
public:
    ANDriver(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~ANDriver();

    // Convert NED orientation (Advanced Navigation) to ENU orientation (ROS)
    static inline geometry_msgs::Quaternion nedToEnu(geometry_msgs::Quaternion const &in)
    {
        tf::Quaternion enu(in.x, in.y, in.z, in.w); // x, y, z, w according to tf documentation
        // To get from NED to ENU we have to:
        // 1) Rotate 90 degrees around Z
        // 2) Rotate the result 180 degrees around Y
        tf::Transform z_90(tf::Quaternion(tf::Vector3(0, 0, 1), 0.5 * M_PI));
        tf::Transform y_180(tf::Quaternion(tf::Vector3(0, 1, 0), M_PI));

        enu = (y_180 * (z_90 * enu));
        geometry_msgs::Quaternion out;
        out.x = enu.x();
        out.y = enu.y();
        out.z = enu.z();
        out.w = enu.w();
        return out;
    }

    // Convert the packet status to a ros diagnostic message
    static inline void appendSystemStatus(diagnostic_msgs::DiagnosticArray &status_array, unsigned int const status, int const level, std::string const &failure_message = "", std::string const &success_message = "")
    {
        if (status)
        {
            diagnostic_msgs::DiagnosticStatus status_msg;
            status_msg.level = level; // WARN=1 ERROR=2
            status_msg.message = failure_message;
            status_array.status.push_back(status_msg);
        }
    }

    // Convert the packet status to a ros diagnostic message
    static inline diagnostic_msgs::DiagnosticStatus filterStatusToMsg(unsigned int const status, int const level, std::string const &failure_message = "", std::string const &success_message = "")
    {
        diagnostic_msgs::DiagnosticStatus status_msg;
        status_msg.message = success_message;
        status_msg.level = 0; // default OK state
        if (!status)
        {
            status_msg.level = level; // WARN=1 ERROR=2
            status_msg.message = failure_message;
        }
        return status_msg;
    }

    static inline Eigen::Matrix3f rollM(const float roll)
    {
        Eigen::Matrix3f Rx;
        float cr = cos(roll);
        float sr = sin(roll);
        Rx << 1.0, 0.0, 0.0,
            0.0, cr, -sr,
            0.0, sr, cr;
        return Rx;
    }

    static inline Eigen::Matrix3f pitchM(const float pitch)
    {
        Eigen::Matrix3f Ry;
        float cp = cos(pitch);
        float sp = sin(pitch);
        Ry << cp, 0.0, sp,
            0.0, 1.0, 0.0,
            -sp, 0.0, cp;
        return Ry;
    }

    static inline Eigen::Matrix3f yawM(const float yaw)
    {
        Eigen::Matrix3f Rz;
        float cy = cos(yaw);
        float sy = sin(yaw);
        Rz << cy, -sy, 0.0,
            sy, cy, 0.0,
            0.0, 0.0, 1.0;
        return Rz;
    }

    static inline float fixHourOverflow(const float hourTimestamp)
    {
        static int hours = 0;
        static float last_timestamp = -1.0;
        if (last_timestamp > hourTimestamp)
        {
            hours++;
            ROS_INFO_STREAM("Hour overflow, hours: " << hours);
        }
        last_timestamp = hourTimestamp;
        return last_timestamp + 60 * 60 * hours;
    }

    void loop();

private:
    ros::NodeHandle nh_;  // node handle
    ros::NodeHandle pnh_; // private node handle
    // parameters
    std::string com_port_s_;
    char *com_port_;
    int baud_rate_;
    int loop_rate_;
    std::string imu_frame_id_;
    std::string nav_sat_frame_id_;
    std::string topic_prefix_;
    std::string output_binary_log_;
    // publishers
    ros::Publisher nav_sat_fix_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher timeref_pub_;
    ros::Publisher diagnostics_pub_;
};

} // namespace an_driver
