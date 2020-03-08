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

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <string.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>

#include <rs232/rs232.h>
#include <an_packet_protocol.h>
#include <spatial_packets.h>

#include <advanced_navigation_driver/an_driver.h>
#include <advanced_navigation_driver/anpp_logger.h>

namespace an_driver
{

#define RADIANS_TO_DEGREES (180.0 / M_PI)
const double PI = 4 * atan(1);

ANDriver::ANDriver(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
{
	// ros params
	pnh.param("port", com_port_s_, std::string("/dev/ttyUSB0")); // "/dev/ttyS0"
	com_port_ = (char *)com_port_s_.c_str();
	pnh.param("baud_rate", baud_rate_, 115200);
	pnh.param("loop_rate", loop_rate_, 100);
	pnh.param("imu_frame_id", imu_frame_id_, std::string("imu"));
	pnh.param("navsat_frame_id", nav_sat_frame_id_, std::string("gps"));
	pnh.param("output_binary_log", output_binary_log_, std::string(""));
	pnh.param("antenna_offset_x", antenna_offset_[0], 0.0);
	pnh.param("antenna_offset_y", antenna_offset_[1], 0.0);
	pnh.param("antenna_offset_z", antenna_offset_[2], 0.0);
	pnh.param("debug", debug_, 0);
	pnh.param("device_time", device_time_, false);
	pnh.param("remove_gravity", remove_gravity_, false);

	// Set up the COM port
	if (OpenComport(com_port_, baud_rate_))
	{
		ROS_INFO("Could not open serial port %s at %d baud.", com_port_, baud_rate_);
		ros::shutdown();
		// exit(EXIT_FAILURE);
	}
	ROS_INFO("port:%s@%d", com_port_, baud_rate_);

	// ros topics
	nav_sat_fix_pub_ = pnh.advertise<sensor_msgs::NavSatFix>("nav_sat_fix", 10);
	imu_pub_ = pnh.advertise<sensor_msgs::Imu>("imu", 10);
	odom_pub_ = pnh.advertise<nav_msgs::Odometry>("odom", 10);
	timeref_pub_ = nh.advertise<sensor_msgs::TimeReference>("timeref", 10);
	diagnostics_pub_ = pnh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);
}

ANDriver::~ANDriver()
{
}

void ANDriver::loop()
{
	// Initialise gps message
	sensor_msgs::NavSatFix nav_sat_fix_msg;
	nav_sat_fix_msg.header.frame_id = "an_device";
	nav_sat_fix_msg.position_covariance_type = 2; // fixed to variance on the diagonal
	nav_sat_fix_msg.position_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	// Initialize wheel odometry message
	nav_msgs::Odometry odom_msg;
	odom_msg.header.frame_id = "odom";
	odom_msg.child_frame_id = "an_device";
	odom_msg.twist.covariance = {
		1.0, 0, 0, 0, 0, 0,
		0, 1.0, 0, 0, 0, 0,
		0, 0, 1.0, 0, 0, 0,
		0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0};

	// Initialize imu message
	sensor_msgs::Imu imu_msg;
	imu_msg.header.frame_id = "an_device";
	imu_msg.orientation_covariance = {// Will be read from device
									  0.0, 0.0, 0.0,
									  0.0, 0.0, 0.0,
									  0.0, 0.0, 0.0};
	imu_msg.angular_velocity_covariance = {// Cannot be read from device
										   0.2, 0.0, 0.0,
										   0.0, 0.2, 0.0,
										   0.0, 0.0, 0.2};
	imu_msg.linear_acceleration_covariance = {// Cannot be read from device
											  1.0, 0.0, 0.0,
											  0.0, 1.0, 0.0,
											  0.0, 0.0, 1.0};

	tf::Quaternion orientation;

	// init time ref msg
	sensor_msgs::TimeReference time_ref;
	time_ref.header.frame_id = "an_device";

	// Status messages
	diagnostic_msgs::DiagnosticArray diagnostics_msg;

	// get data from com port //
	an_decoder_t an_decoder;
	an_packet_t *an_packet;

	// Array for message received
	int packets_received[1000] = {0};

	// packet types
	system_state_packet_t system_state_packet;												   // Packet 20
	euler_orientation_standard_deviation_packet_t euler_orientation_standard_deviation_packet; // Packet 26
	body_velocity_packet_t body_velocity_packet;											   // Packet 36
	quaternion_orientation_packet_t quaternion_orientation_packet;							   // Packet 40
	status_packet_t status_packet;															   // packet 23
	satellites_packet_t satellites_packet;													   // packet 30
	odometer_state_packet_t odometer_state_packet;											   // packet 51
	quaternion_orientation_standard_deviation_packet_t quaternion_orientation_standard_deviation_packet;
	raw_sensors_packet_t raw_sensors_packet;
	// running_time_packet_t time_packet;														   // packet 49

	// satellite metrics
	size_t satelites_cnt = 0;
	float hdop = -1.0;
	float vdop = -1.0;

	an_decoder_initialise(&an_decoder);
	int bytes_received;

	// start bin logger
	ANPPLogger anpp_logger;
	if (!output_binary_log_.empty())
	{
		anpp_logger.open(output_binary_log_);
		ROS_INFO_STREAM("Binary log saved to: " << output_binary_log_);
	}
	ROS_INFO_STREAM("Output ANPP log file successfully opened");

	// set packet data rate //
	packet_timer_period_packet_t timer_packet = {0};
	timer_packet.permanent = TRUE;
	timer_packet.utc_synchronisation = TRUE;
	timer_packet.packet_timer_period = 1;
	an_packet = encode_packet_timer_period_packet(&timer_packet);
	an_packet_encode(an_packet);
	SendBuf(an_packet_pointer(an_packet), an_packet_size(an_packet));
	an_packet_free(&an_packet);

	packet_periods_packet_t period_packet = {0};
	int period = 1.e3 / loop_rate_;
	period_packet.permanent = TRUE;
	period_packet.clear_existing_packets = TRUE;
	period_packet.packet_periods[0].packet_id = packet_id_system_state;
	period_packet.packet_periods[0].period = period;
	period_packet.packet_periods[1].packet_id = packet_id_raw_sensors;
	period_packet.packet_periods[1].period = period;
	period_packet.packet_periods[2].packet_id = packet_id_quaternion_orientation_standard_deviation;
	period_packet.packet_periods[2].period = period;
	an_packet = encode_packet_periods_packet(&period_packet);
	an_packet_encode(an_packet);
	SendBuf(an_packet_pointer(an_packet), an_packet_size(an_packet));
	an_packet_free(&an_packet);

	// set antenna offset (not currently working)//
	installation_alignment_packet_t installation_alignment = {0};
	installation_alignment.permanent = TRUE;
	installation_alignment.alignment_dcm[0][0] = 1.0;
	installation_alignment.alignment_dcm[1][1] = 1.0;
	installation_alignment.alignment_dcm[2][2] = 1.0;
	installation_alignment.gnss_antenna_offset[0] = antenna_offset_[0];
	installation_alignment.gnss_antenna_offset[1] = antenna_offset_[1];
	installation_alignment.gnss_antenna_offset[2] = antenna_offset_[2];
	an_packet = encode_installation_alignment_packet(&installation_alignment);
	an_packet_encode(an_packet);
	SendBuf(an_packet_pointer(an_packet), an_packet_size(an_packet));
	an_packet_free(&an_packet);

	// some more parameters that are needed //
	an_decoder_initialise(&an_decoder);
	long long ros_last = ros::Time::now().toNSec() / 1000;
	ros::Time ros_time = ros::Time::now();

	// Loop continuously, polling for packets
	ros::Rate rate(loop_rate_);

	while (ros::ok())
	{
		ros::spinOnce();

		if ((bytes_received = PollComport(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) > 0)
		{
			// increment the decode buffer length by the number of bytes received //
			an_decoder_increment(&an_decoder, bytes_received);

			// decode all the packets in the buffer //
			while ((an_packet = an_packet_decode(&an_decoder)) != NULL)
			{
				ros::Time ros_now = ros::Time::now(); // time now

				//mark that we have received a certain packet
				if (an_packet->id < 1000)
				{
					packets_received[an_packet->id] += 1;
				}

				// acknowledgement packet (0) //
				if (an_packet->id == packet_id_acknowledge)
				{
					ROS_INFO("acknowledgement data: %d", an_packet->data[3]);
				}

				// status packet (23)
				if (an_packet->id == packet_id_status)
				{
					if (decode_status_packet(&status_packet, an_packet) == 0)
					{
						// last_dual_antena_active = status_packet.filter_status.b.dual_antenna_heading_active;
					}
				}

				// satellites packet (30)
				if (an_packet->id == packet_id_satellites)
				{
					if (decode_satellites_packet(&satellites_packet, an_packet) == 0)
					{
						satelites_cnt = satellites_packet.gps_satellites + satellites_packet.glonass_satellites +
										satellites_packet.beidou_satellites + satellites_packet.galileo_satellites + satellites_packet.sbas_satellites;
						hdop = satellites_packet.hdop;
						vdop = satellites_packet.vdop;
					}
				}

				// system state packet (20) //
				if (an_packet->id == packet_id_system_state)
				{
					if (decode_system_state_packet(&system_state_packet, an_packet) == 0)
					{
						ros_time = ros::Time::now();
						if (!device_time_)
						{
							system_state_packet.unix_time_seconds = ros_time.sec;
							system_state_packet.microseconds = ros_time.nsec / 1000;
						}

						// NavSatFix
						nav_sat_fix_msg.header.stamp.sec = system_state_packet.unix_time_seconds;
						nav_sat_fix_msg.header.stamp.nsec = system_state_packet.microseconds * 1000;
						if ((system_state_packet.filter_status.b.gnss_fix_type == 1) ||
							(system_state_packet.filter_status.b.gnss_fix_type == 2))
						{
							nav_sat_fix_msg.status.status = 0;
						}
						else if ((system_state_packet.filter_status.b.gnss_fix_type == 3) ||
								 (system_state_packet.filter_status.b.gnss_fix_type == 5))
						{
							nav_sat_fix_msg.status.status = 1;
						}
						else if ((system_state_packet.filter_status.b.gnss_fix_type == 4) ||
								 (system_state_packet.filter_status.b.gnss_fix_type == 6) ||
								 (system_state_packet.filter_status.b.gnss_fix_type == 7))
						{
							nav_sat_fix_msg.status.status = 2;
						}
						else
						{
							nav_sat_fix_msg.status.status = -1;
						}
						nav_sat_fix_msg.latitude = system_state_packet.latitude * RADIANS_TO_DEGREES;
						nav_sat_fix_msg.longitude = system_state_packet.longitude * RADIANS_TO_DEGREES;
						nav_sat_fix_msg.altitude = system_state_packet.height;
						nav_sat_fix_msg.position_covariance[0] = pow(system_state_packet.standard_deviation[1], 2);
						nav_sat_fix_msg.position_covariance[4] = pow(system_state_packet.standard_deviation[0], 2);
						nav_sat_fix_msg.position_covariance[8] = pow(system_state_packet.standard_deviation[2], 2);

						// IMU
						imu_msg.header.stamp.sec = system_state_packet.unix_time_seconds;
						imu_msg.header.stamp.nsec = system_state_packet.microseconds * 1000;
						imu_msg.header.frame_id = imu_frame_id_;
						// Convert roll, pitch, yaw from radians to quaternion format //
						orientation.setRPY(
							system_state_packet.orientation[0],
							system_state_packet.orientation[1],
							PI / 2.0f - system_state_packet.orientation[2] // REP 103
						);
						imu_msg.orientation.x = orientation[0];
						imu_msg.orientation.y = orientation[1];
						imu_msg.orientation.z = orientation[2];
						imu_msg.orientation.w = orientation[3];

						if (remove_gravity_)
						{
							imu_msg.angular_velocity.x = system_state_packet.angular_velocity[0]; // These the same as the TWIST msg values
							imu_msg.angular_velocity.y = system_state_packet.angular_velocity[1];
							imu_msg.angular_velocity.z = system_state_packet.angular_velocity[2];
							imu_msg.linear_acceleration.x = system_state_packet.body_acceleration[0];
							imu_msg.linear_acceleration.y = system_state_packet.body_acceleration[1];
							imu_msg.linear_acceleration.z = system_state_packet.body_acceleration[2];
						}

						// TIME REF
						time_ref.header.stamp = ros_now;
						float hourTimestamp = system_state_packet.unix_time_seconds % 3600 + system_state_packet.microseconds * 1e-6;
						float timestamp = fixHourOverflow(hourTimestamp);
						time_ref.time_ref = ros::Time(timestamp);

						// Filter Status
						diagnostics_msg.header.stamp.sec = system_state_packet.unix_time_seconds;
						diagnostics_msg.header.stamp.nsec = system_state_packet.microseconds * 1000;
						diagnostics_msg.status.clear();
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.orientation_filter_initialised, 1, "Orientation Filter NOT Initialised", "Orientation Filter Initialised"));
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.ins_filter_initialised, 1, "Navigation Filter NOT Initialised", "Navigation Filter Initialised"));
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.heading_initialised, 1, "Heading NOT Initialised", "Heading Initialised"));
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.utc_time_initialised, 1, "UTC Time NOT Initialised", "UTC Time Initialised"));
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.event1_flag, 1, "Event 1 NOT Occurred", "Event 1 Occurred"));
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.event2_flag, 1, "Event 2 NOT Occurred", "Event 2 Occurred"));
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.internal_gnss_enabled, 1, "Internal GNSS NOT Enabled", "Internal GNSS Enabled"));
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.magnetic_heading_enabled, 1, "Magnetic Heading NOT Active", "Magnetic Heading Active"));
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.velocity_heading_enabled, 1, "Velocity Heading NOT Enabled", "Velocity Heading Enabled"));
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.atmospheric_altitude_enabled, 1, "Atmospheric Altitude NOT Enabled", "Atmospheric Altitude Enabled"));
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.external_position_active, 1, "External Position NOT Active", "External Position Active"));
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.external_velocity_active, 1, "External Velocity NOT Active", "External Velocity Active"));
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.external_heading_active, 1, "External Heading NOT Active", "External Heading Active"));

						// System Status
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.system_failure, 2, "System Failure!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.system_failure, 2, "System Failure!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.accelerometer_sensor_failure, 2, "Accelerometer Sensor Failure!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.gyroscope_sensor_failure, 2, "Gyroscope Sensor Failure!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.magnetometer_sensor_failure, 2, "Magnetometer Sensor Failure!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.pressure_sensor_failure, 2, "Pressure Sensor Failure!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.gnss_failure, 2, "GNSS Failure!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.accelerometer_over_range, 2, "Accelerometer Over Range!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.gyroscope_over_range, 2, "Gyroscope Over Range!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.magnetometer_over_range, 2, "Magnetometer Over Range!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.pressure_over_range, 2, "Pressure Over Range!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.minimum_temperature_alarm, 2, "Minimum Temperature Alarm!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.maximum_temperature_alarm, 2, "Maximum Temperature Alarm!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.low_voltage_alarm, 2, "Low Voltage Alarm!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.high_voltage_alarm, 2, "High Voltage Alarm!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.gnss_antenna_disconnected, 2, "GNSS Antenna Disconnected!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.serial_port_overflow_alarm, 2, "Data Output Overflow Alarm!");
					}
				}

				// euler orientation standard deviation packet (26) //
				if (an_packet->id == packet_id_euler_orientation_standard_deviation)
				{
					if (decode_euler_orientation_standard_deviation_packet(&euler_orientation_standard_deviation_packet, an_packet) == 0)
					{
						imu_msg.orientation_covariance[0] = pow(euler_orientation_standard_deviation_packet.standard_deviation[0], 2);
						imu_msg.orientation_covariance[4] = pow(euler_orientation_standard_deviation_packet.standard_deviation[1], 2);
						imu_msg.orientation_covariance[8] = pow(euler_orientation_standard_deviation_packet.standard_deviation[2], 2);
					}

					// copy all the binary data into the typedef struct for the packet //
					// this allows easy access to all the different values             //
					// if (decode_quaternion_orientation_standard_deviation_packet(&quaternion_orientation_standard_deviation_packet, an_packet) == 0)
					// {
					// 	imu_msg.orientation_covariance[0] = pow(euler_orientation_standard_deviation_packet.standard_deviation[0], 2);
					// 	imu_msg.orientation_covariance[4] = pow(euler_orientation_standard_deviation_packet.standard_deviation[1], 2);
					// 	imu_msg.orientation_covariance[8] = pow(euler_orientation_standard_deviation_packet.standard_deviation[2], 2);
					// }
				}

				// body velocity packet (36)
				if (an_packet->id == packet_id_body_velocity)
				{
					if (decode_body_velocity_packet(&body_velocity_packet, an_packet) == 0)
					{
						odom_msg.header.stamp.sec = system_state_packet.unix_time_seconds;
						odom_msg.header.stamp.nsec = system_state_packet.microseconds * 1000;
						odom_msg.twist.twist.linear.x = body_velocity_packet.velocity[0];
						odom_msg.twist.twist.linear.y = body_velocity_packet.velocity[1]; // Should be close to zero
						odom_msg.twist.twist.linear.z = body_velocity_packet.velocity[2]; // Should be close to zero
					}
				}

				// quaternion orientation packet (40) //
				if (an_packet->id == packet_id_quaternion_orientation)
				{
					if (decode_quaternion_orientation_packet(&quaternion_orientation_packet, an_packet) == 0)
					{
						imu_msg.orientation.x = quaternion_orientation_packet.orientation[1]; // AN reports in s (w?), x, y, z
						imu_msg.orientation.y = quaternion_orientation_packet.orientation[2];
						imu_msg.orientation.z = quaternion_orientation_packet.orientation[3];
						imu_msg.orientation.w = quaternion_orientation_packet.orientation[0];
						imu_msg.orientation = nedToEnu(imu_msg.orientation); // AN reports in NED, ROS expects ENU
					}
				}

				// odometer state (51)
				if (an_packet->id == packet_id_odometer_state)
				{
					if (decode_odometer_state_packet(&odometer_state_packet, an_packet) == 0)
					{
						// odometer_active = odometer_state_packet.active;
						// odometer_speed = odometer_state_packet.speed;
					}
				}

				// receiver information packet (69) //
				if (an_packet->id == packet_id_gnss_receiver_information)
				{
					ROS_INFO("receiver information: %d", an_packet->data[0]);
				}

				// raw imu data (28) //
				if (!remove_gravity_ && an_packet->id == packet_id_raw_sensors)
				{
					// copy all the binary data into the typedef struct for the packet //
					// this allows easy access to all the different values			 //
					if (decode_raw_sensors_packet(&raw_sensors_packet, an_packet) == 0)
					{
						// IMU
						imu_msg.angular_velocity.x = raw_sensors_packet.gyroscopes[0];
						imu_msg.angular_velocity.y = raw_sensors_packet.gyroscopes[1];
						imu_msg.angular_velocity.z = raw_sensors_packet.gyroscopes[2];
						imu_msg.linear_acceleration.x = raw_sensors_packet.accelerometers[0];
						imu_msg.linear_acceleration.y = raw_sensors_packet.accelerometers[1];
						imu_msg.linear_acceleration.z = raw_sensors_packet.accelerometers[2];
					}
				}

				// log bin packet
				anpp_logger.write(*an_packet);

				// Ensure that you free the an_packet when your done with it //
				// or you will leak memory                                   //
				an_packet_free(&an_packet);

				if (debug_)
					ROS_INFO("%d %d %d\n", packets_received[packet_id_system_state],
							 packets_received[packet_id_quaternion_orientation_standard_deviation],
							 packets_received[packet_id_raw_sensors]);

				// check that we have at least one of each required packages
				if (!packets_received[packet_id_system_state] ||
					!packets_received[packet_id_quaternion_orientation_standard_deviation] ||
					(!packets_received[packet_id_raw_sensors] || remove_gravity_))
					continue;

				memset(packets_received, 0, sizeof(packets_received));

				// Make sure packages are only published if the time stamp actually differs //
				long long ros_msec = ros_time.toNSec() / 1000;
				if (ros_msec <= ros_last)
					continue;
				if (debug_)
					ROS_INFO("send at %g\n", 1e6f / (ros_msec - ros_last));
				ros_last = ros_msec;

				// publish tf
				// tf_msg.stamp_ = ros_now;
				// tf_msg.setOrigin(tf::Vector3(
				// 	orientation_msg.pose.position.x, orientation_msg.pose.position.y, orientation_msg.pose.position.z));
				// tf_msg.setRotation(tf::Quaternion(
				// 	orientation_msg_with_heading.pose.orientation.x, orientation_msg_with_heading.pose.orientation.y,
				// 	orientation_msg_with_heading.pose.orientation.z, orientation_msg_with_heading.pose.orientation.w));
				// tf_broadcaster.sendTransform(tf_msg);

				// Publish messages //
				nav_sat_fix_pub_.publish(nav_sat_fix_msg);
				odom_pub_.publish(odom_msg);
				imu_pub_.publish(imu_msg);
				timeref_pub_.publish(time_ref);
				diagnostics_pub_.publish(diagnostics_msg);
			}
		}

		rate.sleep();
	}
}

} // namespace an_driver
