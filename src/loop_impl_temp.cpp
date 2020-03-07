#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <image_transport/image_transport.h>

#include <eigen3/Eigen/Eigen>

#include "advanced_navigation_driver/InfoPanelData.h"
#include "advanced_navigation_driver/InfoPanelError.h"

using namespace std;

void load_system_status(const system_state_packet_t &system_state_packet,
                        diagnostic_msgs::DiagnosticStatus &system_status_msg)
{
    system_status_msg.message = "";
    system_status_msg.level = 0; // default OK state
    if (system_state_packet.system_status.b.system_failure)
    {
        system_status_msg.level = 2; // ERROR state
        system_status_msg.message = system_status_msg.message + "0. System Failure! ";
    }
    if (system_state_packet.system_status.b.accelerometer_sensor_failure)
    {
        system_status_msg.level = 2; // ERROR state
        system_status_msg.message = system_status_msg.message + "1. Accelerometer Sensor Failure! ";
    }
    if (system_state_packet.system_status.b.gyroscope_sensor_failure)
    {
        system_status_msg.level = 2; // ERROR state
        system_status_msg.message = system_status_msg.message + "2. Gyroscope Sensor Failure! ";
    }
    if (system_state_packet.system_status.b.magnetometer_sensor_failure)
    {
        system_status_msg.level = 2; // ERROR state
        system_status_msg.message = system_status_msg.message + "3. Magnetometer Sensor Failure! ";
    }
    if (system_state_packet.system_status.b.pressure_sensor_failure)
    {
        system_status_msg.level = 2; // ERROR state
        system_status_msg.message = system_status_msg.message + "4. Pressure Sensor Failure! ";
    }
    if (system_state_packet.system_status.b.gnss_failure)
    {
        system_status_msg.level = 2; // ERROR state
        system_status_msg.message = system_status_msg.message + "5. GNSS Failure! ";
    }
    if (system_state_packet.system_status.b.accelerometer_over_range)
    {
        system_status_msg.level = 2; // ERROR state
        system_status_msg.message = system_status_msg.message + "6. Accelerometer Over Range! ";
    }
    if (system_state_packet.system_status.b.gyroscope_over_range)
    {
        system_status_msg.level = 2; // ERROR state
        system_status_msg.message = system_status_msg.message + "7. Gyroscope Over Range! ";
    }
    if (system_state_packet.system_status.b.magnetometer_over_range)
    {
        system_status_msg.level = 2; // ERROR state
        system_status_msg.message = system_status_msg.message + "8. Magnetometer Over Range! ";
    }
    if (system_state_packet.system_status.b.pressure_over_range)
    {
        system_status_msg.level = 2; // ERROR state
        system_status_msg.message = system_status_msg.message + "9. Pressure Over Range! ";
    }
    if (system_state_packet.system_status.b.minimum_temperature_alarm)
    {
        system_status_msg.level = 2; // ERROR state
        system_status_msg.message = system_status_msg.message + "10. Minimum Temperature Alarm! ";
    }
    if (system_state_packet.system_status.b.maximum_temperature_alarm)
    {
        system_status_msg.level = 2; // ERROR state
        system_status_msg.message = system_status_msg.message + "11. Maximum Temperature Alarm! ";
    }
    if (system_state_packet.system_status.b.low_voltage_alarm)
    {
        system_status_msg.level = 2; // ERROR state
        system_status_msg.message = system_status_msg.message + "12. Low Voltage Alarm! ";
    }
    if (system_state_packet.system_status.b.high_voltage_alarm)
    {
        system_status_msg.level = 2; // ERROR state
        system_status_msg.message = system_status_msg.message + "13. High Voltage Alarm! ";
    }
    if (system_state_packet.system_status.b.gnss_antenna_disconnected)
    {
        system_status_msg.level = 2; // ERROR state
        system_status_msg.message = system_status_msg.message + "14. GNSS Antenna Disconnected! ";
    }
    if (system_state_packet.system_status.b.serial_port_overflow_alarm)
    {
        system_status_msg.level = 2; // ERROR state
        system_status_msg.message = system_status_msg.message + "15. Data Output Overflow Alarm! ";
    }
}

void load_filter_status(const system_state_packet_t &system_state_packet,
                        diagnostic_msgs::DiagnosticStatus &filter_status_msg)
{
    filter_status_msg.message = "";
    filter_status_msg.level = 0; // default OK state
    if (system_state_packet.filter_status.b.orientation_filter_initialised)
    {
        filter_status_msg.message = filter_status_msg.message + "0. Orientation Filter Initialised. ";
    }
    else
    {
        filter_status_msg.level = 1; // WARN state
        filter_status_msg.message = filter_status_msg.message + "0. Orientation Filter NOT Initialised. ";
    }
    if (system_state_packet.filter_status.b.ins_filter_initialised)
    {
        filter_status_msg.message = filter_status_msg.message + "1. Navigation Filter Initialised. ";
    }
    else
    {
        filter_status_msg.level = 1; // WARN state
        filter_status_msg.message = filter_status_msg.message + "1. Navigation Filter NOT Initialised. ";
    }
    if (system_state_packet.filter_status.b.heading_initialised)
    {
        filter_status_msg.message = filter_status_msg.message + "2. Heading Initialised. ";
    }
    else
    {
        filter_status_msg.level = 1; // WARN state
        filter_status_msg.message = filter_status_msg.message + "2. Heading NOT Initialised. ";
    }
    if (system_state_packet.filter_status.b.utc_time_initialised)
    {
        filter_status_msg.message = filter_status_msg.message + "3. UTC Time Initialised. ";
    }
    else
    {
        filter_status_msg.level = 1; // WARN state
        filter_status_msg.message = filter_status_msg.message + "3. UTC Time NOT Initialised. ";
    }
    if (system_state_packet.filter_status.b.event1_flag)
    {
        filter_status_msg.level = 1; // WARN state
        filter_status_msg.message = filter_status_msg.message + "7. Event 1 Occured. ";
    }
    else
    {
        filter_status_msg.message = filter_status_msg.message + "7. Event 1 NOT Occured. ";
    }
    if (system_state_packet.filter_status.b.event2_flag)
    {
        filter_status_msg.level = 1; // WARN state
        filter_status_msg.message = filter_status_msg.message + "8. Event 2 Occured. ";
    }
    else
    {
        filter_status_msg.message = filter_status_msg.message + "8. Event 2 NOT Occured. ";
    }
    if (system_state_packet.filter_status.b.internal_gnss_enabled)
    {
        filter_status_msg.message = filter_status_msg.message + "9. Internal GNSS Enabled. ";
    }
    else
    {
        filter_status_msg.level = 1; // WARN state
        filter_status_msg.message = filter_status_msg.message + "9. Internal GNSS NOT Enabled. ";
    }
    if (system_state_packet.filter_status.b.magnetic_heading_enabled)
    {
        filter_status_msg.message = filter_status_msg.message + "10. Magnetic Heading Active. ";
    }
    else
    {
        filter_status_msg.level = 1; // WARN state
        filter_status_msg.message = filter_status_msg.message + "10. Magnetic Heading NOT Active. ";
    }
    if (system_state_packet.filter_status.b.velocity_heading_enabled)
    {
        filter_status_msg.message = filter_status_msg.message + "11. Velocity Heading Enabled. ";
    }
    else
    {
        filter_status_msg.level = 1; // WARN state
        filter_status_msg.message = filter_status_msg.message + "11. Velocity Heading NOT Enabled. ";
    }
    if (system_state_packet.filter_status.b.atmospheric_altitude_enabled)
    {
        filter_status_msg.message = filter_status_msg.message + "12. Atmospheric Altitude Enabled. ";
    }
    else
    {
        filter_status_msg.message = filter_status_msg.message + "12. Atmospheric Altitude NOT Enabled. ";
        filter_status_msg.level = 1; // WARN state
    }
    if (system_state_packet.filter_status.b.external_position_active)
    {
        filter_status_msg.message = filter_status_msg.message + "13. External Position Active. ";
    }
    else
    {
        filter_status_msg.level = 1; // WARN state
        filter_status_msg.message = filter_status_msg.message + "13. External Position NOT Active. ";
    }
    if (system_state_packet.filter_status.b.external_velocity_active)
    {
        filter_status_msg.message = filter_status_msg.message + "14. External Velocity Active. ";
    }
    else
    {
        filter_status_msg.level = 1; // WARN state
        filter_status_msg.message = filter_status_msg.message + "14. External Velocity NOT Active. ";
    }
    if (system_state_packet.filter_status.b.external_heading_active)
    {
        filter_status_msg.message = filter_status_msg.message + "15. External Heading Active. ";
    }
    else
    {
        filter_status_msg.level = 1; // WARN state
        filter_status_msg.message = filter_status_msg.message + "15. External Heading NOT Active. ";
    }
}

void load_orientation(const float *rpy, geometry_msgs::Quaternion &orientation, bool should_discard_heading)
{
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

float deg(const float rad)
{
    return rad * 180.0 / M_PI;
}

void publish_info_data(const ros::Publisher &pub, geometry_msgs::Vector3Stamped pose_errors_msg,
                       const int gnss_fix_type, const int heading_initialised, const int dual_antenna_heading_active,
                       const size_t satelites, const float hdop, const float vdop, const int odometer_active,
                       const float odometer_speed)
{

    advanced_navigation_driver::InfoPanelData msg;

    msg.roll_error = deg(pose_errors_msg.vector.x);
    msg.pitch_error = deg(pose_errors_msg.vector.y);

    msg.fix_type = gnss_fix_type;
    msg.heading = heading_initialised;
    msg.antena = dual_antenna_heading_active;

    msg.satelites = satelites;
    msg.hdop = hdop;
    msg.vdop = vdop;

    msg.odometer_active = odometer_active;
    msg.odometer_speed = odometer_speed;

    pub.publish(msg);
}

void publish_info_data_failure(const ros::Publisher &pub)
{
    advanced_navigation_driver::InfoPanelError msg;
    msg.error = "IMU failure occoured.";
    pub.publish(msg);
}

class JsonGenerator
{
public:
    JsonGenerator(void) : isOpened(false), isFirst(true)
    {
    }

    void openFile(const string &filename)
    {
        file.open(filename.c_str());
        file << endl
             << "[" << endl;
        isOpened = true;
    }

    void genDict(const float timeStamp,
                 const geometry_msgs::Quaternion &quaternion,
                 const geometry_msgs::Quaternion &quaternion_full,
                 const geometry_msgs::Vector3 eulerStdDev,
                 const geometry_msgs::Vector3 angularSpeed)
    {
        if (isOpened)
        {
            if (!isFirst)
            {
                file << ", " << endl;
            }
            isFirst = false;
            file << std::fixed << "{ \"type\":\"quat_data\", \"timeStamp\":" << std::setprecision(3) << timeStamp << ", ";
            file << std::setprecision(8);
            file << "\"quaternion\":[" << quaternion.x << ", " << quaternion.y << ", " << quaternion.z << ", " << quaternion.w << "], ";
            file << "\"quaternion_full\":[" << quaternion_full.x << ", " << quaternion_full.y << ", " << quaternion_full.z << ", " << quaternion_full.w << "], ";
            file << "\"error\":[" << eulerStdDev.x << ", " << eulerStdDev.y << ", " << eulerStdDev.z << "], ";
            file << "\"angularSpeed\":[" << angularSpeed.x << ", " << angularSpeed.y << ", " << angularSpeed.z << "] }" << flush;
        }
    }

    void close(void)
    {
        if (isOpened)
        {
            file << "]" << endl;
            file.close();
        }
    }

private:
    ofstream file;
    bool isOpened, isFirst;
};

JsonGenerator generator;

void shutdownHandler(int sig)
{
    generator.close();
    ros::shutdown();
}

// SpatialDual publish NEMEA timestamps within current hour to Velodyne
float fixHourOverflow(const float hourTimestamp)
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

void send_and_free_packet(an_packet_t *packet)
{
    an_packet_encode(packet);
    SendBuf(an_packet_pointer(packet), an_packet_size(packet));
    an_packet_free(&packet);
}

void request_packet(packet_id_e packet_id)
{
    an_packet_t *an_packet = encode_request_packet(packet_id);
    send_and_free_packet(an_packet);
}

std_msgs::Float32 norm(const float velocity[])
{
    std_msgs::Float32 msg;
    msg.data = sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1] + velocity[2] * velocity[2]);
    return msg;
}

int main(int argc, char *argv[])
{
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
    pnh.param<int>("uart_baud_rate", baud_rate, 1000000);
    pnh.param<std::string>("output_file", output_filename, "");
    pnh.param<std::string>("output_binary_log", output_binary_log, "");
    pnh.param<bool>("discard_heading", should_discard_heading, true);

    // Initialise Publishers and Topics //
    ros::Publisher orientation_pub = nh.advertise<geometry_msgs::PoseStamped>("imu_pose", 10);
    ros::Publisher orientation_err_pub = nh.advertise<geometry_msgs::Vector3Stamped>("imu_pose_errors", 10);
    ros::Publisher timeref_pub = nh.advertise<sensor_msgs::TimeReference>("imu_timeref", 10);
    ros::Publisher system_status_pub = nh.advertise<diagnostic_msgs::DiagnosticStatus>("imu_status", 10);
    ros::Publisher filter_status_pub = nh.advertise<diagnostic_msgs::DiagnosticStatus>("imu_filter_status", 10);
    ros::Publisher data_pub = nh.advertise<advanced_navigation_driver::InfoPanelData>("info_panel_data", 10);
    ros::Publisher fail_pub = nh.advertise<advanced_navigation_driver::InfoPanelError>("info_panel_error", 10);
    ros::Publisher speed_pub = nh.advertise<std_msgs::Float32>("speed", 10);

    geometry_msgs::PoseStamped orientation_msg, orientation_msg_with_heading;
    orientation_msg.header.frame_id = "imu_base";
    orientation_msg_with_heading.header.frame_id = "imu_base";

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

    if (OpenComport(const_cast<char *>(com_port.c_str()), baud_rate))
    {
        printf("Could not open serial port: %s \n", com_port.c_str());
        exit(EXIT_FAILURE);
    }
    ROS_INFO_STREAM("Serial port successfully opened");

    an_decoder_initialise(&an_decoder);

    if (!output_filename.empty())
    {
        generator.openFile(output_filename);
        ROS_INFO_STREAM("OUTPUT_FILE: " << output_filename);
    }
    ROS_INFO_STREAM("Output JSON file successfully opened");

    if (!output_binary_log.empty())
    {
        anpp_logger.open(output_binary_log);
        ROS_INFO_STREAM("Binary log saved to: " << output_binary_log);
    }
    ROS_INFO_STREAM("Output ANPP log file successfully opened");

    bool imu_filter_failure = false;
    bool dual_antena_package_received = false;
    bool alignment_package_received = false;
    size_t satelites_cnt = 0;
    float hdop = -1.0;
    float vdop = -1.0;
    int odometer_active = 0;
    float odometer_speed = 0.0;

    int last_gnss_fix_type, last_heading_initialized, last_dual_antena_active;

    // Loop continuously, polling for packets
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

                ros::Time ros_now = ros::Time::now();
                //cerr << "Packet came, ID: " << (int)an_packet->id << endl;

                // system state packet //
                if (an_packet->id == packet_id_system_state)
                {
                    if (decode_system_state_packet(&system_state_packet, an_packet) == 0)
                    {
                        // System status
                        load_system_status(system_state_packet, system_status_msg);
                        system_status_pub.publish(system_status_msg);

                        // Filter status
                        load_filter_status(system_state_packet, filter_status_msg);
                        filter_status_pub.publish(filter_status_msg);

                        // SpatialDual publish NEMEA timestamps within current hour to Velodyne
                        float hourTimestamp = system_state_packet.unix_time_seconds % 3600 + system_state_packet.microseconds * 1e-6;
                        float timestamp = fixHourOverflow(hourTimestamp);
                        time_ref.header.stamp = ros_now;
                        time_ref.time_ref = ros::Time(timestamp);
                        timeref_pub.publish(time_ref);

                        // IMU
                        if (system_state_packet.filter_status.b.orientation_filter_initialised == 0)
                        {
                            imu_filter_failure = true;
                        }

                        orientation_msg.header.stamp = ros_now;
                        load_orientation(system_state_packet.orientation, orientation_msg.pose.orientation, should_discard_heading);

                        orientation_msg_with_heading.header.stamp = ros_now;
                        load_orientation(system_state_packet.orientation, orientation_msg_with_heading.pose.orientation, false);
                        orientation_pub.publish(orientation_msg_with_heading);

                        tf_msg.stamp_ = ros_now;
                        tf_msg.setOrigin(tf::Vector3(
                            orientation_msg.pose.position.x, orientation_msg.pose.position.y, orientation_msg.pose.position.z));
                        tf_msg.setRotation(tf::Quaternion(
                            orientation_msg_with_heading.pose.orientation.x, orientation_msg_with_heading.pose.orientation.y,
                            orientation_msg_with_heading.pose.orientation.z, orientation_msg_with_heading.pose.orientation.w));
                        tf_broadcaster.sendTransform(tf_msg);

                        geometry_msgs::Vector3 angular_velocity;
                        angular_velocity.x = system_state_packet.angular_velocity[0];
                        angular_velocity.y = system_state_packet.angular_velocity[1];
                        angular_velocity.z = system_state_packet.angular_velocity[2];

                        generator.genDict(timestamp, orientation_msg.pose.orientation, orientation_msg_with_heading.pose.orientation,
                                          orientation_errors_msg.vector, angular_velocity);

                        if (!alignment_package_received)
                        {
                            request_packet(packet_id_installation_alignment);
                        }
                        if (!dual_antena_package_received)
                        {
                            request_packet(packet_id_dual_antenna_configuration);
                        }

                        last_gnss_fix_type = system_state_packet.filter_status.b.gnss_fix_type;
                        last_heading_initialized = system_state_packet.filter_status.b.heading_initialised;

                        speed_pub.publish(norm(system_state_packet.velocity));
                    }
                }

                if (an_packet->id == packet_id_status)
                {
                    status_packet_t status_packet;
                    if (decode_status_packet(&status_packet, an_packet) == 0)
                    {
                        last_dual_antena_active = status_packet.filter_status.b.dual_antenna_heading_active;
                    }
                }

                if (an_packet->id == packet_id_satellites)
                {
                    satellites_packet_t satellites_packet;
                    if (decode_satellites_packet(&satellites_packet, an_packet) == 0)
                    {
                        satelites_cnt = satellites_packet.gps_satellites + satellites_packet.glonass_satellites +
                                        satellites_packet.beidou_satellites + satellites_packet.galileo_satellites + satellites_packet.sbas_satellites;
                        hdop = satellites_packet.hdop;
                        vdop = satellites_packet.vdop;
                    }
                }

                if (an_packet->id == packet_id_euler_orientation_standard_deviation)
                {
                    if (decode_euler_orientation_standard_deviation_packet(&euler_orientation_standard_deviation_packet, an_packet) == 0)
                    {
                        // IMU error
                        orientation_errors_msg.header.stamp = ros_now;
                        orientation_errors_msg.vector.x = euler_orientation_standard_deviation_packet.standard_deviation[0];
                        orientation_errors_msg.vector.y = euler_orientation_standard_deviation_packet.standard_deviation[1];
                        orientation_errors_msg.vector.z = euler_orientation_standard_deviation_packet.standard_deviation[2];
                        orientation_err_pub.publish(orientation_errors_msg);

                        if (imu_filter_failure)
                        {
                            publish_info_data_failure(fail_pub);
                        }
                        else
                        {
                            publish_info_data(data_pub, orientation_errors_msg,
                                              last_gnss_fix_type, last_heading_initialized, last_dual_antena_active,
                                              satelites_cnt, hdop, vdop, odometer_active, odometer_speed);
                        }
                    }
                }

                if (an_packet->id == packet_id_odometer_state)
                {
                    odometer_state_packet_t odometer_state_packet;
                    if (decode_odometer_state_packet(&odometer_state_packet, an_packet) == 0)
                    {
                        odometer_active = odometer_state_packet.active;
                        odometer_speed = odometer_state_packet.speed;
                    }
                }

                dual_antena_package_received |= (an_packet->id == packet_id_dual_antenna_configuration);
                alignment_package_received |= (an_packet->id == packet_id_installation_alignment);

                anpp_logger.write(*an_packet);

                an_packet_free(&an_packet);
            }
        }
    }
