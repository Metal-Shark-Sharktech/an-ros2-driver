/****************************************************************/
/*                                                              */
/*                       Advanced Navigation                    */
/*         				Device Communications		  			*/
/*          Copyright 2024, Advanced Navigation Pty Ltd         */
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

#ifndef ADVANCED_NAVIGATION_DRIVER_H
#define ADVANCED_NAVIGATION_DRIVER_H

// C System Headers
#include <stdio.h>      // FILE
#include <sstream>      // Stringstream
#include <fstream>

// C++ System Headers
#include <algorithm>    // std::max
#include <chrono>       // Time, std::chrono
#include <functional>   // std::placeholder
#include <memory>       // smart pointers
#include <vector>       // std::vector
#include <string>       // std::string
#include <mutex>        // std::mutex, std:unique_lock
#include <condition_variable>   // std::condition_variable

#include <rs232.h>
#include <an_packet_protocol.h>
#include <ins_packets.h>
#include <adnav_utils.h>
#include <adnav_comms.h>
#include <adnav_logger.h>
#include <adnav_ntrip.h>
#include "ros_conversions.h"

// Adnav_interfaces
#include <adnav_interfaces/srv/packet_periods.hpp>
#include <adnav_interfaces/srv/packet_timer_period.hpp>
#include <adnav_interfaces/srv/request_packets.hpp>
#include <adnav_interfaces/srv/ntrip.hpp>
#include <adnav_interfaces/msg/llh.hpp>

// ROS2 Packages, Services, Messages
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <std_srvs/srv/empty.hpp>

#if defined(WIN32) || defined(_WIN32)
    #pragma comment(lib, "ws2_32.lib")  // Winsock Library
    #pragma comment(lib, "iphlpapi.lib")  // IP Help API Library
    #include<winsock2.h>
    #include<iphlpapi.h>
    #include<netioapi.h>
    #include<WS2tcpip.h>
#else
    #include <unistd.h>
    #include <netdb.h>
    #include <ifaddrs.h>
    #include <sys/socket.h>
    #include <arpa/inet.h>
#endif

#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>


namespace adnav {

constexpr const double RADIANS_TO_DEGREES = (180.0/M_PI);
constexpr const int    DEFAULT_BAUD_RATE = 115200;
constexpr const int    DEFAULT_TIMER_PERIOD = 20000;
constexpr const int    DEFAULT_PACKET_TIMER_PERIOD = 10000;
constexpr const char * DEFAULT_COM_PORT = "ttyUSB0";
constexpr const int    DEFAULT_PACKET_REQUEST[4] = {20, 10, 28, 10};
constexpr const char * DEFAULT_PACKET_REQUEST_STR = "20, 10, 28, 10";
constexpr const char * DEFAULT_IP_ADDRESS = "0.0.0.0";
constexpr const bool   DEFAULT_NTRIP_STATE = false;
constexpr const int    DEFAULT_GPGGA_REPORT_PERIOD = 1;  // Second(s)
constexpr const int    DEFAULT_TIMEOUT = 5;
constexpr const int    MAX_TIMER_PERIOD = 65535;
constexpr const int    MIN_TIMER_PERIOD = 1000;
constexpr const int    MIN_PACKET_PERIOD = 1;
constexpr const int    MAX_PACKET_PERIOD = 65535;
// Certus sensor datasheet values in SI units.
constexpr const double DEFAULT_GYRO_NOISE_DENSITY = 0.004 * M_PI / 180.0;          // rad/s/sqrt(Hz)
constexpr const double DEFAULT_GYRO_BIAS_INSTABILITY = 3.0 * M_PI / 180.0 / 3600.0; // rad/s
constexpr const double DEFAULT_ACCEL_NOISE_DENSITY = 100.0e-6 * STANDARD_GRAVITY;   // m/s^2/sqrt(Hz)
constexpr const double DEFAULT_ACCEL_BIAS_INSTABILITY = 20.0e-6 * STANDARD_GRAVITY; // m/s^2
constexpr const double DEFAULT_ANGULAR_ACCELERATION_NOISE = 0.1;  // rad/s^2, unmeasured placeholder
constexpr const int    MIN_PORT = 0;
constexpr const int    MAX_PORT = 65535;

typedef struct {
    bool en;
    std::string ip;
    int port;
    struct hostent *he;
    std::string username;
    std::string password;
    std::string mountpoint;
}ntrip_client_state_t;

class Driver : public rclcpp::Node  // Inheriting gives every "this->" as a pointer to the node.
{
 public:
    // ~~~~ Constructors
    Driver();
    ~Driver();

    /// Output sequences whose imu and accel were dropped.
    struct ImuDrops {
        uint64_t missing_orientation_sd = 0;  ///< packet 26 never arrived
        uint64_t late_orientation_sd = 0;     ///< packet 26 decoded after the sequence gap
    };
    ImuDrops imuDrops();

 private:
    // Debug variables
    int pub_num_ = 0, P28_num_ = 0, P20_num_ = 0, P33_num_ = 0, P0_num_ = 0;

    // Defines what communication method to use, refer to adnav_driver_connection_e.
    int communication_state_;

    // String to hold frame_id
    std::string frame_id_ = "imu_link";

    // String to hold node name.
    std::string node_name_;

    // device communication settings
    std::unique_ptr<adnav::Communicator> communicator_;
    adnav_connections_data_t comms_data_;

    // Packet settings;
    std::vector<int64_t> packet_request_;
    int packet_timer_period_;

    // Log files.
    std::string log_path_;
    adnav::Logger anpp_logger_;
    adnav::Logger rtcm_logger_;

    // ANPP Packet variables
    acknowledge_packet_t acknowledge_packet_;  // only access with protection of acknowledge_mutex_
    device_information_packet_t device_information_packet_;

    // Msgs. Only access with protection of messages_mutex_
    tf2::Quaternion                 orientation_;
    sensor_msgs::msg::Imu           imu_msg_;
    sensor_msgs::msg::Imu           imu_raw_msg_;
    sensor_msgs::msg::MagneticField mag_field_msg_;
    sensor_msgs::msg::NavSatFix     nav_fix_msg_;
    sensor_msgs::msg::FluidPressure baro_msg_;
    sensor_msgs::msg::Temperature   temp_msg_;
    geometry_msgs::msg::Twist       twist_msg_;
    geometry_msgs::msg::TwistWithCovarianceStamped body_twist_msg_;
    geometry_msgs::msg::Pose        pose_msg_;
    geometry_msgs::msg::AccelWithCovarianceStamped accel_msg_;
    diagnostic_msgs::msg::DiagnosticStatus system_status_msg_;
    diagnostic_msgs::msg::DiagnosticStatus filter_status_msg_;

    // Body-frame velocity state. Only access with protection of messages_mutex_.
    // Latest velocity standard deviation (packet 25, NED frame) cached for the
    // covariance of the next body velocity (packet 36).
    velocity_standard_deviation_packet_t velocity_sd_packet_;
    // Whether a velocity standard deviation packet has been received yet.
    bool velocity_sd_received_ = false;

    // Every packet in an output sequence shares the time of validity of its packet 20. Only access
    // with protection of messages_mutex_.
    SequenceTracker sequence_;
    bool raw_sensors_fresh_ = false;

    // The linear acceleration covariance of the current sequence, from its packet 20 attitude and,
    // with packet 26 requested, its packet 26 roll and pitch variances. imu and accel share it and
    // are not published in a sequence without it.
    bool orientation_sd_requested_ = false;
    // Taken at the sequence's packet 20, so a schedule change before its packet 26 cannot alter them.
    std::array<double, 2> sequence_roll_pitch_{};
    double sequence_accel_variance_ = 0.0;
    bool sequence_waits_for_orientation_sd_ = false;
    bool imu_pending_ = false;
    bool pending_orientation_sd_late_ = false;
    ImuDrops imu_drops_;
    bool linear_covariance_ready_ = false;
    std::array<double, 9> linear_covariance_{};

    // Latest packet 20 body rates and acceleration in FLU, for packets later in the same sequence.
    geometry_msgs::msg::Vector3 angular_velocity_flu_;
    geometry_msgs::msg::Vector3 body_acceleration_flu_;

    // Sensor noise parameters and the variances derived from them at the requested packet rates.
    // Only access the variances with protection of messages_mutex_.
    double gyro_noise_density_ = DEFAULT_GYRO_NOISE_DENSITY;
    double gyro_bias_instability_ = DEFAULT_GYRO_BIAS_INSTABILITY;
    double accel_noise_density_ = DEFAULT_ACCEL_NOISE_DENSITY;
    double accel_bias_instability_ = DEFAULT_ACCEL_BIAS_INSTABILITY;
    double angular_acceleration_noise_ = DEFAULT_ANGULAR_ACCELERATION_NOISE;
    double gyro_variance_ = 0.0;
    double accel_variance_ = 0.0;
    double raw_gyro_variance_ = 0.0;
    double raw_accel_variance_ = 0.0;
    double angular_acceleration_variance_ = 0.0;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr             		imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr             		imu_raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr       		nav_sat_fix_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr 			magnetic_field_pub_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr 			barometric_pressure_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr 			temperature_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr 				twist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr body_twist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr 					pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr accel_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr 	system_status_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr 	filter_status_pub_;

    // ~~~~~~~~~~~~~~~ Callback handles and parameters
    // Callback groups Allows the callbacks to be processed on a different thread by
    // the Multithread executor.
    rclcpp::CallbackGroup::SharedPtr publishing_group_;
    rclcpp::CallbackGroup::SharedPtr reading_group_;
    rclcpp::CallbackGroup::SharedPtr service_group_;

    // Parameter callbacks and handlers
    OnSetParametersCallbackHandle::SharedPtr param_set_cb_;
    std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> publish_us_cb_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> read_us_cb_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> packet_request_cb_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> packet_timer_cb_;

    // Timers
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr read_timer_;
    // Timer intervals
    std::chrono::microseconds publish_timer_interval_;
    std::chrono::microseconds read_timer_interval_;

    // Service Handlers
    rclcpp::Service<adnav_interfaces::srv::PacketPeriods>::SharedPtr packet_period_srv_;
    rclcpp::Service<adnav_interfaces::srv::PacketTimerPeriod>::SharedPtr packet_period_timer_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr restart_pub_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr restart_read_srv_;
    rclcpp::Service<adnav_interfaces::srv::RequestPackets>::SharedPtr request_packet_srv_;
    rclcpp::Service<adnav_interfaces::srv::Ntrip>::SharedPtr ntrip_srv_;

    // Threading variables
    std::mutex messages_mutex_;
    std::condition_variable msg_cv_;
    bool msg_write_done_;

    std::mutex acknowledge_mutex_;
    std::condition_variable srv_cv_;
    bool acknowledge_recieve_;

    // NTRIP Variables
    std::unique_ptr<adnav::ntrip::Client> ntrip_client_;
    ntrip_client_state_t ntrip_state_;
    adnav_interfaces::msg::LLH llh_;
    std::ofstream rtcm_log_file_;
    char rtcm_filename_[200];

    //~~~~~~~~~~~~~~~~~~~~~ Private Methods.

    //~~~~~~ Setup Functions
    void waitForDevicePacket();
    void requestDeviceInfo();
    void createPublishers();
    void createServices();
    void deviceSetup();
    void setupParamService();
    void setupParams();

    //~~~~~~ Control Functions
    void recievePackets();
    void publishTimerCallback();
    void RestartPublisher();
    void RestartReader();

    //~~~~~~ Logging Functions
    void openLogFile();
    void statusErrLog(const std::string& errmsg);
    void statusWarnLog(const std::string& warnmsg);

    //~~~~~~ ROS Services
    void srvPacketPeriods(const std::shared_ptr<adnav_interfaces::srv::PacketPeriods::Request> request,
            std::shared_ptr<adnav_interfaces::srv::PacketPeriods::Response> response);
    void srvPacketTimerPeriod(const std::shared_ptr<adnav_interfaces::srv::PacketTimerPeriod::Request> request,
            std::shared_ptr<adnav_interfaces::srv::PacketTimerPeriod::Response> response);
    void srvRequestPackets(const std::shared_ptr<adnav_interfaces::srv::RequestPackets::Request> request,
        std::shared_ptr<adnav_interfaces::srv::RequestPackets::Response> response);
    void srvNtrip(const std::shared_ptr<adnav_interfaces::srv::Ntrip::Request> request,
        std::shared_ptr<adnav_interfaces::srv::Ntrip::Response> response);

    //~~~~~~ Parameter Functions
    rcl_interfaces::msg::SetParametersResult ParamSetCallback(const std::vector<rclcpp::Parameter>& Params);
    rcl_interfaces::msg::SetParametersResult validateBaudRate(const rclcpp::Parameter& parameter);
    rcl_interfaces::msg::SetParametersResult validateComPort(const rclcpp::Parameter& parameter);
    rcl_interfaces::msg::SetParametersResult validatePublishUs(const rclcpp::Parameter& parameter);
    void updatePublishUs(const rclcpp::Parameter& parameter);
    rcl_interfaces::msg::SetParametersResult validateReadUs(const rclcpp::Parameter& parameter);
    void updateReadUs(const rclcpp::Parameter& parameter);
    rcl_interfaces::msg::SetParametersResult validatePacketRequest(const rclcpp::Parameter& parameter);
    void updatePacketRequest(const rclcpp::Parameter& parameter);
    rcl_interfaces::msg::SetParametersResult validatePacketTimer(const rclcpp::Parameter& parameter);
    void updatePacketTimer(const rclcpp::Parameter& parameter);
    void validateAndSaveIPAddress(const rclcpp::Parameter& parameter);
    void updatePacketSchedule();
    void completeImu(double roll_variance, double pitch_variance);

    //~~~~~~ NTRIP Functions
    void updateNTRIPClientService();
    void getDataFromHostStr(const std::string& host);
    void NtripReceiveFunction(const char* buffer, int size);


    //~~~~~~ Device Communication Functions
    void encodeAndSend(an_packet_t* an_packet);
    adnav_interfaces::msg::RawAcknowledge AcknowledgeHandler();
    adnav_interfaces::msg::RawAcknowledge SendPacketTimer(int packet_timer_period, bool utc_sync = true , bool permanent = true);
    adnav_interfaces::msg::RawAcknowledge SendPacketPeriods(const std::vector<adnav_interfaces::msg::PacketPeriod>& periods,
        bool clear_existing = true, bool permanent = true);

    //~~~~~~ Decoders
    void decodePackets(an_decoder_t &an_decoder, const int &bytes_received);
    void acknowledgeDecoder(an_packet_t* an_packet);
    void deviceInfoDecoder(an_packet_t* an_packet);
    void systemStateRosDecoder(an_packet_t* an_packet);
    void eulerOrientSDRosDecoder(an_packet_t* an_packet);
    void angularAccelerationRosDecoder(an_packet_t* an_packet);
    void bodyVelocityRosDecoder(an_packet_t* an_packet);
    void velocityStandardDeviationDecoder(an_packet_t* an_packet);
    void ecefPosRosDecoder(an_packet_t* an_packet);
    void rawSensorsRosDecoder(an_packet_t* an_packet);
};

}  // namespace adnav

#endif  // ADVANCED_NAVIGATION_DRIVER_H
