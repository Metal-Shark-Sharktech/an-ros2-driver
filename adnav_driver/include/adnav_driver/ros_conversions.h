#ifndef ADNAV_DRIVER_ROS_CONVERSIONS_H
#define ADNAV_DRIVER_ROS_CONVERSIONS_H

#include <array>
#include <cstdint>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <ins_packets.h>

namespace adnav {

constexpr double STANDARD_GRAVITY = 9.80665;  // m/s^2

/// Advanced Navigation body frame (forward-right-down) to REP-103 (forward-left-up).
geometry_msgs::msg::Vector3 frdToFlu(const float frd[3]);

/// Orientation of the FLU body in ENU from Certus roll, pitch and heading of the FRD body in NED.
tf2::Quaternion nedFrdToEnuFlu(const float roll_pitch_heading[3]);

/// Row-major 3x3 covariance with the given diagonal.
std::array<double, 9> diagonalCovariance(double xx, double yy, double zz);

/// Output rate in Hz of a packet in a [id, period, id, period, ...] request, or 0 when it is not
/// requested. Periods count packet timer periods of packet_timer_period_us.
double packetOutputRateHz(const std::vector<int64_t>& packet_request, int packet_id,
	int packet_timer_period_us);

/// Whether packet_id is requested at packet 20's period, or either is not requested. Only then does
/// each of its outputs share an output sequence with exactly one packet 20.
bool sharesPacket20Period(const std::vector<int64_t>& packet_request, int packet_id);

/// Variance of an anti-aliased sensor output: white noise over a bandwidth of half the output rate,
/// plus the bias instability. Returns 0 when output_rate_hz is not positive.
double sensorVariance(double noise_density, double bias_instability, double output_rate_hz);

/// Orientation covariance (rad^2) about x, y, z from roll, pitch and heading standard deviations.
std::array<double, 9> orientationCovariance(
	const euler_orientation_standard_deviation_packet_t& packet);

/// Row-major 3x3 derivative of FLU body gravity with respect to (roll, pitch, yaw) of an ENU/FLU
/// intrinsic ZYX attitude. The yaw column is zero: heading turns about gravity.
std::array<double, 9> gravityLeakJacobian(double roll, double pitch);

/// Covariance in FLU of a gravity-free acceleration: accel_variance on each axis plus the gravity
/// that independent roll and pitch errors leak through gravityLeakJacobian. roll and pitch are the
/// FLU angles of the published orientation. Positive definite for accel_variance > 0.
///
/// On the Certus this is an independence-based approximation of the INS output: packet 26 gives
/// marginal standard deviations, and the correlation of attitude error with accelerometer error
/// inside the INS is not reported.
std::array<double, 9> gravityFreeAccelerationCovariance(double accel_variance, double roll_variance,
	double pitch_variance, double roll, double pitch);

/// FLU roll and pitch of an ENU/FLU orientation.
std::array<double, 2> fluRollPitch(const tf2::Quaternion& enu_flu);

/// Covariance in the FLU body of a velocity with the given north, east, down standard deviations.
std::array<double, 9> bodyVelocityCovariance(const tf2::Quaternion& enu_flu, const float ned_sd[3]);

/// Magnetic field in tesla in FLU from the Certus FRD milligauss.
geometry_msgs::msg::Vector3 magneticFieldTesla(const float frd_milligauss[3]);

/// Tracks which state packets share an output sequence with a packet 20. The device outputs each
/// sequence in increasing packet ID order, so a state packet whose ID does not exceed the previous
/// one, or that arrives more than max_gap_s after it, starts a new sequence.
class SequenceTracker {
 public:
	void setMaxGap(double max_gap_s) { max_gap_s_ = max_gap_s; }

	/// Call for every decoded packet before handling it.
	void observe(int packet_id, double receive_time_s);

	/// Call on a packet 20; only one with valid UTC lets the rest of its sequence use its time.
	void markState(bool utc_valid, const builtin_interfaces::msg::Time& stamp) {
		has_packet20_ = true;
		has_state_ = utc_valid;
		stamp_ = stamp;
	}

	/// Whether the current sequence holds a packet 20, whatever its UTC validity.
	bool hasPacket20() const { return has_packet20_; }

	bool hasState() const { return has_state_; }

	/// Time of validity of the current sequence; meaningful only while hasState().
	const builtin_interfaces::msg::Time& stamp() const { return stamp_; }

 private:
	double max_gap_s_ = 0.01;
	int last_id_ = 0;
	double last_time_s_ = 0.0;
	bool has_packet20_ = false;
	bool has_state_ = false;
	builtin_interfaces::msg::Time stamp_;
};

/// Unstamped acceleration in FLU: the angular part from packet 43, the linear part given.
geometry_msgs::msg::AccelWithCovarianceStamped accelerationMsg(
	const angular_acceleration_packet_t& packet, const geometry_msgs::msg::Vector3& linear_flu,
	const std::array<double, 9>& linear_covariance, double angular_variance);

}  // namespace adnav

#endif  // ADNAV_DRIVER_ROS_CONVERSIONS_H
