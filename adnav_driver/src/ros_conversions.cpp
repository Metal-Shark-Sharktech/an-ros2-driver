#include "ros_conversions.h"

#include <cmath>

namespace adnav {

geometry_msgs::msg::Vector3 frdToFlu(const float frd[3]) {
	geometry_msgs::msg::Vector3 flu;
	flu.x = frd[0];
	flu.y = -frd[1];
	flu.z = -frd[2];
	return flu;
}

tf2::Quaternion nedFrdToEnuFlu(const float roll_pitch_heading[3]) {
	// Nose up is positive pitch about FRD right but negative about FLU left.
	tf2::Quaternion enu;
	enu.setRPY(roll_pitch_heading[0], -roll_pitch_heading[1], M_PI / 2.0 - roll_pitch_heading[2]);
	return enu;
}

std::array<double, 9> diagonalCovariance(double xx, double yy, double zz) {
	return {xx, 0.0, 0.0,
		0.0, yy, 0.0,
		0.0, 0.0, zz};
}

double packetOutputRateHz(const std::vector<int64_t>& packet_request, int packet_id,
	int packet_timer_period_us) {
	for (size_t i = 0; i + 1 < packet_request.size(); i += 2) {
		if (packet_request[i] == packet_id && packet_request[i + 1] > 0 && packet_timer_period_us > 0) {
			return 1.0e6 / (static_cast<double>(packet_request[i + 1]) * packet_timer_period_us);
		}
	}
	return 0.0;
}

bool sharesPacket20Period(const std::vector<int64_t>& packet_request, int packet_id) {
	int64_t state_period = 0;
	int64_t packet_period = 0;
	for (size_t i = 0; i + 1 < packet_request.size(); i += 2) {
		if (packet_request[i] == packet_id_system_state && state_period == 0) {
			state_period = packet_request[i + 1];
		}
		if (packet_request[i] == packet_id && packet_period == 0) {
			packet_period = packet_request[i + 1];
		}
	}
	return state_period == 0 || packet_period == 0 || state_period == packet_period;
}

double sensorVariance(double noise_density, double bias_instability, double output_rate_hz) {
	if (output_rate_hz <= 0.0) {
		return 0.0;
	}
	return noise_density * noise_density * output_rate_hz / 2.0 + bias_instability * bias_instability;
}

std::array<double, 9> orientationCovariance(
	const euler_orientation_standard_deviation_packet_t& packet) {
	const float* sd = packet.standard_deviation;
	return diagonalCovariance(sd[0] * sd[0], sd[1] * sd[1], sd[2] * sd[2]);
}

std::array<double, 9> bodyVelocityCovariance(const tf2::Quaternion& enu_flu, const float ned_sd[3]) {
	const tf2::Matrix3x3 rotation(enu_flu);
	const tf2::Matrix3x3 enu(
		ned_sd[1] * ned_sd[1], 0.0, 0.0,
		0.0, ned_sd[0] * ned_sd[0], 0.0,
		0.0, 0.0, ned_sd[2] * ned_sd[2]);
	const tf2::Matrix3x3 body = rotation.transpose() * enu * rotation;
	std::array<double, 9> covariance;
	for (int row = 0; row < 3; ++row) {
		for (int col = 0; col < 3; ++col) {
			covariance[row * 3 + col] = body[row][col];
		}
	}
	return covariance;
}

geometry_msgs::msg::Vector3 magneticFieldTesla(const float frd_milligauss[3]) {
	constexpr double TESLA_PER_MILLIGAUSS = 1.0e-7;
	geometry_msgs::msg::Vector3 field = frdToFlu(frd_milligauss);
	field.x *= TESLA_PER_MILLIGAUSS;
	field.y *= TESLA_PER_MILLIGAUSS;
	field.z *= TESLA_PER_MILLIGAUSS;
	return field;
}

void SequenceTracker::observe(int packet_id, double receive_time_s) {
	if (packet_id < START_STATE_PACKETS || packet_id >= START_CONFIGURATION_PACKETS) {
		return;
	}
	if (packet_id <= last_id_ || receive_time_s - last_time_s_ > max_gap_s_) {
		has_packet20_ = false;
		has_state_ = false;
	}
	last_id_ = packet_id;
	last_time_s_ = receive_time_s;
}

geometry_msgs::msg::AccelWithCovarianceStamped accelerationMsg(
	const angular_acceleration_packet_t& packet, const geometry_msgs::msg::Vector3& linear_flu,
	const std::array<double, 9>& linear_covariance, double angular_variance) {
	geometry_msgs::msg::AccelWithCovarianceStamped msg;
	msg.accel.accel.linear = linear_flu;
	msg.accel.accel.angular = frdToFlu(packet.angular_acceleration);

	// Row-major 6x6 over (ax, ay, az, alpha_x, alpha_y, alpha_z).
	msg.accel.covariance.fill(0.0);
	for (size_t row = 0; row < 3; ++row) {
		for (size_t col = 0; col < 3; ++col) {
			msg.accel.covariance[row * 6 + col] = linear_covariance[row * 3 + col];
		}
		msg.accel.covariance[(row + 3) * 7] = angular_variance;
	}
	return msg;
}

}  // namespace adnav
