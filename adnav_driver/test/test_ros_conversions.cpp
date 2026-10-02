#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "adnav_driver.h"

namespace {

// Round-trips a packet through the wire encoding and the stream decoder.
an_packet_t* wireRoundTrip(uint8_t id, const float* data, uint8_t count) {
	const uint8_t length = count * sizeof(float);
	an_packet_t* sent = an_packet_allocate(length, id);
	std::memcpy(sent->data, data, length);
	an_packet_encode(sent);

	an_decoder_t decoder;
	an_decoder_initialise(&decoder);
	std::memcpy(an_decoder_pointer(&decoder), an_packet_pointer(sent), an_packet_size(sent));
	an_decoder_increment(&decoder, an_packet_size(sent));
	an_packet_free(&sent);
	return an_packet_decode(&decoder);
}

}  // namespace

TEST(RosConversions, FrdToFluNegatesRightAndDown) {
	const float frd[3] = {1.0f, 2.0f, 3.0f};
	const auto flu = adnav::frdToFlu(frd);
	EXPECT_EQ(flu.x, 1.0);
	EXPECT_EQ(flu.y, -2.0);
	EXPECT_EQ(flu.z, -3.0);
}

TEST(RosConversions, OrientationMatchesPhysicalAttitude) {
	const auto axis = [](const tf2::Quaternion& q, double x, double y, double z) {
		return tf2::quatRotate(q, tf2::Vector3(x, y, z));
	};

	// Heading north, level: forward is ENU north, left is ENU west.
	const float north[3] = {0.0f, 0.0f, 0.0f};
	EXPECT_NEAR(axis(adnav::nedFrdToEnuFlu(north), 1, 0, 0).y(), 1.0, 1e-6);
	EXPECT_NEAR(axis(adnav::nedFrdToEnuFlu(north), 0, 1, 0).x(), -1.0, 1e-6);

	// Heading east: forward is ENU east.
	const float east[3] = {0.0f, 0.0f, static_cast<float>(M_PI / 2.0)};
	EXPECT_NEAR(axis(adnav::nedFrdToEnuFlu(east), 1, 0, 0).x(), 1.0, 1e-6);

	// Positive Certus pitch is nose up.
	const float nose_up[3] = {0.0f, 0.2f, 0.0f};
	EXPECT_NEAR(axis(adnav::nedFrdToEnuFlu(nose_up), 1, 0, 0).z(), std::sin(0.2), 1e-6);

	// Positive Certus roll is right side down, so the left side rises.
	const float right_down[3] = {0.2f, 0.0f, 0.0f};
	EXPECT_NEAR(axis(adnav::nedFrdToEnuFlu(right_down), 0, 1, 0).z(), std::sin(0.2), 1e-6);
}

TEST(RosConversions, PacketOutputRate) {
	const std::vector<int64_t> request{20, 20, 28, 10, 33, 50};
	EXPECT_DOUBLE_EQ(adnav::packetOutputRateHz(request, 20, 1000), 50.0);
	EXPECT_DOUBLE_EQ(adnav::packetOutputRateHz(request, 28, 1000), 100.0);
	EXPECT_DOUBLE_EQ(adnav::packetOutputRateHz(request, 33, 1000), 20.0);
	EXPECT_DOUBLE_EQ(adnav::packetOutputRateHz(request, 43, 1000), 0.0);
	EXPECT_DOUBLE_EQ(adnav::packetOutputRateHz(request, 20, 2000), 25.0);
}

TEST(RosConversions, SharesPacket20Period) {
	EXPECT_TRUE(adnav::sharesPacket20Period({20, 20, 26, 20, 28, 10}, 26));
	EXPECT_FALSE(adnav::sharesPacket20Period({20, 100, 26, 20}, 26));
	EXPECT_FALSE(adnav::sharesPacket20Period({20, 20, 26, 100}, 26));
	EXPECT_TRUE(adnav::sharesPacket20Period({20, 20, 28, 20}, 26));
	EXPECT_TRUE(adnav::sharesPacket20Period({26, 20, 28, 20}, 26));
}

TEST(RosConversions, SensorVarianceIsBandLimitedNoisePlusBias) {
	EXPECT_DOUBLE_EQ(adnav::sensorVariance(0.1, 0.0, 50.0), 0.01 * 25.0);
	EXPECT_DOUBLE_EQ(adnav::sensorVariance(0.1, 0.2, 50.0), 0.01 * 25.0 + 0.04);
	EXPECT_DOUBLE_EQ(adnav::sensorVariance(0.1, 0.2, 0.0), 0.0);
}

TEST(RosConversions, DatasheetGyroVarianceAt50Hz) {
	// 0.004 deg/s/sqrt(Hz) over 25 Hz is 0.02 deg/s.
	const double sd = 0.02 * M_PI / 180.0;
	const double variance = adnav::sensorVariance(adnav::DEFAULT_GYRO_NOISE_DENSITY, 0.0, 50.0);
	EXPECT_NEAR(variance, sd * sd, 1e-15);
	EXPECT_GT(adnav::sensorVariance(adnav::DEFAULT_GYRO_NOISE_DENSITY, adnav::DEFAULT_GYRO_BIAS_INSTABILITY,
		50.0), variance);
}

TEST(RosConversions, EulerOrientationCovarianceIsVariance) {
	const float sd[3] = {0.01f, 0.02f, 0.03f};
	an_packet_t* packet = wireRoundTrip(packet_id_euler_orientation_standard_deviation, sd, 3);
	ASSERT_NE(packet, nullptr);
	euler_orientation_standard_deviation_packet_t decoded;
	ASSERT_EQ(decode_euler_orientation_standard_deviation_packet(&decoded, packet), 0);
	an_packet_free(&packet);

	const auto covariance = adnav::orientationCovariance(decoded);
	EXPECT_NEAR(covariance[0], 0.01 * 0.01, 1e-8);
	EXPECT_NEAR(covariance[4], 0.02 * 0.02, 1e-8);
	EXPECT_NEAR(covariance[8], 0.03 * 0.03, 1e-8);
	for (size_t i : {1, 2, 3, 5, 6, 7}) {
		EXPECT_EQ(covariance[i], 0.0);
	}
}

TEST(RosConversions, BodyVelocityCovarianceFollowsAttitude) {
	const float ned_sd[3] = {0.1f, 0.2f, 0.5f};

	// Heading north, level: surge takes north, sway takes east, heave takes down.
	const float north[3] = {0.0f, 0.0f, 0.0f};
	auto covariance = adnav::bodyVelocityCovariance(adnav::nedFrdToEnuFlu(north), ned_sd);
	EXPECT_NEAR(covariance[0], 0.01, 1e-8);
	EXPECT_NEAR(covariance[4], 0.04, 1e-8);
	EXPECT_NEAR(covariance[8], 0.25, 1e-8);

	// Heading east: surge takes east.
	const float east[3] = {0.0f, 0.0f, static_cast<float>(M_PI / 2.0)};
	covariance = adnav::bodyVelocityCovariance(adnav::nedFrdToEnuFlu(east), ned_sd);
	EXPECT_NEAR(covariance[0], 0.04, 1e-8);
	EXPECT_NEAR(covariance[4], 0.01, 1e-8);

	// Rolled 10 degrees heading north: sway mixes east and down, and stays symmetric.
	const float rolled[3] = {static_cast<float>(10.0 * M_PI / 180.0), 0.0f, 0.0f};
	covariance = adnav::bodyVelocityCovariance(adnav::nedFrdToEnuFlu(rolled), ned_sd);
	const double c = std::cos(rolled[0]);
	const double s = std::sin(rolled[0]);
	EXPECT_NEAR(covariance[4], c * c * 0.04 + s * s * 0.25, 1e-6);
	EXPECT_NEAR(covariance[5], covariance[7], 1e-12);
	EXPECT_GT(std::abs(covariance[5]), 1e-4);
}

TEST(RosConversions, AngularAccelerationPacketToFluAccel) {
	const float angular_frd[3] = {0.1f, 0.2f, 0.3f};
	an_packet_t* packet = wireRoundTrip(packet_id_angular_acceleration, angular_frd, 3);
	ASSERT_NE(packet, nullptr);
	angular_acceleration_packet_t decoded;
	ASSERT_EQ(decode_angular_acceleration_packet(&decoded, packet), 0);
	an_packet_free(&packet);

	geometry_msgs::msg::Vector3 linear;
	linear.x = 1.0;
	linear.y = -2.0;
	linear.z = 9.8;
	const std::array<double, 9> linear_covariance{0.5, 0.1, 0.0, 0.1, 0.6, 0.0, 0.0, 0.0, 0.7};
	const auto msg = adnav::accelerationMsg(decoded, linear, linear_covariance, 0.25);

	EXPECT_FLOAT_EQ(msg.accel.accel.angular.x, 0.1f);
	EXPECT_FLOAT_EQ(msg.accel.accel.angular.y, -0.2f);
	EXPECT_FLOAT_EQ(msg.accel.accel.angular.z, -0.3f);
	EXPECT_EQ(msg.accel.accel.linear, linear);

	for (size_t row = 0; row < 6; ++row) {
		for (size_t col = 0; col < 6; ++col) {
			double expected = row == col ? 0.25 : 0.0;
			if (row < 3 && col < 3) {
				expected = linear_covariance[row * 3 + col];
			}
			EXPECT_EQ(msg.accel.covariance[row * 6 + col], expected) << row << "," << col;
		}
	}
}

TEST(RosConversions, WrongLengthAngularAccelerationIsRejected) {
	const float two[2] = {0.1f, 0.2f};
	an_packet_t* packet = wireRoundTrip(packet_id_angular_acceleration, two, 2);
	ASSERT_NE(packet, nullptr);
	angular_acceleration_packet_t decoded;
	EXPECT_NE(decode_angular_acceleration_packet(&decoded, packet), 0);
	an_packet_free(&packet);
}

TEST(RosConversions, MagneticFieldIsTeslaInFlu) {
	const float frd_milligauss[3] = {100.0f, 200.0f, 300.0f};
	const auto field = adnav::magneticFieldTesla(frd_milligauss);
	EXPECT_NEAR(field.x, 1.0e-5, 1e-12);
	EXPECT_NEAR(field.y, -2.0e-5, 1e-12);
	EXPECT_NEAR(field.z, -3.0e-5, 1e-12);
}

namespace {

// Feeds one sequence 1 ms apart from start_s, marking packet 20 with utc_valid. Returns hasState()
// after each packet.
std::vector<bool> feed(adnav::SequenceTracker& tracker, const std::vector<int>& ids, double start_s,
	bool utc_valid = true) {
	std::vector<bool> has_state;
	double t = start_s;
	for (const int id : ids) {
		tracker.observe(id, t);
		if (id == packet_id_system_state) {
			tracker.markState(utc_valid, builtin_interfaces::msg::Time());
		}
		has_state.push_back(tracker.hasState());
		t += 0.001;
	}
	return has_state;
}

}  // namespace

TEST(SequenceTracker, PacketsAfterPacket20ShareItsSequence) {
	adnav::SequenceTracker tracker;
	tracker.setMaxGap(0.01);
	const std::vector<int> full{20, 25, 26, 28, 33, 36, 43};
	EXPECT_EQ(feed(tracker, full, 0.0), std::vector<bool>(7, true));

	// A packet 33 alone starts a sequence without packet 20.
	EXPECT_EQ(feed(tracker, {33}, 0.050), std::vector<bool>{false});
	EXPECT_EQ(feed(tracker, full, 0.060), std::vector<bool>(7, true));
}

TEST(SequenceTracker, SystemPacketsDoNotBreakASequence) {
	adnav::SequenceTracker tracker;
	tracker.setMaxGap(0.01);
	EXPECT_EQ(feed(tracker, {20, 0, 25, 3, 43}, 0.0), std::vector<bool>(5, true));
}

TEST(SequenceTracker, InvalidUtcGivesNoState) {
	adnav::SequenceTracker tracker;
	tracker.setMaxGap(0.01);
	EXPECT_EQ(feed(tracker, {20, 36, 43}, 0.0, false), std::vector<bool>(3, false));
}

TEST(SequenceTracker, LowerIdWithinTheGapStartsANewSequence) {
	adnav::SequenceTracker tracker;
	tracker.setMaxGap(0.01);
	EXPECT_EQ(feed(tracker, {20, 25, 43}, 0.0), std::vector<bool>(3, true));
	// A 25 after the 43, still inside the gap, belongs to a sequence without its packet 20.
	EXPECT_EQ(feed(tracker, {25}, 0.004), std::vector<bool>{false});
}

TEST(SequenceTracker, LostPacket20EndsTheSequenceAtTheBoundary) {
	adnav::SequenceTracker tracker;
	tracker.setMaxGap(0.01);
	EXPECT_EQ(feed(tracker, {20, 25, 26}, 0.0), std::vector<bool>(3, true));
	// The next sequence loses 20, 25 and 26; its 28 has a higher ID but arrives 20 ms later.
	EXPECT_EQ(feed(tracker, {28, 36, 43}, 0.020), std::vector<bool>(3, false));
}

TEST(SequenceTracker, LostPacket20InsideTheGapIsCaughtByIdOrder) {
	adnav::SequenceTracker tracker;
	tracker.setMaxGap(0.01);
	EXPECT_EQ(feed(tracker, {20, 25, 26, 28, 36, 43}, 0.0), std::vector<bool>(6, true));
	// The next sequence loses its packet 20 and arrives within the gap.
	EXPECT_EQ(feed(tracker, {25, 36, 43}, 0.007), std::vector<bool>(3, false));
}

namespace {

// Packet 20 payload with only the fields the sequence stamp depends on.
std::vector<uint8_t> systemStatePayload(uint32_t unix_seconds, uint32_t microseconds, bool utc_valid) {
	std::vector<uint8_t> data(100, 0);
	const uint16_t filter_status = utc_valid ? (1u << 3) : 0u;  // bit 3: UTC time initialised
	std::memcpy(&data[2], &filter_status, sizeof(filter_status));
	std::memcpy(&data[4], &unix_seconds, sizeof(unix_seconds));
	std::memcpy(&data[8], &microseconds, sizeof(microseconds));
	return data;
}

void appendPacket(std::vector<uint8_t>& stream, uint8_t id, const std::vector<uint8_t>& data) {
	an_packet_t* packet = an_packet_allocate(data.size(), id);
	std::memcpy(packet->data, data.data(), data.size());
	an_packet_encode(packet);
	const uint8_t* bytes = an_packet_pointer(packet);
	stream.insert(stream.end(), bytes, bytes + an_packet_size(packet));
	an_packet_free(&packet);
}

struct Decoded {
	int id;
	bool has_state;
	builtin_interfaces::msg::Time stamp;
};

// Decodes a byte stream the way the driver's read loop does, one packet per millisecond.
std::vector<Decoded> decodeStream(adnav::SequenceTracker& tracker, const std::vector<uint8_t>& stream,
	double start_s) {
	an_decoder_t decoder;
	an_decoder_initialise(&decoder);
	std::memcpy(an_decoder_pointer(&decoder), stream.data(), stream.size());
	an_decoder_increment(&decoder, stream.size());

	std::vector<Decoded> decoded;
	double t = start_s;
	an_packet_t* packet;
	while ((packet = an_packet_decode(&decoder)) != nullptr) {
		tracker.observe(packet->id, t);
		system_state_packet_t state;
		if (packet->id == packet_id_system_state && decode_system_state_packet(&state, packet) == 0) {
			builtin_interfaces::msg::Time stamp;
			stamp.sec = state.unix_time_seconds;
			stamp.nanosec = state.microseconds * 1000;
			tracker.markState(state.filter_status.b.utc_time_initialised, stamp);
		}
		decoded.push_back({packet->id, tracker.hasState(), tracker.stamp()});
		an_packet_free(&packet);
		t += 0.001;
	}
	return decoded;
}

}  // namespace

TEST(SequenceTracker, EncodedSequenceSharesPacket20Time) {
	adnav::SequenceTracker tracker;
	tracker.setMaxGap(0.01);
	std::vector<uint8_t> stream;
	appendPacket(stream, packet_id_system_state, systemStatePayload(1000, 500000, true));
	for (const uint8_t id : {25, 26, 28, 36}) {
		appendPacket(stream, id, std::vector<uint8_t>(id == 28 ? 48 : 12, 0));
	}
	const float angular[3] = {0.1f, 0.2f, 0.3f};
	std::vector<uint8_t> angular_bytes(12);
	std::memcpy(angular_bytes.data(), angular, sizeof(angular));
	appendPacket(stream, packet_id_angular_acceleration, angular_bytes);

	const auto decoded = decodeStream(tracker, stream, 0.0);
	ASSERT_EQ(decoded.size(), 6u);
	for (const auto& packet : decoded) {
		EXPECT_TRUE(packet.has_state) << packet.id;
		EXPECT_EQ(packet.stamp.sec, 1000) << packet.id;
		EXPECT_EQ(packet.stamp.nanosec, 500000000u) << packet.id;
	}
}

TEST(SequenceTracker, EncodedSequenceWithoutValidUtcHasNoState) {
	adnav::SequenceTracker tracker;
	tracker.setMaxGap(0.01);
	std::vector<uint8_t> stream;
	appendPacket(stream, packet_id_system_state, systemStatePayload(12, 0, false));
	appendPacket(stream, packet_id_body_velocity, std::vector<uint8_t>(12, 0));
	appendPacket(stream, packet_id_angular_acceleration, std::vector<uint8_t>(12, 0));

	for (const auto& packet : decodeStream(tracker, stream, 0.0)) {
		EXPECT_FALSE(packet.has_state) << packet.id;
	}
}

TEST(SequenceTracker, EncodedSequenceMissingPacket20InheritsNothing) {
	adnav::SequenceTracker tracker;
	tracker.setMaxGap(0.01);
	std::vector<uint8_t> first;
	appendPacket(first, packet_id_system_state, systemStatePayload(1000, 0, true));
	appendPacket(first, packet_id_angular_acceleration, std::vector<uint8_t>(12, 0));
	decodeStream(tracker, first, 0.0);

	std::vector<uint8_t> second;
	appendPacket(second, packet_id_body_velocity, std::vector<uint8_t>(12, 0));
	appendPacket(second, packet_id_angular_acceleration, std::vector<uint8_t>(12, 0));
	for (const auto& packet : decodeStream(tracker, second, 0.005)) {
		EXPECT_FALSE(packet.has_state) << packet.id;
	}
}
