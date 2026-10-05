#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "adnav_driver.h"

namespace {

using namespace std::chrono_literals;

int freeUdpPort() {
	const int sock = socket(AF_INET, SOCK_DGRAM, 0);
	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	addr.sin_port = 0;
	bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
	socklen_t len = sizeof(addr);
	getsockname(sock, reinterpret_cast<sockaddr*>(&addr), &len);
	close(sock);
	return ntohs(addr.sin_port);
}

// Stands in for the Certus: sends encoded ANPP packets to the driver's UDP port.
class FakeCertus {
 public:
	explicit FakeCertus(int port) : sock_(socket(AF_INET, SOCK_DGRAM, 0)) {
		to_.sin_family = AF_INET;
		to_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
		to_.sin_port = htons(port);
	}
	~FakeCertus() { close(sock_); }

	void send(uint8_t id, const std::vector<uint8_t>& data) {
		an_packet_t* packet = an_packet_allocate(data.size(), id);
		std::memcpy(packet->data, data.data(), data.size());
		an_packet_encode(packet);
		sendto(sock_, an_packet_pointer(packet), an_packet_size(packet), 0,
			reinterpret_cast<const sockaddr*>(&to_), sizeof(to_));
		an_packet_free(&packet);
	}

	// Acknowledges the packet periods the driver sends on a packet_request change.
	void acknowledgePeriodsFor(std::chrono::milliseconds duration) {
		const auto end = std::chrono::steady_clock::now() + duration;
		while (std::chrono::steady_clock::now() < end) {
			send(packet_id_acknowledge, {packet_id_packet_periods, 0, 0, 0});
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}

	void sendFloats(uint8_t id, const std::vector<float>& values) {
		std::vector<uint8_t> data(values.size() * sizeof(float));
		std::memcpy(data.data(), values.data(), data.size());
		send(id, data);
	}

	void sendSystemState(uint32_t unix_seconds, uint32_t microseconds, bool utc_valid,
		const std::array<float, 3>& roll_pitch_heading = {}) {
		std::vector<uint8_t> data(100, 0);
		const uint16_t filter_status = utc_valid ? (1u << 3) : 0u;  // bit 3: UTC time initialised
		std::memcpy(&data[2], &filter_status, sizeof(filter_status));
		std::memcpy(&data[4], &unix_seconds, sizeof(unix_seconds));
		std::memcpy(&data[8], &microseconds, sizeof(microseconds));
		std::memcpy(&data[64], roll_pitch_heading.data(), sizeof(float) * 3);
		send(packet_id_system_state, data);
	}

	// Packets 25, 26 (when orientation_sd is given), 28, 36 and 43 of one output sequence, in ID order.
	void sendRestOfSequence(std::optional<std::array<float, 3>> orientation_sd = {{0.01f, 0.02f, 0.03f}}) {
		sendFloats(packet_id_velocity_standard_deviation, {0.1f, 0.2f, 0.3f});
		if (orientation_sd) {
			sendFloats(packet_id_euler_orientation_standard_deviation,
				{(*orientation_sd)[0], (*orientation_sd)[1], (*orientation_sd)[2]});
		}
		send(packet_id_raw_sensors, std::vector<uint8_t>(48, 0));
		sendFloats(packet_id_body_velocity, {5.0f, 0.5f, 0.0f});
		sendFloats(packet_id_angular_acceleration, {0.1f, 0.2f, 0.3f});
	}

 private:
	int sock_;
	sockaddr_in to_{};
};

template <typename Msg>
class Recorder {
 public:
	Recorder(rclcpp::Node& node, const std::string& topic) {
		sub_ = node.create_subscription<Msg>(topic, 10, [this](const Msg& msg) {
			std::lock_guard<std::mutex> lock(mutex_);
			stamps_.push_back(msg.header.stamp.sec);
			all_.push_back(msg);
		});
	}

	std::vector<int32_t> stamps() {
		std::lock_guard<std::mutex> lock(mutex_);
		return stamps_;
	}

	Msg last() {
		std::lock_guard<std::mutex> lock(mutex_);
		return all_.empty() ? Msg() : all_.back();
	}

	std::vector<Msg> all() {
		std::lock_guard<std::mutex> lock(mutex_);
		return all_;
	}

 private:
	typename rclcpp::Subscription<Msg>::SharedPtr sub_;
	std::mutex mutex_;
	std::vector<int32_t> stamps_;
	std::vector<Msg> all_;
};

// Records the level and message of each system_status, which carries no header.
class StatusRecorder {
 public:
	StatusRecorder(rclcpp::Node& node, const std::string& topic) {
		sub_ = node.create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(topic, 10,
			[this](const diagnostic_msgs::msg::DiagnosticStatus& msg) {
				std::lock_guard<std::mutex> lock(mutex_);
				all_.push_back(msg);
			});
	}

	size_t count() {
		std::lock_guard<std::mutex> lock(mutex_);
		return all_.size();
	}

	diagnostic_msgs::msg::DiagnosticStatus last() {
		std::lock_guard<std::mutex> lock(mutex_);
		return all_.empty() ? diagnostic_msgs::msg::DiagnosticStatus() : all_.back();
	}

 private:
	rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr sub_;
	std::mutex mutex_;
	std::vector<diagnostic_msgs::msg::DiagnosticStatus> all_;
};

// Sends device information packets until the driver has configured the device.
bool connect(FakeCertus& certus, const adnav::Driver& driver) {
	for (int i = 0; i < 500 && !driver.deviceReady(); ++i) {
		certus.send(packet_id_device_information, std::vector<uint8_t>(24, 0));
		std::this_thread::sleep_for(10ms);
	}
	return driver.deviceReady();
}

class DriverDecoding : public ::testing::Test {
 protected:
	void SetUp() override {
		port_ = freeUdpPort();
		char log_dir[] = "/tmp/adnav_driver_test_XXXXXX";
		ASSERT_NE(mkdtemp(log_dir), nullptr);
		const std::vector<std::string> args{"test_driver_decoding", "--ros-args",
			"-p", "comm_select:=3", "-p", "port:=" + std::to_string(port_),
			"-p", "log_path:=" + std::string(log_dir) + "/",
			"-p", "packet_request:=[20,100,25,100,26,100,28,100,36,100,43,100]",
			"-p", "packet_timer_period:=1000", "-p", "read_us:=1000", "-p", "publish_us:=20000"};
		std::vector<const char*> argv;
		for (const auto& arg : args) {
			argv.push_back(arg.c_str());
		}
		rclcpp::init(static_cast<int>(argv.size()), argv.data());

		certus_ = std::make_unique<FakeCertus>(port_);
		driver_ = std::make_shared<adnav::Driver>();

		listener_ = std::make_shared<rclcpp::Node>("listener");
		imu_ = std::make_unique<Recorder<sensor_msgs::msg::Imu>>(*listener_, "/adnav_driver/imu");
		imu_raw_ = std::make_unique<Recorder<sensor_msgs::msg::Imu>>(*listener_, "/adnav_driver/imu_raw");
		accel_ = std::make_unique<Recorder<geometry_msgs::msg::AccelWithCovarianceStamped>>(
			*listener_, "/adnav_driver/accel");
		twist_ = std::make_unique<Recorder<geometry_msgs::msg::TwistWithCovarianceStamped>>(
			*listener_, "/adnav_driver/twist_body");
		system_status_ = std::make_unique<StatusRecorder>(*listener_, "/adnav_driver/system_status");

		executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
		executor_->add_node(driver_);
		executor_->add_node(listener_);
		spin_ = std::thread([this] { executor_->spin(); });
		ASSERT_TRUE(connect(*certus_, *driver_));
		std::this_thread::sleep_for(1s);  // discovery
	}

	void TearDown() override {
		executor_->cancel();
		// Releases the reading callback, which blocks on the socket.
		for (int i = 0; i < 10; ++i) {
			certus_->send(packet_id_device_information, std::vector<uint8_t>(24, 0));
		}
		spin_.join();
		executor_.reset();
		driver_.reset();
		listener_.reset();
		rclcpp::shutdown();
	}

	void settle() { std::this_thread::sleep_for(300ms); }

	int port_ = 0;
	std::unique_ptr<FakeCertus> certus_;
	std::shared_ptr<adnav::Driver> driver_;
	std::shared_ptr<rclcpp::Node> listener_;
	std::unique_ptr<Recorder<sensor_msgs::msg::Imu>> imu_;
	std::unique_ptr<Recorder<sensor_msgs::msg::Imu>> imu_raw_;
	std::unique_ptr<Recorder<geometry_msgs::msg::AccelWithCovarianceStamped>> accel_;
	std::unique_ptr<Recorder<geometry_msgs::msg::TwistWithCovarianceStamped>> twist_;
	std::unique_ptr<StatusRecorder> system_status_;
	std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
	std::thread spin_;
};

}  // namespace

TEST_F(DriverDecoding, SystemStatusKeepsPublishingNoDataAfterTheDeviceStops) {
	certus_->sendSystemState(1000, 0, true);
	settle();
	EXPECT_EQ(system_status_->last().level, diagnostic_msgs::msg::DiagnosticStatus::OK);

	std::this_thread::sleep_for(std::chrono::seconds(adnav::CONNECTION_TIMEOUT) * 2 + 500ms);
	const size_t quiet_count = system_status_->count();
	EXPECT_EQ(system_status_->last().level, diagnostic_msgs::msg::DiagnosticStatus::STALE);

	std::this_thread::sleep_for(std::chrono::seconds(adnav::CONNECTION_TIMEOUT) * 2);
	EXPECT_GT(system_status_->count(), quiet_count);

	certus_->sendSystemState(1001, 0, true);
	settle();
	EXPECT_EQ(system_status_->last().level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(DriverDecoding, SequenceSharesPacket20TimeAndPublishesOnce) {
	certus_->sendSystemState(1000, 500000, true);
	certus_->sendRestOfSequence();
	settle();
	certus_->sendSystemState(1001, 500000, true);
	certus_->sendRestOfSequence();
	settle();

	EXPECT_EQ(imu_->stamps(), (std::vector<int32_t>{1000, 1001}));
	EXPECT_EQ(imu_raw_->stamps(), (std::vector<int32_t>{1000, 1001}));
	EXPECT_EQ(accel_->stamps(), (std::vector<int32_t>{1000, 1001}));
	EXPECT_EQ(twist_->stamps(), (std::vector<int32_t>{1000, 1001}));
	EXPECT_EQ(accel_->last().header.stamp.nanosec, 500000000u);
	EXPECT_FLOAT_EQ(accel_->last().accel.accel.angular.y, -0.2f);
	EXPECT_FLOAT_EQ(twist_->last().twist.twist.linear.y, -0.5f);
	EXPECT_GT(imu_->last().angular_velocity_covariance[0], 0.0);
	EXPECT_NEAR(imu_->last().orientation_covariance[8], 0.03 * 0.03, 1e-8);
}

TEST_F(DriverDecoding, InvalidUtcSuppressesSequenceStampedTopics) {
	certus_->sendSystemState(2000, 0, false);
	certus_->sendRestOfSequence();
	settle();

	EXPECT_TRUE(imu_raw_->stamps().empty());
	EXPECT_TRUE(accel_->stamps().empty());
	EXPECT_TRUE(twist_->stamps().empty());
}

TEST_F(DriverDecoding, SequenceWithoutPacket20InheritsNothing) {
	certus_->sendSystemState(3000, 0, true);
	certus_->sendRestOfSequence();
	settle();
	certus_->sendRestOfSequence();
	settle();

	EXPECT_EQ(accel_->stamps(), std::vector<int32_t>{3000});
	EXPECT_EQ(twist_->stamps(), std::vector<int32_t>{3000});
	EXPECT_EQ(imu_raw_->stamps(), std::vector<int32_t>{3000});
}

namespace {

// Linear acceleration block of an accel covariance.
std::array<double, 9> linearBlock(const geometry_msgs::msg::AccelWithCovarianceStamped& accel) {
	std::array<double, 9> block;
	for (size_t row = 0; row < 3; ++row) {
		for (size_t col = 0; col < 3; ++col) {
			block[row * 3 + col] = accel.accel.covariance[row * 6 + col];
		}
	}
	return block;
}

}  // namespace

TEST_F(DriverDecoding, ImuAndAccelShareTheirSequencesCovariance) {
	const std::array<float, 3> attitude{0.35f, 0.12f, 2.1f};
	const std::array<std::array<float, 3>, 2> sds{{{0.002f, 0.003f, 0.03f}, {0.004f, 0.001f, 0.03f}}};
	for (size_t k = 0; k < sds.size(); ++k) {
		certus_->sendSystemState(4000 + k, 0, true, attitude);
		certus_->sendRestOfSequence(sds[k]);
		settle();
	}

	const auto imus = imu_->all();
	const auto accels = accel_->all();
	ASSERT_EQ(imus.size(), 2u);
	ASSERT_EQ(accels.size(), 2u);
	// packet_request asks for packet 20 at 10 Hz.
	const double accel_variance =
		adnav::sensorVariance(adnav::DEFAULT_ACCEL_NOISE_DENSITY, adnav::DEFAULT_ACCEL_BIAS_INSTABILITY, 10.0);
	const auto expected = adnav::diagonalCovariance(accel_variance, accel_variance, accel_variance);
	for (size_t k = 0; k < sds.size(); ++k) {
		const auto accel_block = linearBlock(accels[k]);
		for (size_t i = 0; i < 9; ++i) {
			EXPECT_EQ(imus[k].linear_acceleration_covariance[i], accel_block[i]) << k << " " << i;
			EXPECT_NEAR(imus[k].linear_acceleration_covariance[i], expected[i], 1e-12) << k << " " << i;
		}
		EXPECT_NEAR(imus[k].orientation_covariance[0], sds[k][0] * sds[k][0], 1e-12);
	}
}

TEST_F(DriverDecoding, SequenceWithoutPacket26PublishesNeitherImuNorAccel) {
	certus_->sendSystemState(5000, 0, true);
	certus_->sendRestOfSequence();
	settle();
	certus_->sendSystemState(5001, 0, true);
	certus_->sendRestOfSequence(std::nullopt);
	settle();
	certus_->sendSystemState(5002, 0, true);
	certus_->sendRestOfSequence();
	settle();

	EXPECT_EQ(imu_->stamps(), (std::vector<int32_t>{5000, 5002}));
	EXPECT_EQ(accel_->stamps(), (std::vector<int32_t>{5000, 5002}));
	EXPECT_EQ(driver_->imuDrops().missing_orientation_sd, 1u);
	EXPECT_EQ(driver_->imuDrops().late_orientation_sd, 0u);
	EXPECT_EQ(twist_->stamps(), (std::vector<int32_t>{5000, 5001, 5002}));
	EXPECT_EQ(imu_raw_->stamps(), (std::vector<int32_t>{5000, 5001, 5002}));
}

TEST_F(DriverDecoding, Packet26DecodedAfterTheSequenceGapIsNotMatched) {
	// packet_request gives packet 20 at 10 Hz, so a sequence ends 50 ms after its last packet.
	certus_->sendSystemState(6000, 0, true);
	std::this_thread::sleep_for(120ms);
	certus_->sendRestOfSequence();
	settle();
	certus_->sendSystemState(6001, 0, true);
	certus_->sendRestOfSequence();
	settle();

	EXPECT_EQ(imu_->stamps(), std::vector<int32_t>{6001});
	EXPECT_EQ(accel_->stamps(), std::vector<int32_t>{6001});
	EXPECT_EQ(driver_->imuDrops().late_orientation_sd, 1u);
	EXPECT_EQ(driver_->imuDrops().missing_orientation_sd, 0u);
}

TEST_F(DriverDecoding, PendingSampleKeepsTheAccelVarianceOfItsPacket20Schedule) {
	certus_->sendSystemState(7000, 0, true);
	// Slow every packet to 1 Hz between the sequence's packet 20 and its packet 26.
	ASSERT_TRUE(driver_->set_parameter(rclcpp::Parameter("packet_request",
		std::vector<int64_t>{20, 1000, 25, 1000, 26, 1000, 28, 1000, 36, 1000, 43, 1000})).successful);
	certus_->acknowledgePeriodsFor(300ms);
	certus_->sendRestOfSequence();
	settle();

	ASSERT_EQ(imu_->stamps(), std::vector<int32_t>{7000});
	const double ten_hz =
		adnav::sensorVariance(adnav::DEFAULT_ACCEL_NOISE_DENSITY, adnav::DEFAULT_ACCEL_BIAS_INSTABILITY, 10.0);
	const double one_hz =
		adnav::sensorVariance(adnav::DEFAULT_ACCEL_NOISE_DENSITY, adnav::DEFAULT_ACCEL_BIAS_INSTABILITY, 1.0);
	ASSERT_NE(ten_hz, one_hz);
	// Level, so z carries the accelerometer variance alone.
	EXPECT_EQ(imu_->last().linear_acceleration_covariance[8], ten_hz);
}

TEST_F(DriverDecoding, Packet20OutsideTheScheduleStartsNoImu) {
	ASSERT_TRUE(driver_->set_parameter(rclcpp::Parameter("packet_request",
		std::vector<int64_t>{25, 100, 26, 100, 28, 100, 36, 100, 43, 100})).successful);
	certus_->acknowledgePeriodsFor(300ms);
	certus_->sendSystemState(7100, 0, true);
	certus_->sendRestOfSequence();
	settle();

	EXPECT_TRUE(imu_->stamps().empty());
	EXPECT_TRUE(accel_->stamps().empty());
}

TEST_F(DriverDecoding, RejectsPacket26AtAnotherPeriodThanPacket20) {
	const auto result = driver_->set_parameter(
		rclcpp::Parameter("packet_request", std::vector<int64_t>{20, 100, 26, 20, 28, 100}));
	EXPECT_FALSE(result.successful);
	EXPECT_NE(result.reason.find("Packet 26"), std::string::npos);
}

namespace {

// Constructs a driver on a fake Certus UDP link with extra parameter overrides, passing on what the
// constructor throws. The caller shuts rclcpp down.
std::shared_ptr<adnav::Driver> constructOnUdp(const std::vector<std::string>& overrides,
	const int port = freeUdpPort()) {
	char log_dir[] = "/tmp/adnav_driver_test_XXXXXX";
	if (mkdtemp(log_dir) == nullptr) {
		throw std::runtime_error("mkdtemp failed");
	}
	std::vector<std::string> args{"test_driver_decoding", "--ros-args", "-p", "comm_select:=3",
		"-p", "port:=" + std::to_string(port), "-p", "log_path:=" + std::string(log_dir) + "/"};
	args.insert(args.end(), overrides.begin(), overrides.end());
	std::vector<const char*> argv;
	for (const auto& arg : args) {
		argv.push_back(arg.c_str());
	}
	rclcpp::init(static_cast<int>(argv.size()), argv.data());
	return std::make_shared<adnav::Driver>();
}

}  // namespace

TEST(DriverParameters, RejectsZeroAccelVariance) {
	try {
		constructOnUdp({"-p", "accel_noise_density:=0.0", "-p", "accel_bias_instability:=0.0"});
		ADD_FAILURE() << "constructed with a zero accelerometer variance";
	} catch (const std::invalid_argument& e) {
		EXPECT_NE(std::string(e.what()).find("accel_noise_density"), std::string::npos) << e.what();
	}
	rclcpp::shutdown();
}

TEST(DriverParameters, RejectsNegativeAccelNoiseDensity) {
	try {
		constructOnUdp({"-p", "accel_noise_density:=-0.001"});
		ADD_FAILURE() << "constructed with a negative accel_noise_density";
	} catch (const std::invalid_argument& e) {
		EXPECT_NE(std::string(e.what()).find("accel_noise_density"), std::string::npos) << e.what();
	}
	rclcpp::shutdown();
}

TEST(DriverParameters, AcceptsZeroAccelNoiseDensityWithABias) {
	auto driver = constructOnUdp({"-p", "accel_noise_density:=0.0", "-p", "accel_bias_instability:=0.001"});
	ASSERT_NE(driver, nullptr);
	driver.reset();
	rclcpp::shutdown();
}

TEST(DriverParameters, AcceptsAPositiveAccelNoiseDensity) {
	auto driver = constructOnUdp({"-p", "accel_noise_density:=0.001"});
	ASSERT_NE(driver, nullptr);
	EXPECT_EQ(driver->get_parameter("accel_noise_density").as_double(), 0.001);
	driver.reset();
	rclcpp::shutdown();
}

TEST(DriverConnection, SystemStatusReportsNoDataBeforeTheDevice) {
	const int port = freeUdpPort();
	auto driver = constructOnUdp({}, port);
	auto listener = std::make_shared<rclcpp::Node>("listener");
	StatusRecorder system_status(*listener, "/adnav_driver/system_status");

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(driver);
	executor.add_node(listener);
	std::thread spin([&] { executor.spin(); });
	std::this_thread::sleep_for(std::chrono::seconds(adnav::CONNECTION_TIMEOUT) * 2 + 1s);

	EXPECT_FALSE(driver->deviceReady());
	EXPECT_GE(system_status.count(), 1u);
	EXPECT_EQ(system_status.last().level, diagnostic_msgs::msg::DiagnosticStatus::STALE);

	executor.cancel();
	// Releases the reading callback, which blocks on the socket.
	FakeCertus certus(port);
	for (int i = 0; i < 10; ++i) {
		certus.send(packet_id_device_information, std::vector<uint8_t>(24, 0));
	}
	spin.join();
	driver.reset();
	listener.reset();
	rclcpp::shutdown();
}
