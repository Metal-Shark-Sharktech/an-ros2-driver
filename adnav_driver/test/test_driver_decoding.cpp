#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <mutex>
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

	void sendFloats(uint8_t id, const std::vector<float>& values) {
		std::vector<uint8_t> data(values.size() * sizeof(float));
		std::memcpy(data.data(), values.data(), data.size());
		send(id, data);
	}

	void sendSystemState(uint32_t unix_seconds, uint32_t microseconds, bool utc_valid) {
		std::vector<uint8_t> data(100, 0);
		const uint16_t filter_status = utc_valid ? (1u << 3) : 0u;  // bit 3: UTC time initialised
		std::memcpy(&data[2], &filter_status, sizeof(filter_status));
		std::memcpy(&data[4], &unix_seconds, sizeof(unix_seconds));
		std::memcpy(&data[8], &microseconds, sizeof(microseconds));
		send(packet_id_system_state, data);
	}

	// Packets 25, 26, 28, 36 and 43 of one output sequence, in ID order.
	void sendRestOfSequence() {
		sendFloats(packet_id_velocity_standard_deviation, {0.1f, 0.2f, 0.3f});
		sendFloats(packet_id_euler_orientation_standard_deviation, {0.01f, 0.02f, 0.03f});
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
			last_ = msg;
		});
	}

	std::vector<int32_t> stamps() {
		std::lock_guard<std::mutex> lock(mutex_);
		return stamps_;
	}

	Msg last() {
		std::lock_guard<std::mutex> lock(mutex_);
		return last_;
	}

 private:
	typename rclcpp::Subscription<Msg>::SharedPtr sub_;
	std::mutex mutex_;
	std::vector<int32_t> stamps_;
	Msg last_;
};

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
		std::atomic<bool> constructed{false};
		std::thread construct([&] {
			driver_ = std::make_shared<adnav::Driver>();
			constructed = true;
		});
		// The constructor waits for a device information packet.
		for (int i = 0; i < 500 && !constructed; ++i) {
			certus_->send(packet_id_device_information, std::vector<uint8_t>(24, 0));
			std::this_thread::sleep_for(10ms);
		}
		construct.join();

		listener_ = std::make_shared<rclcpp::Node>("listener");
		imu_ = std::make_unique<Recorder<sensor_msgs::msg::Imu>>(*listener_, "/adnav_driver/imu");
		imu_raw_ = std::make_unique<Recorder<sensor_msgs::msg::Imu>>(*listener_, "/adnav_driver/imu_raw");
		accel_ = std::make_unique<Recorder<geometry_msgs::msg::AccelWithCovarianceStamped>>(
			*listener_, "/adnav_driver/accel");
		twist_ = std::make_unique<Recorder<geometry_msgs::msg::TwistWithCovarianceStamped>>(
			*listener_, "/adnav_driver/twist_body");

		executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
		executor_->add_node(driver_);
		executor_->add_node(listener_);
		spin_ = std::thread([this] { executor_->spin(); });
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
	std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
	std::thread spin_;
};

}  // namespace

TEST_F(DriverDecoding, SequenceSharesPacket20TimeAndPublishesOnce) {
	certus_->sendSystemState(1000, 500000, true);
	certus_->sendRestOfSequence();
	settle();
	certus_->sendSystemState(1001, 500000, true);
	certus_->sendRestOfSequence();
	settle();

	// The first imu waits for an orientation covariance, which arrives after it in its sequence.
	EXPECT_EQ(imu_->stamps(), std::vector<int32_t>{1001});
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
