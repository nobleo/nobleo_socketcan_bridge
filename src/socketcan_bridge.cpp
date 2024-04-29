#include <fmt/core.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <can_msgs/msg/frame.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <thread>

std::string to_string(const can_msgs::msg::Frame & msg)
{
  auto result = fmt::format("{:0>3X} [{}]", msg.id, msg.dlc);
  for (auto i = 0; i < msg.dlc; ++i) {
    result += fmt::format(" {:0>2X}", msg.data[i]);
  }
  return result;
}

class SocketCanBridge
{
public:
  using CanCallback = std::function<void(const can_msgs::msg::Frame &)>;

  SocketCanBridge(
    rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock, const std::string & interface,
    const CanCallback & receive_callback)
  : logger_(logger), clock_(clock), receive_callback_(receive_callback)
  {
    using std::placeholders::_1;

    if ((socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
      throw std::system_error(errno, std::generic_category());
    }

    timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);

    ifreq ifr;
    strncpy(ifr.ifr_name, interface.c_str(), sizeof(ifr.ifr_name));
    if (ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
      throw std::system_error(errno, std::generic_category());
    };

    sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_, (sockaddr *)&addr, sizeof(addr)) < 0) {
      throw std::system_error(errno, std::generic_category());
    }

    read_thread_ = std::jthread{std::bind(&SocketCanBridge::read_loop, this, _1)};
  }

  ~SocketCanBridge() { close(); }

  void write(const can_msgs::msg::Frame & msg)
  {
    can_frame frame;
    frame.can_id = msg.id;
    frame.len = msg.dlc;
    std::copy(msg.data.begin(), msg.data.end(), frame.data);
    RCLCPP_DEBUG(logger_, "Sending %s", to_string(msg).c_str());
    auto n = ::write(socket_, &frame, sizeof(frame));
    if (n < 0) {
      throw std::system_error(errno, std::generic_category());
    }
    RCLCPP_DEBUG(logger_, "Wrote %zd bytes to the socket", n);
  }

  void close()
  {
    RCLCPP_INFO(logger_, "Stopping the read thread");
    read_thread_.request_stop();

    RCLCPP_INFO(logger_, "Waiting for the read thread to stop");
    read_thread_.join();

    RCLCPP_INFO(logger_, "Closing the socket");
    if (::close(socket_) < 0) {
      throw std::system_error(errno, std::generic_category());
    }
  }

private:
  void read_loop(std::stop_token stoken)
  {
    RCLCPP_INFO(logger_, "Read loop started");
    while (!stoken.stop_requested()) {
      can_frame frame;
      auto nbytes = read(socket_, &frame, sizeof(can_frame));
      if (nbytes < 0) {
        if (errno == EAGAIN) continue;
        throw std::system_error(errno, std::generic_category());
      }
      if (nbytes != sizeof(frame)) {
        throw std::runtime_error("Partial CAN frame received");
      }

      can_msgs::msg::Frame msg;
      msg.header.stamp = clock_->now();
      msg.id = frame.can_id;
      msg.dlc = frame.len;
      std::copy_n(frame.data, sizeof(frame.data), msg.data.begin());

      RCLCPP_DEBUG(logger_, "Received %s", to_string(msg).c_str());
      receive_callback_(msg);
    }
    RCLCPP_INFO(logger_, "Read loop stopped");
  }

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  int socket_;
  CanCallback receive_callback_;
  std::jthread read_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("socketcan_bridge");

  auto can_pub = node->create_publisher<can_msgs::msg::Frame>("~/rx", 100);
  SocketCanBridge::CanCallback cb = [&can_pub](const can_msgs::msg::Frame & msg) {
    can_pub->publish(msg);
  };

  SocketCanBridge bridge{node->get_logger(), node->get_clock(), "vcan0", cb};
  auto can_sub = node->create_subscription<can_msgs::msg::Frame>(
    "~/tx", 100, [&bridge](can_msgs::msg::Frame::ConstSharedPtr msg) { bridge.write(*msg); });

  rclcpp::experimental::executors::EventsExecutor executor;
  executor.add_node(node);
  executor.spin();

  return EXIT_SUCCESS;
}
