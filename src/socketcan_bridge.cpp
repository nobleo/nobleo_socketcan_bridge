#include <linux/can/raw.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <thread>

class SocketCanBridge
{
public:
  SocketCanBridge(rclcpp::Logger logger, const std::string & interface) : logger_(logger)
  {
    using std::placeholders::_1;

    if ((socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
      throw std::system_error(errno, std::generic_category());
    }

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);

    struct ifreq ifr;
    strcpy(ifr.ifr_name, interface.c_str());
    ioctl(socket_, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      throw std::system_error(errno, std::generic_category());
    }

    read_thread_ = std::jthread{std::bind(&SocketCanBridge::read_loop, this, _1)};
  }

  ~SocketCanBridge() { close(); }

  void write() {}

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
      struct can_frame frame;
      auto nbytes = read(socket_, &frame, sizeof(struct can_frame));
      if (nbytes < 0) {
        if (errno == EAGAIN) continue;
        throw std::system_error(errno, std::generic_category());
      }

      printf("0x%03X [%d] ", frame.can_id, frame.can_dlc);

      for (auto i = 0; i < frame.can_dlc; i++) printf("%02X ", frame.data[i]);

      printf("\r\n");
    }
    RCLCPP_INFO(logger_, "Read loop stopped");
  }

  rclcpp::Logger logger_;
  int socket_;
  std::jthread read_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("socketcan_bridge");
  SocketCanBridge bridge{node->get_logger(), "vcan0"};

  rclcpp::experimental::executors::EventsExecutor executor;
  executor.add_node(node);
  executor.spin();

  return EXIT_SUCCESS;
}
