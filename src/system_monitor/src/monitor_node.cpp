#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

struct CpuStats
{
  unsigned long long user{0};
  unsigned long long nice{0};
  unsigned long long system{0};
  unsigned long long idle{0};
  unsigned long long iowait{0};
  unsigned long long irq{0};
  unsigned long long softirq{0};
  unsigned long long steal{0};
};

class SystemMonitor : public rclcpp::Node
{
public:
  SystemMonitor()
  : Node("system_monitor"), has_prev_cpu_(false), prev_total_(0), prev_idle_(0)
  {
    cpu_publisher_ = this->create_publisher<std_msgs::msg::Float64>("system/cpu", 10);
    ram_publisher_ = this->create_publisher<std_msgs::msg::Float64>("system/ram", 10);

    read_cpu_stats(prev_stats_);
    has_prev_cpu_ = true;

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&SystemMonitor::publish_stats, this));

    RCLCPP_INFO(this->get_logger(), "SystemMonitor node started");
  }

private:
  static bool read_cpu_stats(CpuStats & stats)
  {
    std::ifstream file("/proc/stat");
    if (!file.is_open()) {
      return false;
    }

    std::string cpu_label;
    file >> cpu_label >> stats.user >> stats.nice >> stats.system >> stats.idle >>
      stats.iowait >> stats.irq >> stats.softirq >> stats.steal;

    return cpu_label == "cpu";
  }

  static unsigned long long total_cpu(const CpuStats & stats)
  {
    return stats.user + stats.nice + stats.system + stats.idle +
           stats.iowait + stats.irq + stats.softirq + stats.steal;
  }

  static unsigned long long idle_cpu(const CpuStats & stats)
  {
    return stats.idle + stats.iowait;
  }

  double get_cpu_percent()
  {
    CpuStats current;
    if (!read_cpu_stats(current) || !has_prev_cpu_) {
      return 0.0;
    }

    const unsigned long long total = total_cpu(current);
    const unsigned long long idle = idle_cpu(current);
    const unsigned long long total_delta = total - prev_total_;
    const unsigned long long idle_delta = idle - prev_idle_;

    prev_total_ = total;
    prev_idle_ = idle;
    prev_stats_ = current;

    if (total_delta == 0) {
      return 0.0;
    }

    return 100.0 * static_cast<double>(total_delta - idle_delta) /
           static_cast<double>(total_delta);
  }

  static double get_ram_percent()
  {
    std::ifstream file("/proc/meminfo");
    if (!file.is_open()) {
      return 0.0;
    }

    std::string key;
    unsigned long long value = 0;
    std::string unit;
    unsigned long long mem_total = 0;
    unsigned long long mem_available = 0;

    while (file >> key >> value >> unit) {
      if (key == "MemTotal:") {
        mem_total = value;
      } else if (key == "MemAvailable:") {
        mem_available = value;
      }
    }

    if (mem_total == 0) {
      return 0.0;
    }

    const unsigned long long mem_used = mem_total - mem_available;
    return 100.0 * static_cast<double>(mem_used) / static_cast<double>(mem_total);
  }

  void publish_stats()
  {
    const double cpu_usage = get_cpu_percent();
    const double ram_usage = get_ram_percent();

    std_msgs::msg::Float64 cpu_msg;
    cpu_msg.data = cpu_usage;
    cpu_publisher_->publish(cpu_msg);

    std_msgs::msg::Float64 ram_msg;
    ram_msg.data = ram_usage;
    ram_publisher_->publish(ram_msg);

    RCLCPP_INFO(
      this->get_logger(), "CPU: %.2f%%, RAM: %.2f%%", cpu_usage, ram_usage);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cpu_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ram_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool has_prev_cpu_;
  CpuStats prev_stats_;
  unsigned long long prev_total_;
  unsigned long long prev_idle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemMonitor>());
  rclcpp::shutdown();
  return 0;
}
