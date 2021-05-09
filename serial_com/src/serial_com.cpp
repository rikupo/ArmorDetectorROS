#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <boost/asio/buffer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>
#include <string>

class serial_stream
{
public:
  serial_stream()
  {
    port_ = new boost::asio::serial_port(io_service_);
  }
  bool open()
  {
    device_ = "/dev/ttyUSB0";
    const int baund_rate = 115200;
    // open the serial port
    try
    {
      port_->open(device_);
      port_->set_option(boost::asio::serial_port_base::baud_rate(baund_rate));
      port_->set_option(boost::asio::serial_port_base::character_size(8));
      port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
      port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
      port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    }
    catch (const std::exception &error)
    {
      ROS_ERROR("Serial Fail: cound not open %s", device_.c_str());
      return false;
    }
    connected_ = true;
    ROS_INFO("Serial Open %s", device_.c_str());
    return true;
  }
  void close()
  {
    connected_ = false;
    port_->close();
  };
  void write(const uint8_t *packet)
  {
    if (port_->write_some(boost::asio::buffer(packet, sizeof(packet))) < 0)
    {
      close();
    }
  }
  void read_callback(const boost::system::error_code &e, std::size_t size)
  {
    std::cout.write(buf_.data(), size);
    port_->async_read_some(boost::asio::buffer(buf_), boost::bind(&serial_stream::read_callback, this,
                                                                  boost::placeholders::_1, boost::placeholders::_2));
  }
  void startRead()
  {
    if (isConnected())
    {
      ROS_ERROR("serial Fail: port is not open");
    }
    boost::thread thr_io(boost::bind(&boost::asio::io_service::run, &io_service_));
    port_->async_read_some(boost::asio::buffer(buf_), boost::bind(&serial_stream::read_callback, this,
                                                                  boost::placeholders::_1, boost::placeholders::_2));
  }
  bool isConnected()
  {
    return connected_;
  }
  bool getStatus()
  {
    if (read_success_ > 0)
    {
      read_success_ = 0;
      return true;
    }
    else
    {
      return false;
    }
  }
  std::string getDeviceName()
  {
    return device_;
  }

private:
  std::string device_;
  bool connected_ = false;
  int read_success_ = 0;
  boost::asio::io_service io_service_;
  boost::asio::serial_port *port_;
  boost::array<char, 64> buf_;
};

diagnostic_updater::Updater *p_updater;
ros::Publisher serial_pub;
serial_stream ss0;
union Position {
  struct
  {
    float x;
    float y;
  };
  uint8_t bin[sizeof(float) * 2];
};
void serial_callback(const std_msgs::Float32MultiArray &target_data)
{
  uint8_t packet[10];
  // header
  packet[0] = 255;
  packet[1] = 254;
  Position pos;
  //受信したx,y座標をunion型のPositon変数に格納
  pos.x = target_data.data[0];
  pos.y = target_data.data[1];
  for (int i = 2; i < 10; i++)
  {
    //packet[i] = pos.bin[i - 2];
    packet[i] = 253;
  }
  ss0.write(packet);
  ROS_INFO("send_data : %f ,%f", target_data.data[0], target_data.data[1]);
}
bool first_time = true;
bool last_connected = false;
//再接続
void timer_callback(const ros::TimerEvent &)
{
  if (!ss0.isConnected())
  {
    ss0.open();
    if (ss0.isConnected())
      ROS_INFO("Serial Open %s", ss0.getDeviceName().c_str());
    else
    {
      if (first_time)
      {
        ROS_ERROR("Serial Fail: cound not open %s", ss0.getDeviceName().c_str());
      }
      else if (last_connected)
      {
        ROS_ERROR("Serial Fail: Connection is broken %s", ss0.getDeviceName().c_str());
      }
    }
  }
  first_time = false;
  last_connected = ss0.isConnected();
  p_updater->update();
  ros::spinOnce();
}

void diagnostic0(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  bool serial_c = ss0.isConnected();
  bool serial_s = ss0.getStatus();
  if (serial_c && serial_s)
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Active.");
  else if (serial_c && !serial_s)
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "No Recieve.");
  else
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "No Connection.");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "serial_com");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  // param
  // pn.getParam("device_name", device_);
  // ss0.set_name(device_name);
  ss0.open();
  // Publisher
  serial_pub = n.advertise<std_msgs::String>("Serial_in", 10);
  // timer
  ros::Timer timer = n.createTimer(ros::Duration(0.001), timer_callback);
  // Diagnostic
  diagnostic_updater::Updater updater;
  p_updater = &updater;
  updater.setHardwareID("SerialPort");
  updater.add("Connect", diagnostic0);
  // Subscriber
  // ros::Subscriber serial_sub = n.subscribe("Serial_out", 1, serial_callback);
  ros::Subscriber typeA_sub = n.subscribe("targets", 1, serial_callback);

  ros::spin();
  ss0.close();
  return 0;
}