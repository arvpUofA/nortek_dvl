#include "dvl.h"

using namespace nortek_dvl;

DvlInterface::DvlInterface()
    : address_("arvp-dvl"),
      port_(9004),
      dvl_status_topic_("status"),
      dvl_topic_("/dvl"),
      nh_(),
      private_nh_("~") {
  readParams();
  connect();

  dvl_pub_ = private_nh_.advertise<nortek_dvl::Dvl>(dvl_topic_, 10);
  dvl_status_pub_ =
      private_nh_.advertise<nortek_dvl::DvlStatus>(dvl_status_topic_, 10);
}

DvlInterface::~DvlInterface() { client_.disconnect(); }

void DvlInterface::dataCb(tacopie::tcp_client &client,
                 const tacopie::tcp_client::read_result &res) {
  if (res.success) {
    process(std::string(res.buffer.begin(), res.buffer.end()));

    client_.async_read({1024, std::bind(&DvlInterface::dataCb, this, std::ref(client_),
                                        std::placeholders::_1)});
  }
}

void DvlInterface::connect() {
  try {
    client_.connect(address_, port_, 500);
    client_.async_read({1024, std::bind(&DvlInterface::dataCb, this, std::ref(client_),
                                        std::placeholders::_1)});
  } catch (tacopie::tacopie_error &e) {
    throw std::runtime_error("Unable to connect to DVL on address " + address_ +
                             ":" + std::to_string(port_));
  }
}

bool DvlInterface::validateChecksum(std::string &message) {
  std::vector<std::string> results;
  boost::algorithm::split(results, message, boost::algorithm::is_any_of("*"));

  if (results.size() != 2) {
    return false;
  } else {
    // if checksum correct
    message = results[0];
    auto checksum = hexStringToInt<unsigned int>(results[1]);
    // TODO: compare checksum
    return true;
  }
}

void DvlInterface::process(std::string message) {
  nortek_dvl::Dvl dvl;
  nortek_dvl::DvlStatus status;
  boost::algorithm::trim(message);
  if (message.compare(0, 6, "Nortek") == 0) {
    ROS_INFO("Connected to DVL");
  } else {
    if (validateChecksum(message)) {
      ROS_DEBUG("%s", message.c_str());
      if (stringToDvlMessage(message, dvl, status)) {
        if (dvl_pub_.getNumSubscribers() > 0) dvl_pub_.publish(dvl);
        if (dvl_status_pub_.getNumSubscribers() > 0)
          dvl_status_pub_.publish(status);
      }
    } else {
      ROS_WARN("Invalid message from DVL");
    }
  }
}

template <class T>
T DvlInterface::hexStringToInt(std::string str) {
  T x;
  std::stringstream ss;
  ss << std::hex << str;
  ss >> x;
  return x;
}

bool DvlInterface::stringToDvlMessage(std::string &str, nortek_dvl::Dvl &dvl,
                             nortek_dvl::DvlStatus &status) {
  std::vector<std::string> results;
  boost::algorithm::split(results, str, boost::algorithm::is_any_of(",="));

  if (results.size() == 17) {
    dvl.header.stamp = ros::Time::now();
    dvl.time = std::stod(results[1]);
    dvl.dt1 = std::stof(results[2]);
    dvl.dt2 = std::stof(results[3]);
    dvl.velocity.x = std::stof(results[4]);
    dvl.velocity.y = std::stof(results[5]);
    dvl.velocity.z = std::stof(results[6]);
    if (isVelocityValid(dvl.velocity.x) and isVelocityValid(dvl.velocity.y) and
        isVelocityValid(dvl.velocity.z)) {
      if (dvl_rotation_ != 0) {
        // apply 2d rotation
        double temp_x = cos(dvl_rotation_) * dvl.velocity.x -
                        sin(dvl_rotation_) * dvl.velocity.y;
        double temp_y = sin(dvl_rotation_) * dvl.velocity.x +
                        cos(dvl_rotation_) * dvl.velocity.y;
        dvl.velocity.x = temp_x;
        dvl.velocity.y = temp_y;
      }
    } else {
      dvl.velocity.x = std::numeric_limits<double>::quiet_NaN();
      dvl.velocity.y = std::numeric_limits<double>::quiet_NaN();
      dvl.velocity.z = std::numeric_limits<double>::quiet_NaN();
    }
    dvl.figureOfMerit = std::stof(results[7]);
    dvl.beamDistance[0] = std::stof(results[8]);
    dvl.beamDistance[1] = std::stof(results[9]);
    dvl.beamDistance[2] = std::stof(results[10]);
    dvl.beamDistance[3] = std::stof(results[11]);
    dvl.batteryVoltage = std::stof(results[12]);
    dvl.speedSound = std::stof(results[13]);
    dvl.pressure = std::stof(results[14]);
    dvl.temp = std::stof(results[15]);

    parseDvlStatus(hexStringToInt<unsigned long>(results[16].substr(2)),
                   status);
    return true;
  }

  return false;
}

void DvlInterface::parseDvlStatus(unsigned long num, nortek_dvl::DvlStatus &status) {
  std::bitset<32> bset(num);

  status.header.stamp = ros::Time::now();

  status.b1_vel_valid = bset[0];
  status.b2_vel_valid = bset[1];
  status.b3_vel_valid = bset[2];
  status.b4_vel_valid = bset[3];
  status.b1_dist_valid = bset[4];
  status.b2_dist_valid = bset[5];
  status.b3_dist_valid = bset[6];
  status.b4_dist_valid = bset[7];
  status.b1_fom_valid = bset[8];
  status.b2_fom_valid = bset[9];
  status.b3_fom_valid = bset[10];
  status.b4_fom_valid = bset[11];
  status.x_vel_valid = bset[12];
  status.y_vel_valid = bset[13];
  status.z1_vel_valid = bset[14];
  status.z2_vel_valid = bset[15];
  status.x_fom_valid = bset[16];
  status.y_fom_valid = bset[17];
  status.z1_fom_valid = bset[18];
  status.z2_fom_valid = bset[19];

  if (bset[20]) {
    status.proc_cap = 3;
  } else if (bset[21]) {
    status.proc_cap = 6;
  } else if (bset[22]) {
    status.proc_cap = 12;
  }

  int wakeupstate = bset[28] << 3 | bset[29] << 2 | bset[30] << 1 | bset[31];

  if (wakeupstate == 0b0010) {
    status.wakeup_state = "break";
  } else if (wakeupstate == 0b0011) {
    status.wakeup_state = "RTC Alarm";
  } else if (wakeupstate == 0b0000) {
    status.wakeup_state = "bad power";
  } else if (wakeupstate == 0b0001) {
    status.wakeup_state = "power applied";
  }
}

void DvlInterface::readParams() {
  int port;
  private_nh_.getParam("address", address_);
  private_nh_.getParam("port", port);
  port_ = static_cast<uint16_t>(port);
  private_nh_.getParam("dvl_topic", dvl_topic_);
  private_nh_.getParam("status_topic", dvl_status_topic_);
  private_nh_.getParam("dvl_rotation", dvl_rotation_);

  std::cout << "DVL PARAMS" << std::endl;
  std::cout << "-----------------" << std::endl;
  std::cout << "address: " << address_ << std::endl;
  std::cout << "port: " << port_ << std::endl;
  std::cout << "rotation: " << dvl_rotation_ << std::endl;
  std::cout << "status_topic: " << ros::this_node::getName() << "/"
            << dvl_status_topic_ << std::endl;
  std::cout << "dvl data topic: " << ros::this_node::getName() << "/" 
            << dvl_topic_ << std::endl;
  std::cout << "-----------------\n" << std::endl;
}

bool DvlInterface::isVelocityValid(double vel) {
  return vel > -32;  // -32.786 is invalid velocity
}
