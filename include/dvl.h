#ifndef __DVL_INTERFACE_H__
#define __DVL_INTERFACE_H__

#include <bits/stdc++.h>
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#include <limits>
#include <string>
#include <tacopie/tacopie>

#include <nortek_dvl/Dvl.h>
#include <nortek_dvl/DvlStatus.h>

namespace nortek_dvl {

class DvlInterface {
 public:
  explicit DvlInterface();
  ~DvlInterface();

 private:
  void dataCb(tacopie::tcp_client& client,
              const tacopie::tcp_client::read_result& res);
  void connect();
  void process(std::string message);
  bool validateChecksum(std::string& message);

  bool stringToDvlMessage(std::string& str, nortek_dvl::Dvl& dvl,
                          nortek_dvl::DvlStatus& status);
  void parseDvlStatus(unsigned long num, nortek_dvl::DvlStatus& status);
  template <class T>
  T hexStringToInt(std::string str);
  bool isVelocityValid(double vel);

  void readParams();

  std::string address_;
  uint16_t port_;
  std::string dvl_topic_;
  std::string dvl_status_topic_;
  tacopie::tcp_client client_;
  double dvl_rotation_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher dvl_pub_;
  ros::Publisher dvl_status_pub_;
};
}  // namespace nortek_dvl

#endif 
