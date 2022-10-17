#include <ros/ros.h>
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsfilterprofilearray.h>

Journaller * gJournal = 0;

class XdaInterface
{
public:
  XdaInterface()
  {
    ROS_INFO("Creating XsControl object...");
    m_control = XsControl::construct();
    assert(m_control != 0);
  }
  ~XdaInterface()
  {
    ROS_INFO("Cleaning up ...");
    m_control->closePort(m_device);
    m_control->destruct();
  }

  bool connectDevice()
  {
    // Read baudrate parameter if set
    XsBaudRate baudrate = XBR_Invalid;
    if (ros::param::has("~baudrate")) {
      int baudrateParam = 0;
      ros::param::get("~baudrate", baudrateParam);
      ROS_INFO("Found baudrate parameter: %d", baudrateParam);
      baudrate = XsBaud::numericToRate(baudrateParam);
    }
    // Read device ID parameter
    bool checkDeviceID = false;
    std::string deviceId;
    if (ros::param::has("~device_id")) {
      ros::param::get("~device_id", deviceId);
      checkDeviceID = true;
      ROS_INFO("Found device ID parameter: %s.", deviceId.c_str());
    }
    // Read port parameter if set
    XsPortInfo mtPort;
    if (ros::param::has("~port")) {
      std::string portName;
      ros::param::get("~port", portName);
      ROS_INFO("Found port name parameter: %s", portName.c_str());
      mtPort = XsPortInfo(portName, baudrate);
      ROS_INFO("Scanning port %s ...", portName.c_str());
      if (!XsScanner::scanPort(mtPort, baudrate))
        throw std::runtime_error("No MTi device found. Verify port and baudrate.");
      if (checkDeviceID && mtPort.deviceId().toString().c_str() != deviceId)
        throw std::runtime_error("No MTi device found with matching device ID.");

    } else {
      ROS_INFO("Scanning for devices...");
      XsPortInfoArray portInfoArray = XsScanner::scanPorts(baudrate);

      for (auto const & portInfo : portInfoArray) {
        if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig()) {
          if (checkDeviceID) {
            if (portInfo.deviceId().toString().c_str() == deviceId) {
              mtPort = portInfo;
              break;
            }
          } else {
            mtPort = portInfo;
            break;
          }
        }
      }
    }

    if (mtPort.empty()) throw std::runtime_error("No MTi device found.");

    ROS_INFO(
      "Found a device with ID: %s @ port: %s, baudrate: %d",
      mtPort.deviceId().toString().toStdString().c_str(), mtPort.portName().toStdString().c_str(),
      XsBaud::rateToNumeric(mtPort.baudrate()));

    ROS_INFO("Opening port %s ...", mtPort.portName().toStdString().c_str());
    if (!m_control->openPort(mtPort)) throw std::runtime_error("Could not open port");

    m_device = m_control->device(mtPort.deviceId());
    assert(m_device != 0);

    ROS_INFO(
      "Device: %s, with ID: %s opened.", m_device->productCode().toStdString().c_str(),
      m_device->deviceId().toString().c_str());

    return true;
  }

  void listFilterProfiles()
  {
    assert(m_device != 0);

    auto profile = m_device->onboardFilterProfile();
    ROS_INFO(
      "onboardFilterProfile: '%s' label=%s kind=%s", profile.toString().c_str(), profile.label(),
      profile.kind());

    ROS_INFO("onboard filter profiles:");
    for (auto profile : m_device->availableOnboardFilterProfiles()) {
      ROS_INFO(
        "\t'%s' label=%s kind=%s", profile.toString().c_str(), profile.label(), profile.kind());
    }

    profile = m_device->xdaFilterProfile();
    ROS_INFO(
      "xdaFilterProfile: '%s' label=%s kind=%s", profile.toString().c_str(), profile.label(),
      profile.kind());

    ROS_INFO("xda filter profiles:");
    for (auto profile : m_device->availableXdaFilterProfiles()) {
      ROS_INFO(
        "\t'%s' label=%s kind=%s", profile.toString().c_str(), profile.label(), profile.kind());
    }
  }

private:
  XsControl * m_control = nullptr;
  XsDevice * m_device = nullptr;
};

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "xsens_driver");
  ros::NodeHandle node;

  XdaInterface interface;
  interface.connectDevice();
  interface.listFilterProfiles();

  return EXIT_SUCCESS;
}
