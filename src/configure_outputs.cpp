#include <ros/ros.h>
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>

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

  void configureOutput(int acceleration_hz, int angular_velocity_hz, int magnetic_field_hz)
  {
    assert(m_device != 0);

    ROS_INFO("Putting device into configuration mode...");
    if (!m_device->gotoConfig())
      throw std::runtime_error("Could not put device into configuration mode");

    ROS_INFO("Configuring the device...");
    XsOutputConfigurationArray configArray;

    if (m_device->deviceId().isImu())
    {
      configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
      configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));
      configArray.push_back(XsOutputConfiguration(XDI_Acceleration, acceleration_hz));
      configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, angular_velocity_hz));
      configArray.push_back(XsOutputConfiguration(XDI_MagneticField, magnetic_field_hz));
    }
    else
    {
      throw std::runtime_error("No IMU device while configuring. Aborting.");
    }

    if (!m_device->setOutputConfiguration(configArray))
      throw std::runtime_error("Could not configure MTi device. Aborting.");
  }

private:
  XsControl * m_control = nullptr;
  XsDevice * m_device = nullptr;
};

int main(int argc, char * argv[])
{
  if (argc != 3)
    std::runtime_error("Three integers are needed");

  ros::init(argc, argv, "xsens_driver");
  XdaInterface interface;
  interface.connectDevice();
  interface.configureOutput(std::atoi(argv[0]), std::atoi(argv[1]), std::atoi(argv[2]));

  return EXIT_SUCCESS;
}
