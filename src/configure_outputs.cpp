#include <ros/ros.h>
#include "xdainterface.h"

Journaller * gJournal = 0;

int main(int argc, char * argv[])
{
  if (argc != 5){
    printf("Usage: rosrun xsens_mti_driver sampling_frequency enable_acceleration(0-1) "
           "enable_angular_velocity(0-1) enable_orientation(0-1)\n\n");
    throw std::runtime_error("Four integers are needed");
  }

  ros::init(argc, argv, "xsens_driver");
  XdaInterface interface;
  interface.connectDevice();
  interface.configureOutput(std::atoi(argv[1]), std::atoi(argv[2]), std::atoi(argv[3]), std::atoi(argv[4]));

  return EXIT_SUCCESS;
}
