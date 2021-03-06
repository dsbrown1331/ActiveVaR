#include "ros/ros.h"
#include "active_var/FillArray.h"
#include <cstdlib>
#include <string>
#include <vector>
#include <sstream>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fill_array_client");
  if (argc != 3)
  {
    ROS_INFO("usage: fill_array_client size value");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<active_var::FillArray>("fill_array");
  active_var::FillArray srv;
  srv.request.size = atoll(argv[1]);
  srv.request.value = atoll(argv[2]);
  if (client.call(srv))
  {
        std::stringstream ss;

        for(int i=0; i<srv.request.size; i++)
        {
            ss << srv.response.policy[i] << " ";

        }
        ROS_INFO("Array: %s", ss.str().c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service fill_array");
    return 1;
  }

  return 0;
}
