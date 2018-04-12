#include "ros/ros.h"
#include "active_var/ActiveVaRQuery.h"
#include "active_var/StateAction.h"
#include "grid_domain.hpp"
#include <cstdlib>
#include <string>
#include <vector>
#include <sstream>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "active_var_client");
  if (argc != 3)
  {
    ROS_INFO("usage: active_var_client width height");
    return 1;
  }

  unsigned int width = 5;
  unsigned int height = 5;
  unsigned int numFeatures = 3;
  unsigned int numStates = 25;
  double discount = 0.95;
  double birl_conf = 20;
  double var_alpha = 0.95;
  double var_delta = 0.05;
  double stopping_var = 0.1;

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<active_var::ActiveVaRQuery>("active_var");
  active_var::ActiveVaRQuery srv;
  srv.request.width = width;
  srv.request.height = height;
  srv.request.num_features = numFeatures;
  vector<unsigned int> init_states = {0,4,20,24};
  vector<unsigned int> term_states = {};
  srv.request.discount = discount;
  srv.request.confidence = birl_conf;
  srv.request.alpha = var_alpha;
  srv.request.delta = var_delta;
  srv.request.epsilon = stopping_var;
  srv.request.initial_states = init_states;
  srv.request.terminal_states = term_states;
  
  //generate state features for query mdp
  double** stateFeatures = laptopCupWorld(numStates, numFeatures);
  vector<active_var::FloatVector> sFeatures;
  for(int i=0; i<numStates; i++)
  {
    active_var::FloatVector fv;
    for(int f = 0; f < numFeatures; f++)
    {
        fv.FloatVector.push_back(stateFeatures[i][f]);
    }
    sFeatures.push_back(fv);
  }
  srv.request.state_features = sFeatures;
  
  //give demonstration
  vector<active_var::StateAction> demo;
  active_var::StateAction sa;
  sa.state = 0;
  sa.action = 3;
  demo.push_back(sa);
  sa.state = 1;
  sa.action = 3;
  demo.push_back(sa);
  sa.state = 2;
  sa.action = 3;
  demo.push_back(sa);
  sa.state = 3;
  sa.action = 1;
  demo.push_back(sa);
  sa.state = 20;
  sa.action = 3;
  demo.push_back(sa);
  sa.state = 21;
  sa.action = 3;
  demo.push_back(sa);
  sa.state = 22;
  sa.action = 3;
  demo.push_back(sa);
  sa.state = 23;
  sa.action = 0;
  demo.push_back(sa);
  
  srv.request.demonstration = demo;
  if (client.call(srv))
  {
        std::stringstream ss;

        for(int i=0; i<numStates; i++)
        {
            ss << srv.response.policy[i] << " ";

        }
        ROS_INFO("Policy: %s", ss.str().c_str());
        ROS_INFO("Status: %s", srv.response.status.c_str());
        ROS_INFO("Query State: %u", srv.response.query_state);
            
  }
  else
  {
    ROS_ERROR("Failed to call service active_var");
    return 1;
  }

  return 0;
}
