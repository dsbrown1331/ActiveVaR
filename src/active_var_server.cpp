#include "ros/ros.h"
#include "ros/assert.h"
#include "active_var/ActiveVaRQuery.h"
#include "active_var/StateAction.h"
#include <string>
#include <vector>
#include <cstdlib>
#include <sstream>
#include "mdp.hpp"
#include "feature_birl.hpp"
#include "confidence_bounds.hpp"

using namespace std;

pair<unsigned int, double> calculateVaRquery(FeatureBIRL* birl, vector<unsigned int> & initStates, vector<unsigned int> & eval_policy, double VaR_alpha)
{
    cout << "calculating VaR" << endl;
    unsigned int chain_length = birl->getChainLength();
    unsigned int num_states = birl->getGridHeight() * birl->getGridWidth();
    double eps = birl->getValueIterationStoppingThreshold();
    //Get V^* and V^\pi_eval for each state 

    vector<vector<double>> evds(initStates.size(), vector<double>(chain_length));
    
    for(unsigned int i=0; i<chain_length; i++)
    {
        //cout.precision(5);
        //get sampleMDP from chain
        FeatureGridMDP* sampleMDP = (*(birl->getRewardChain() + i));
        //((FeatureGridMDP*)sampleMDP)->displayFeatureWeights();
        //cout << "===================" << endl;
        //cout << "Reward " << i << endl;
        //sampleMDP->displayRewards();
        //cout << "--------" << endl;
        //cout << birl.calculateMaxEntPosterior((FeatureGridMDP*)sampleMDP) << endl;
        vector<unsigned int> sample_pi(sampleMDP->getNumStates());
        //cout << "sample opt policy" << endl;
        sampleMDP->getOptimalPolicy(sample_pi);
        //sampleMDP->displayPolicy(sample_pi);
        //cout << "Value" << endl;
        //sampleMDP->displayValues();
        vector<double> Vstar_vec =  getExpectedReturnVector(sampleMDP);
        //cout << "True Exp Val" << endl;
        //cout << Vstar << endl;
        //cout << "Eval Policy" << endl; 
        vector<double> Vhat_vec = evaluateExpectedReturnVector(eval_policy, sampleMDP, eps);
        //cout << "Vhat_vec" << endl;
        //cout << Vhat << endl;
        //save EVDiffs for each starting state for this hypothesis reward
        for(unsigned int j = 0; j < Vstar_vec.size(); j++)
        {
            //cout << j << endl;
            double EVDiff = Vstar_vec[j] - Vhat_vec[j];
            ROS_ASSERT_MSG(EVDiff > -0.01,"evd < 0 should never happen %f",EVDiff);
            //cout << "evddiff" << EVDiff << endl;
            evds[j][i] = EVDiff;
            //cout << "saved" << endl;
        }

    }    
        
    //output VaR data
    //ofstream outfile_var("data/active/var_" + filename);
    cout << "VaR:" << endl;
    unsigned int query_state_idx = 0;
    double max_VaR = 0;
    unsigned int init_state_count = 0;
    for(unsigned int s = 0; s < initStates.size(); s++)
    {
        std::sort(evds[s].begin(), evds[s].end());
        int VaR_index = (int) chain_length * VaR_alpha;
        double eval_VaR = evds[s][VaR_index];  
        if (eval_VaR > max_VaR)
        {
            max_VaR = eval_VaR;
            query_state_idx = s;
        }      
        
        cout << "state " << initStates[s] << " VaR = " << eval_VaR << endl;
        
    }
    //outfile_var.close();
    unsigned int query_state = initStates[query_state_idx];
    cout << "query state: " << query_state << endl;
    cout << "VaR of query state: " << max_VaR << endl;
    
    return make_pair(query_state, max_VaR);

}


bool active_query(active_var::ActiveVaRQuery::Request  &req,
         active_var::ActiveVaRQuery::Response &res)
{
    unsigned int width = req.width;
    unsigned int height = req.height;
    unsigned int numStates = width * height;
    vector<unsigned int> initStates = req.initial_states;
    vector<unsigned int> termStates = req.terminal_states;
    unsigned int numFeatures = req.num_features;
    double fWeights[numFeatures];
    vector<active_var::FloatVector> stateFeatures = req.state_features;
    ROS_ASSERT_MSG(stateFeatures.size() == numStates, "size of state features in service call is not same as width * height");
    //convert to double**
    double** sFeatures = new double*[numStates];
    for(int i=0; i<numStates; i++)
    {
        active_var::FloatVector fv = stateFeatures[i];
        vector<double> features = fv.FloatVector;
        ROS_ASSERT_MSG(features.size() == numFeatures,"size of individual state feature vector not same as numFeatures in service call");
        sFeatures[i] = new double[numFeatures];
        for(int f = 0; f < numFeatures; f++)
        {
            sFeatures[i][f] = features[f];
        }
    }

    bool stochastic = false;
    double gamma = req.discount;
    
    FeatureGridMDP fmdp(width, height, initStates, termStates, numFeatures, fWeights, sFeatures, stochastic, gamma);
//    vector<unsigned int> opt_policy (fmdp.getNumStates());
//    fmdp.valueIteration(0.0001);
//    fmdp.calculateQValues();
//    fmdp.getOptimalPolicy(opt_policy);
    
    
    cout << "==========" << endl;
    cout << "Initial MDP" << endl;
    cout << "==========" << endl;
    cout << "width = " << width << endl;
    cout << "height = " << height << endl;
    cout << "states = " << numStates << endl;
    cout << "initial states = ";
    for(unsigned int s : initStates) 
        cout << s << " ";
    cout << endl;
    cout << "terminal states = ";
    for(unsigned int s : termStates) 
        cout << s << " ";
    cout << endl;
    cout << "state features " << endl;
    for(unsigned int s = 0; s < numStates; s++)
    {
        for(int f = 0; f < numFeatures; f++)
            cout << sFeatures[s][f] << " ";
        cout << endl;    
    }
    cout << "discount = " << gamma << endl;
//    cout << "feature weights" << endl;
//    fmdp.displayFeatureWeights();
//    cout << "-- optimal policy --" << endl;
//    fmdp.displayPolicy(opt_policy);
        
    
    cout << "==========" << endl;    
    
    
    
    
    
  
        
    //run BIRL    
    double min_reward = -1;
    double max_reward = 1;
    unsigned int chain_length = 1000;
    double step = 0.05;
    int sample_flag = 4;                      //param for mcmc walk type
    int num_steps = 10;                       //tweaks per step in mcmc
    double value_iteration_threshold = 0.0001;
    bool mcmc_reject = true;   
    double conf = req.confidence;
    double var_alpha = req.alpha;
    double delta = req.delta;  //TODO: use this in computing VaR bound
    double stop_threshold = req.epsilon;  //TODO: use this in determining status
    
    vector<active_var::StateAction> demonstration = req.demonstration;

    cout << "BIRL Params" << endl;
    cout << "confidence = " << conf << endl;
    cout << "VaR alpha = " << var_alpha << endl;
    cout << "delta = " << delta << endl;
    cout << "epsilon = " << stop_threshold << endl;
    //extract demonstration    
    vector<pair<unsigned int,unsigned int> > demo;
    for(active_var::StateAction sa : demonstration)
    {
        demo.push_back(make_pair(sa.state, sa.action));
    }

    
    
    //FeatureBIRL birl(&fmdp, min_reward, max_reward, chain_length, step, conf, sample_flag,
    //  mcmc_reject_flag, num_steps); //mine
    FeatureBIRL birl(&fmdp, min_reward, max_reward, chain_length, step, conf, sample_flag, mcmc_reject, num_steps, value_iteration_threshold);
    birl.addPositiveDemos(demo);
    cout << "running birl" << endl;
    birl.displayDemos();
    birl.run();//eps
    FeatureGridMDP* mapMDP = birl.getMAPmdp();
    mapMDP->displayFeatureWeights();
    cout << "Recovered reward" << endl;
    mapMDP->displayRewards();
    cout << "Recovered policy" << endl;        
    vector<unsigned int> map_policy  (mapMDP->getNumStates());
    mapMDP->valueIteration(value_iteration_threshold);
    mapMDP->calculateQValues();
    mapMDP->getOptimalPolicy(map_policy);
    mapMDP->displayPolicy(map_policy);
    
    //get largest VaR state as query
    pair<unsigned int, double> query_var = calculateVaRquery(&birl, initStates, map_policy, var_alpha);
    unsigned int query_state = query_var.first;
    double max_VaR = query_var.second;


    //return policy, status, and query state
    res.policy = map_policy;
    if(max_VaR < stop_threshold)
    {
        res.status = "Done";
    }
    else
    {
        res.status = "Query";
    }
    res.query_state = query_state;
    
    ROS_INFO("request to solve MDP: size=%d", numStates);
    std::stringstream ss;
    for(int i=0; i<numStates; i++)
    {
        ss << map_policy[i] << " ";

    }
    ROS_INFO("sending back policy: \n%s", ss.str().c_str());
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "active_var_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("active_var", active_query);
  ROS_INFO("Ready to solve for active var query.");
  ros::spin();

  return 0;
}
