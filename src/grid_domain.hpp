#ifndef grid_domains_h
#define grid_domains_h

using namespace std;

void initFeaturesToyFeatureDomain5x5(double** stateFeatures);

//TODO make sure to delete this!!!
//Set's up stateFeatures for simple grid world (daniel)
double** initFeaturesToyFeatureDomain5x5(const int numStates, const int numFeatures)
{
     if(numStates != 25 || numFeatures != 5) 
     {
        cout << "[ERROR] This domain only works for 5x5 with 5 features!" << endl;
        return nullptr;
     }
    double** stateFeatures;
    stateFeatures = new double*[numStates];
    for(int i=0; i<numStates; i++)
        stateFeatures[i] = new double[numFeatures];
        
    double whiteFeature[]  = {1,0,0,0,0};
    double redFeature[]    = {0,1,0,0,0};
    double blueFeature[]   = {0,0,1,0,0};
    double yellowFeature[] = {0,0,0,1,0};
    double greenFeature[]  = {0,0,0,0,1};
    
    double YBFeature[]    = {0,0,1,1,0};

    char features[] = {'w','w','w','w','w',
                       'w','y','w','w','w',
                       'w','Y','g','w','w',
                       'w','Y','Y','b','w',
                       'w','w','w','w','w',
                       };
                       
    for(int i=0; i < numStates; i++)
    {
        switch(features[i])
        {
            case 'w':
                std::copy(whiteFeature, whiteFeature+numFeatures, stateFeatures[i]);
                break;
            case 'r':
                std::copy(redFeature, redFeature+numFeatures, stateFeatures[i]);
                break;
            case 'b':
                std::copy(blueFeature, blueFeature+numFeatures, stateFeatures[i]);
                break;
            case 'y':
                std::copy(yellowFeature, yellowFeature+numFeatures, stateFeatures[i]);
                break;
            case 'g':
                std::copy(greenFeature, greenFeature+numFeatures, stateFeatures[i]);
                break;
            case 'Y':
                std::copy(YBFeature, YBFeature+numFeatures, stateFeatures[i]);
                break;
        }
    
    }
    return stateFeatures;
}

double** initFeaturesSimpleFeatureDomain5x5(const int numStates, const int numFeatures)
{
     if(numStates != 25 ) 
     {
        cout << "[ERROR] This domain only works for 5x5 with 5 features!" << endl;
        return nullptr;
     }
    double** stateFeatures;
    stateFeatures = new double*[numStates];
    for(int i=0; i<numStates; i++)
        stateFeatures[i] = new double[numFeatures];
        
    double whiteFeature[]  = {1,0};
    double redFeature[]    = {0,1};
    
    char features[] = {'w','w','w','w','w',
                       'w','w','w','r','w',
                       'w','w','w','r','w',
                       'w','r','r','r','w',
                       'w','w','w','w','w'
                       };
                       
    for(int i=0; i < numStates; i++)
    {
        switch(features[i])
        {
            case 'w':
                std::copy(whiteFeature, whiteFeature+numFeatures, stateFeatures[i]);
                break;
            case 'r':
                std::copy(redFeature, redFeature+numFeatures, stateFeatures[i]);
                break;
        }
    
    }
    return stateFeatures;
}

double** laptopCupWorld(const int numStates, const int numFeatures)
{
     if(numStates != 25 || numFeatures != 3) 
     {
        cout << "[ERROR] This domain only works for 5x5 with 3 features!" << endl;
        return nullptr;
     }
    double** stateFeatures;
    stateFeatures = new double*[numStates];
    for(int i=0; i<numStates; i++)
        stateFeatures[i] = new double[numFeatures];
        
    double tableFeature[]  = {1,0,0};
    double laptopFeature[]    = {0,1,0};
    double cupHolderFeature[] = {0,0,1};
    
    char features[] = {'w','w','w','w','w',
                       'w','w','w','c','w',
                       'w','l','l','w','w',
                       'w','l','l','w','w',
                       'w','w','w','w','w'
                       };
                       
    for(int i=0; i < numStates; i++)
    {
        switch(features[i])
        {
            case 'w':
                std::copy(tableFeature, tableFeature+numFeatures, stateFeatures[i]);
                break;
            case 'l':
                std::copy(laptopFeature, laptopFeature+numFeatures, stateFeatures[i]);
                break;
            case 'c':
                std::copy(cupHolderFeature, cupHolderFeature+numFeatures, stateFeatures[i]);
        }
    
    }
    return stateFeatures;
}

double** initPuddleDomain(const int numStates, const int numFeatures)
{
     if(numStates != 64 || numFeatures != 2) 
     {
        cout << "[ERROR] This domain only works for 8x8 with 2 features!" << endl;
        return nullptr;
     }
    double** stateFeatures;
    stateFeatures = new double*[numStates];
    for(int i=0; i<numStates; i++)
        stateFeatures[i] = new double[numFeatures];
        
    double regularFeature[]      = {1,0}; //  N
    double goalFeature[]        = {0,1};  //  G
    char features[64];
   
    for(unsigned int i = 0; i < 64; i++) features[i] = 'N';
    features[rand()%64] = 'G'; 
                       
    for(int i=0; i < numStates; i++)
    {
        switch(features[i])
        {
            case 'G':
                std::copy( goalFeature,  goalFeature+numFeatures, stateFeatures[i]);
                break;
            default:
                std::copy(regularFeature, regularFeature+numFeatures, stateFeatures[i]);
                break;
        }
    
    }
    return stateFeatures;
}


double** initRandomFeaturesRandomDomain(const int numStates, const int numFeatures)
{
    double** stateFeatures;
    stateFeatures = new double*[numStates];
    for(int i=0; i<numStates; i++)
    {
        stateFeatures[i] = new double[numFeatures];
        double feature[numFeatures];
        for (int feat=0; feat < numFeatures; feat++) feature[feat] = (double)(rand() % 2);
        std::copy(feature, feature+numFeatures, stateFeatures[i]);
    }
    return stateFeatures;
}


//No terminal state
double** randomNGridNavWorldXFeatures(int N, int X)
{
    int numStates = N;
    int numFeatures = X;
    double** stateFeatures;
    stateFeatures = new double*[numStates];
    for(int i=0; i<numStates; i++)
    {
        stateFeatures[i] = new double[numFeatures];
        for(int f = 0; f < numFeatures; f++)
            stateFeatures[i][f] = 0.0;
    }
        
                         
    for(int i=0; i < numStates; i++)
    {
        
        //randomly select one of the colors
        int f = rand() % numFeatures;
        stateFeatures[i][f] = 1.0;
    
    }
   
    return stateFeatures;


}



#endif

