#include "thormang3_kinematics_dynamics/kinematics_dynamics.h"
#include "thormang3_kinematics_dynamics/link_data.h"
#include "generateSamples.h"
#include <iostream>
#include <cstdlib>
#include <map>
#include <vector>
#include <random>
#include <cstdlib>
#include <assert.h>
#include <string>
#include <fstream>
#include <cmath>

#define DELTA_LIMIT 0.5
#define MAX_DISTANCE 0.5

std::vector<double> getFKResult(std::map<int, double> joint_values, int end_effector_id){
    // Vector to be returned, filled with information about position and orientation of requested
    // ID
    std::vector<double> results;

    // Resets all joints to position 0.0
    for (int id = 1; id <= MAX_JOINT_ID; id++)
        robotis_->thormang3_link_data_[id]->joint_angle_ = 0.0;

    // Set joint values for passed IDs and angles
    for(std::map<int, double>::iterator it = joint_values.begin(); it != joint_values.end(); ++it){
        //std::cout << "ID: " << it->first << " Value: " << it->second << " ";
        robotis_->thormang3_link_data_[it->first]->joint_angle_ = it->second;
    }
    
    // Calculate FK
    robotis_->calcForwardKinematics(0);

    // Retrieve X, Y, Z information
    results.push_back(robotis_->thormang3_link_data_[end_effector_id]->position_.coeff(0, 0));
    results.push_back(robotis_->thormang3_link_data_[end_effector_id]->position_.coeff(1, 0));
    results.push_back(robotis_->thormang3_link_data_[end_effector_id]->position_.coeff(2, 0));

    // Retrieve orientation information
    
    Eigen::Vector3d rpy = robotis_framework::convertRotationToRPY(robotis_->thormang3_link_data_[end_effector_id]->orientation_);
    results.push_back(rpy[0]);
    results.push_back(rpy[1]);
    results.push_back(rpy[2]);

    return results;
}

// Initialize all distributions declared in the header file
void initRandomJointAngleGenerator(std::map<int, double> lowerLimit, std::map<int, double> upperLimit){
    std::cout << "Initializing random generator...\n";

    std::map<int, double>::iterator it_lower, it_upper; // Create iterators for both maps
 
    // Iterate maps together
    double currLower, currUpper;
    int currID;
    std::pair<int, std::uniform_real_distribution<double>> currPair;
    for(it_lower = lowerLimit.begin(), it_upper = upperLimit.begin(); it_lower != lowerLimit.end(); ++it_lower, ++it_upper){
        assert(it_lower->first == it_upper->first); // IDs should always match!

        currID = it_lower->first;

        // Get the specified limit values
        currLower = it_lower->second;
        currUpper = it_upper->second;

        // Instantiate a random uniform distribution object with specified bounds
        std::uniform_real_distribution<double> dist(currLower, currUpper);

        // Insert into the map declared in header together with ID
        randAngleDis.insert(std::pair<int, std::uniform_real_distribution<double>>(currID, dist));
    } 
}

// A std::map<int, double> is passed here just to get the joint IDs,
// since we're using this structure everywhere. The double value doesn't
// matter.
void initRandomJointChangeGenerator(std::map<int, double> joints_info){
    for(std::map<int, double>::iterator itr = joints_info.begin(); itr != joints_info.end(); ++itr){
        // Create distribution for random change in joint.
        std::uniform_real_distribution<double> dist(-DELTA_LIMIT, DELTA_LIMIT);

        // Insert into map declared on header, copying the joint ID.
        randChangeDis.insert(std::pair<int, std::uniform_real_distribution<double>>(itr->first, dist));
    }
}

std::map<int, double> generateRandomPose(){
    // Vector that will store the generated joint values for given IDs
    std::map<int, double> joint_results;
    // Create iterator object
    std::map<int, double>::iterator jointValuesItr; // For joint -- values pair
    std::map<int, std::uniform_real_distribution<double>>::iterator distItr; // for the random distributions

    double generated_value;
    int jointID;
    for(jointValuesItr = joint_results.begin(), distItr = randAngleDis.begin(); distItr != randAngleDis.end(); ++jointValuesItr, ++distItr){
        jointID = distItr->first; // Get ID of joint
        generated_value = distItr->second(random_engine); // Generate random value

        joint_results.insert(std::pair<int, double>(jointID, generated_value));
    }

    return joint_results;
}

std::map<int, double> applyRandomChange(std::map<int, double> currentJointAngles){
    assert(currentJointAngles.size() == randChangeDis.size());

    std::map<int, double> newJointAngles; // Return vector

    // Instantiate map iterators
    std::map<int, double>::iterator jointItr;
    std::map<int, std::uniform_real_distribution<double>>::iterator distItr;
    double random_change;
    double new_joint_value;
    for(jointItr = currentJointAngles.begin(), distItr = randChangeDis.begin(); jointItr != currentJointAngles.end(); ++jointItr, ++distItr){
        // Make sure IDs match
        assert(jointItr->first == distItr->first);
        // Generate random change
        random_change = distItr->second(random_engine);
        // Apply to joint. Sum current value to random change
        new_joint_value = random_change + jointItr->second;
        // Insert to new joint angles
        newJointAngles.insert(std::pair<int, double>(jointItr->first, new_joint_value));
    }

    return newJointAngles;
}

// Calculate the change (delta) in cartesian space, given the starting info of end effector
// and end position of end effector
std::vector<double> calcChangeInCartesian(std::vector<double> startEF, std::vector<double> endEF){
    std::vector<double> cartesianChange;
    
    assert(startEF.size() == endEF.size());

    for(int i = 0; i < startEF.size(); ++i)
        cartesianChange.push_back(endEF[i] - startEF[i]);

    return cartesianChange;
}

// Calculate the change (delta) in joints space, given the initial and final angle of the joints
std::vector<double> calcChangeInJoints(std::map<int, double> initial, std::map<int, double> end){
    std::vector<double> jointChange;

    // Should have the same number of joints
    assert(initial.size() == end.size());

    // Instantiate iterators
    std::map<int, double>::iterator inItr, enItr;
    for(inItr = initial.begin(), enItr = end.begin(); inItr != initial.end(); ++inItr, ++enItr){
        assert(inItr->first == enItr->first); // make sure the IDs of joints match

        jointChange.push_back(enItr->second - inItr->second);
    }

    return jointChange;
}

void printJointValues(std::map<int, double> jointInfo){
    for(std::map<int, double>::iterator itr = jointInfo.begin(); itr != jointInfo.end(); ++itr){
        std::cout << "[" << itr->first << "]: " << itr->second << " ";
    }
    std::cout << "\n";
}

void printEfInfo(std::vector<double> efInfo){
    std::cout << "X: " << efInfo.at(0) << " ";
    std::cout << "Y: " << efInfo.at(1) << " ";
    std::cout << "Z: " << efInfo.at(2) << " ";
    std::cout << "R: " << efInfo.at(3) << " ";
    std::cout << "P: " << efInfo.at(4) << " ";
    std::cout << "Y: " << efInfo.at(5) << "\n";
}

bool checkIfRotWithinLimits(double r, double p, double y, double limit){
    if (r <= -limit or r >= +limit)
        return false;
    
    if (p <= -limit or p >= +limit)
        return false;

    if (y <= -limit or y >= +limit)
        return false;

    return true;
}

bool checkIfEndPosWithinLimits(std::map<int, double> jointPos, std::map<int, double> upperLimits, std::map<int, double> lowerLimits){
    // Initialize iterators
    std::map<int, double>::iterator jointItr, lowerLimItr, upperLimItr;

    assert(jointPos.size() == upperLimits.size());
    assert(jointPos.size() == lowerLimits.size());

    double lower_limit, upper_limit, joint_value;
    for(jointItr = jointPos.begin(), lowerLimItr = lowerLimits.begin(), upperLimItr = upperLimits.begin();
        jointItr != jointPos.end();
        ++jointItr, ++upperLimItr, ++lowerLimItr){

        // ID of joint and limit should be the same!!!
        assert(jointItr->first == lowerLimItr->first);

        joint_value = jointItr->second;
        lower_limit = lowerLimItr->second;
        upper_limit = upperLimItr->second;
        if(joint_value <= lower_limit or joint_value >= upper_limit)
            return false;
    }

    return true;
}

// The vector representing the delta should be passed here. The function will check
// if the magnitude is less than the imposed limit
bool checkIfDistanceWithinLimit(double x, double y, double z, double limit){
    double distance;
    distance = sqrt(x * x + y * y + z * z);
    if (distance <= limit)
        return true;
    else
        return false;
}

void writeSamplesToDisk(std::vector<std::vector<double>> samples, const char * filename){
    std::ofstream file; // stream file

    file.open(filename, std::ios_base::app); // Open file in append mode
    for(int s = 0; s < samples.size(); s++){ // Iterate over samples
        file << samples.at(s).at(0);
        for(int v = 1; v < samples.at(s).size(); v++){ // Iterate over values
            file << "," << samples.at(s).at(v);
        }
        file << "\n";
    }
    file.close();
}

int main(int argc, const char* argv[]){
    assert(argc == 2);
    int left_arm_ef_id = 34;
    //MAX_CARTESIAN_STEP = std::stod(argv[1]);
    const char *WRITE_FILENAME = argv[1];

    //std::cout << "MAX_CARTESIAN_STEP: " << MAX_CARTESIAN_STEP << "\n";

    // Init robotis singleton
    robotis_ = new thormang3::KinematicsDynamics(thormang3::WholeBody);
    std::uniform_real_distribution<double> jointChangeGenerator;

    // Limits for each joint
    std::map<int, double> lowerLimit = {{2, -1.8393}, {4, 0.7837}, {6, -0.4403}, {8, -1.2003}, {10, 0.0514}, {12, -0.4201}, {14, -1.5500}};
    std::map<int, double> upperLimit = {{2, 0.5173}, {4, 2.2218}, {6, 0.8960}, {8, 1.1748}, {10, 1.8641}, {12, 0.9655}, {14, 0.3806}};
    initRandomJointAngleGenerator(lowerLimit, upperLimit); // Initialize the distribution generators with the given limits
    initRandomJointChangeGenerator(lowerLimit);

    std::map<int, double> initialJointValues; // Initial joint angles
    std::vector<double> startEfInfo; // Initial position end effector info
    std::map<int, double> endJointValues; // Joint angles after applying the random change
    std::vector<double> endEfInfo; // End position end effector info
    std::vector<double> jointDelta; // Change in joints space
    std::vector<double> cartesianDelta; // Change in cartesian space

    // Will store samples in this vector of vector before writing them to a file
    std::vector<std::vector<double>> samples;
    std::vector<double> sample; // vector to store a single sample

    int i = 0;
    int dropped = 0;
    while (1){
        // Generate random initial angles
        initialJointValues = generateRandomPose();
        // Calculate End Effector position and orientation
        startEfInfo = getFKResult(initialJointValues, left_arm_ef_id);

        // Generate random change to the joint values
        endJointValues = applyRandomChange(initialJointValues); 
        // Get new end effector information
        endEfInfo = getFKResult(endJointValues, left_arm_ef_id);

        // Make sure end pos is still within limis
        if(checkIfEndPosWithinLimits(endJointValues, upperLimit, lowerLimit) == false){
            dropped++;
            continue; // skip sample
        }

        // Calculate deltas in joint and cartesian space
        jointDelta = calcChangeInJoints(initialJointValues, endJointValues); 
        cartesianDelta = calcChangeInCartesian(startEfInfo, endEfInfo); 

        //std::cout << "Initial:\n";
        //printJointValues(initialJointValues);
        //printEfInfo(startEfInfo);
        //std::cout << "End:\n";
        //printJointValues(endJointValues);
        //printEfInfo(endEfInfo);

        //std::cout << "jointDelta:\n";
        //std::cout << jointDelta[0] << " ";
        //std::cout << jointDelta[1] << " ";
        //std::cout << jointDelta[2] << " ";
        //std::cout << jointDelta[3] << " ";
        //std::cout << jointDelta[4] << " ";
        //std::cout << jointDelta[5] << " ";

        //std::cout << "cartesianDelta:\n";
        //std::cout << cartesianDelta[0] << " ";
        //std::cout << cartesianDelta[1] << " ";
        //std::cout << cartesianDelta[2] << " ";
        //std::cout << cartesianDelta[3] << " ";
        //std::cout << cartesianDelta[4] << " ";
        //std::cout << cartesianDelta[5] << " ";
        if (checkIfRotWithinLimits(cartesianDelta.at(3), cartesianDelta.at(4), cartesianDelta.at(5), DELTA_LIMIT) and
            checkIfDistanceWithinLimit(cartesianDelta.at(0), cartesianDelta.at(1), cartesianDelta.at(2), MAX_DISTANCE)){
            // Clear vector to store a new sample
            sample.clear();
            
            // Store information in sample vector
            sample.push_back(cartesianDelta[0]);
            sample.push_back(cartesianDelta[1]);
            sample.push_back(cartesianDelta[2]);
            sample.push_back(cartesianDelta[3]);
            sample.push_back(cartesianDelta[4]);
            sample.push_back(cartesianDelta[5]);
            sample.push_back(initialJointValues.find(2)->second); // These are the joint ids
            sample.push_back(initialJointValues.find(4)->second); // These are the joint ids
            sample.push_back(initialJointValues.find(6)->second); // These are the joint ids
            sample.push_back(initialJointValues.find(8)->second); // These are the joint ids
            sample.push_back(initialJointValues.find(10)->second); // These are the joint ids
            sample.push_back(initialJointValues.find(12)->second); // These are the joint ids
            sample.push_back(initialJointValues.find(14)->second); // These are the joint ids
            sample.push_back(jointDelta[0]);
            sample.push_back(jointDelta[1]);
            sample.push_back(jointDelta[2]);
            sample.push_back(jointDelta[3]);
            sample.push_back(jointDelta[4]);
            sample.push_back(jointDelta[5]);
            sample.push_back(jointDelta[6]);

            // Store this sample in samples
            samples.push_back(sample);
            i++;
        }else
            dropped++;

        if (samples.size() == 5000){
            std::cout << "Writing samples to disk...\n";
            writeSamplesToDisk(samples, WRITE_FILENAME);
            samples.clear(); // Clear samples buffer
        }

        if (i % 500 == 0)
            std::cout << "Generated " << i << " samples. Dropped " << dropped << " samples. samples.size()" << samples.size() << "\n";
    }


    return 0;
}
