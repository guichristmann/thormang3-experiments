#include <random>
#include <map>

std::map<int, double> generateRandomPose();
std::map<int, double> applyRandomChange(std::map<int, double>);
void initRandomJointAngleGenerator(std::map<int, double>, std::map<int, double>);
void initRandomJointChangeGenerator(std::map<int, double>);

// Give joint IDs and Angles, return the position and orientation of end effector
std::vector<double> getFKResult(std::map<int, double> joint_values, int end_effector_id);

// This is distribution generator for the initial position of the joints
std::map<int, std::uniform_real_distribution<double>> randAngleDis;
// This is distribution generator for the changes in joints
std::map<int, std::uniform_real_distribution<double>> randChangeDis;
// Random engine used, time is passed as a seed
std::default_random_engine random_engine{static_cast<long unsigned int>(time(NULL))};

// The Robotis singleton
thormang3::KinematicsDynamics *robotis_;

// Will be read from command line
double MAX_CARTESIAN_STEP;
