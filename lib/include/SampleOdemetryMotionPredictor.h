/*
Programmer: An Li
Github: anli0726

Header file of the class SampleOdemetryMotionPredictor.
The class creats a motion model based on the odemetry reading [x, y, theta] of a 2D robot to 
do the control update steps, which predict the pose of the robot based on the control input 
(odometry readings of two adjacent time steps in this case), in the Particle Filter (nonparametric 
implementation of Bayes Filter).

All the algorithms are from Probabilistic Roboitcs by Sebastian Thrun et. al.
*/


#ifndef SAMPLEODEMETRY_H
#define SAMPLEODEMETRY_H

#include <unordered_map>
#include <vector>
#include <random>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>


class SampleOdemetryMotionPredictor
{
public:
    SampleOdemetryMotionPredictor();
    virtual ~SampleOdemetryMotionPredictor() {}

    // setter and getter of the motion noise parameters
    void setNoises();
    std::unordered_map<std::string, double> getNoises() const;
    
    void setNoise(const std::string& param_name);
    double getNoise(const std::string& param_name);

    // sampling for a normal distribution 
    double sampling(double std);
    // pose prediction
    Eigen::Vector3d predict(const Eigen::Vector3d& robot_pose_prev,
                            const Eigen::Vector3d& robot_pose,
                            const Eigen::Vector3d& real_pose_prev);    

private:
    std::unordered_map<std::string, double> motion_noises;
    std::default_random_engine generator;
};
#endif