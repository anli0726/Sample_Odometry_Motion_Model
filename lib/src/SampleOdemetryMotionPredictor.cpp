/*
Programmer: An Li
Github: anli0726

The implementation of class SampleOdemetryMotionPredictor
*/

#include <iostream>
#include <math.h>
#include <stdexcept>
#include "SampleOdemetryMotionPredictor.h"

using namespace std;
using namespace Eigen;

SampleOdemetryMotionPredictor::SampleOdemetryMotionPredictor()
{
    // initial some reasonable values of the motion noise parameters
    motion_noises.insert(make_pair("rot", 0.0));
    motion_noises.insert(make_pair("rot_trans", 0.0000001));
    motion_noises.insert(make_pair("trans", 0.001));
    motion_noises.insert(make_pair("trans_rot", 0.0));
}

void SampleOdemetryMotionPredictor::setNoises()
{
    // set all motion noise parameters if not specify
    for (auto& name_val_pair: motion_noises)
    {
        cout << "Set " << name_val_pair.first << "'s value: ";
        cin >> name_val_pair.second;
        cout << endl;
    }
}

unordered_map<std::string, double> SampleOdemetryMotionPredictor::getNoises() const
{
    return motion_noises;
}

void SampleOdemetryMotionPredictor::setNoise(const string& param_name="")
{
    // if a parameter's name is specified, set it unless it doesn't exist
    if (motion_noises.count(param_name))
    {
        cout << "Set " << param_name << "'s value: ";
        cin >> motion_noises[param_name];
        cout << endl;
    }
    else
        throw invalid_argument("name of the setting motion noise parameter doesn't exist");
}

double SampleOdemetryMotionPredictor::getNoise(const string& param_name)
{
    if (motion_noises.count(param_name))
        return motion_noises[param_name];

    else
        throw invalid_argument("name of the finding motion noise parameter doesn't exist");
}


double SampleOdemetryMotionPredictor::sampling(double std)
{   
    // sampling from a (approximation of) normal distribution 
    // with mean 0 and variance std^2

    double sample = 0.0;
    // randomly generating from a uniform distribution
    uniform_real_distribution<double> distribution(-std, std); 
    for (int i = 0; i < 12; ++i)
        sample += distribution(SampleOdemetryMotionPredictor::generator);
    return 0.5 * sample;
}

Vector3d SampleOdemetryMotionPredictor::predict(const Vector3d& robot_pose_prev,
                                                 const Vector3d& robot_pose,
                                                 const Vector3d& real_pose_prev)
{                                                 
    /*
    input:
        robot_pose_prev = [x_r, y_r, theta_r].T: last reading from the odometry
        robot_pose = [x_r', y_r', theta_r'].T: current reading from the odometry
        real_pose_prev = [x, y, theta].T: known actual last pose of the robot 

    return:
        [x', y', theta].T : a sample particle of the predicted robot pose at time t 


    Algorithm explanation:
        Using the relative motion information from the robot odemetry reading (motor encoder)
        to predict the actual relative motion of the robot, i.e., using the "difference of the 
        odemetry readings" to predict the "difference of the robot actual movement" in one time step.
        In this approach, the "control" of the robot is excatly the difference of the odometry readings.

        To extract the relative distance, the control is decomposed into three steps/parameters: first rotation (d_rot_1), 
        linear translation (d_trans), and second rotation (d_rot_2). 

        The model assumes these three motion parameters are affected by independ noises: pure rotation noise,
        pure transtaion noise, noise from first rotating then translating, and noise from first translating 
        then rotating.

    */
    Vector3d pose_predict = Vector3d::Zero();

    // extract relative motion parameters of odometry reading
    double d_rot_1 = atan2(robot_pose(1) - robot_pose_prev(1), 
                           robot_pose(0) - robot_pose_prev(0)) - robot_pose_prev(2);
    double d_trans = sqrt(pow((robot_pose(0) - robot_pose_prev(0)), 2) + 
                          pow((robot_pose(1) - robot_pose_prev(1)), 2));
    double d_rot_2 = robot_pose(2) - robot_pose_prev(2) - d_rot_1;


    // calculate the predicted relative actual movement. In this case this is an samling approach,
    // so the standard deviation of the probability distibution of each parameter is calculated, 
    // sampled, then subtract with the relaive motion parameters
    double rot_std_1 = sqrt(motion_noises["rot"] * pow(d_rot_1, 2) + 
                            motion_noises["rot_trans"] * pow(d_trans,2));
    double hat_d_rot_1 = d_rot_1 - sampling(rot_std_1);

    double std_trans = sqrt(motion_noises["trans"] * pow(d_trans, 2) + 
                            motion_noises["trans_rot"] * (pow(d_rot_1, 2) + pow(d_rot_2,2)));
    double hat_d_trans = d_trans - sampling(std_trans);

    double std_rot_2 = sqrt(motion_noises["rot"] * pow(d_rot_2, 2) + 
                            motion_noises["rot_trans"] * pow(d_trans, 2));
    double hat_d_rot_2 = d_rot_2 - sampling(std_rot_2);

    // predict the pose of the robot by the predicted relative actual movement
    pose_predict(0) = real_pose_prev(0) + hat_d_trans * cos(real_pose_prev(2) + hat_d_rot_1);
    pose_predict(1) = real_pose_prev(1) + hat_d_trans * sin(real_pose_prev(2) + hat_d_rot_1);
    pose_predict(2) = real_pose_prev(2) + hat_d_rot_1 + hat_d_rot_2;

    return pose_predict;
}