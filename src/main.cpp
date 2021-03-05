/*
Programmer: An Li
Github: anli0726

The demonstration of the pose predction by the sampled odometery motion model.
*/

#include <iostream>
#include <math.h>
#include <vector>
#include <array>
#include <opencv2/opencv.hpp>
#include "SampleOdemetryMotionPredictor.h"

using namespace std;
using namespace Eigen;

cv::Scalar WHITE {255, 255, 255};
cv::Scalar BLUE {255, 0, 0};

void printVector3d(const Vector3d& vec)
{
    for (int i = 0; i < vec.size(); i++)
        cout << vec(i) << " ";
}

// draw the robot pose changes
void drawRobotMovement(const cv::Mat& img, 
                       const Vector3d& robot_pose_prev,
                       const Vector3d& robot_pose)
{
    double x_prev = robot_pose_prev[0];
    double y_prev = robot_pose_prev[1];
    double theat_prev = robot_pose_prev[2];

    double x = robot_pose[0];
    double y = robot_pose[1];
    double theta = robot_pose[2];

    cv::circle(img, cv::Point(x_prev, y_prev), 5, WHITE, 1, cv::LINE_8);
    cv::circle(img, cv::Point(x, y), 5, WHITE, 1, cv::LINE_8);

    cv::line(img, cv::Point(x_prev, y_prev), cv::Point(x, y), WHITE, 1, cv::LINE_8);
}

int main(int argc, char** argv)
{
    // initialize the robot pose
    vector<Vector3d> robot_poses;
    Vector3d real_pose_init; real_pose_init << 150.0, 150.0, 0.0;
    robot_poses.push_back(real_pose_init);

    // using 500 samples (particles) to apporximate the close form result
    const size_t num_samples = 500;
    SampleOdemetryMotionPredictor predictor;

    // motion control
    double d_traslation = 100.0;
    array<Vector3d, num_samples> predict_poses {};
    
    // vairalbes for visulization
    cv::Mat img = cv::Mat::zeros(500, 500, CV_8UC3);
    string window_name = "img";
    
    // go for 7 time steps
    for (int i = 1; i < 7; i++)
    {
        double theta = robot_poses[i-1](2);
        Vector3d delta; delta << d_traslation * cos(theta),
                                 d_traslation * sin(theta),
                                 0.0;
        // robot rotate 90 degree and move every two time steps
        if (i%2 == 0) 
            delta(2) += M_PI/2; 
        
        robot_poses.push_back(robot_poses[i-1] + delta);

        drawRobotMovement(img, robot_poses[i-1], robot_poses[i]);

        /*
        Generate 500 particles and plot them.
        This demonstration assumes robot follows exactly what the robot odometry reads, 
        yet every blue point (predict pose) follows the control update of the Particle Filter.
        This shows that, without measurement update (sensor reading, in this appproach it is NOT
        the reading of odometry) to localize the robot, the pose prediction will expand in the 
        space.
        */  
        for (int j = 0; j < num_samples; j++)
        {
            Vector3d point = Vector3d::Zero();
            if (i == 1)
                point = predictor.predict(robot_poses[i-1], 
                                           robot_poses[i], 
                                           real_pose_init);
            else
                point = predictor.predict(robot_poses[i-1], 
                                           robot_poses[i], 
                                           predict_poses[j]);
            predict_poses[j] = point;
            cv::circle(img, cv::Point(point(0), point(1)), 1, BLUE, cv::FILLED, cv::LINE_8);
        }
        cv::imshow(window_name, img);
        cv::waitKey();
    }
    return 0; 
}