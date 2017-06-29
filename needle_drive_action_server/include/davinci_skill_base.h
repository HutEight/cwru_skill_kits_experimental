#ifndef DAVINCI_SKILL_BASE_H
#define  DAVINCI_SKILL_BASE_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <string>
#include <math.h>

#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/QuadWord.h>
#include <eigen3/Eigen/src/Geometry/Transform.h>
#include <cwru_davinci_kinematics/davinci_kinematics.h>

#include <geometry_msgs/Polygon.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int32MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>

// On probation TODO delete if found useless
const double DEFAULT_NEEDLE_RADIUS = 0.012;
const double DEFAULT_NEEDLE_GRASP_DEPTH = 0.005;
const double DEFAULT_NEEDLE_AXIS_HT = DEFAULT_NEEDLE_RADIUS / 1.5;

const int GRASP_W_NEEDLE_POSITIVE_GRIPPER_Y = 1;
const int GRASP_W_NEEDLE_NEGATIVE_GRIPPER_Y = -1;
const int GRASP_W_NEEDLE_POSITIVE_GRIPPER_Z = 1;
const int GRASP_W_NEEDLE_NEGATIVE_GRIPPER_Z = -1;

const int NSAMPS_DRIVE_PLAN = 21;
const double DEFAULT_PHI_GRAB = 0.0;

const double LCAMERA_TO_PSM_ONE_TRANSLATION[] = {-0.155, -0.03265, 0.0};
const double LCAMERA_TO_PSM_TWO_TRANSLATION[] = {0.145, -0.03265, 0.0};
#endif
