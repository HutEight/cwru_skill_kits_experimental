/*
 * RN
 * 12/6/17
 * A refactored version of Derlin's needle planner
 * For experimental purposes
 * Supporting the needle drive action servers
 */

#ifndef NEEDLE_PLANNER_H
#define  NEEDLE_PLANNER_H

#include <davinci_skill_base.h>
#include <templates.h>


class NeedlePlanner {

public:
  NeedlePlanner(ros::NodeHandle &nodeHandle);
  
  inline void setNeedleRadius(double r) { needle_radius = r; }
  
  inline void setNeedleAxisHt(double h) { needle_axis_ht = h; }
  
  inline void setPsiNeedleAxisTiltWrtTissue(double tilt) { psi_needle_axis_tilt_wrt_tissue = tilt; }
  
  inline void setKvec(Eigen::Vector3d kvec) { kvec_needle = kvec; }
  
  inline void setNeedleOrigin(Eigen::Vector3d O_needle) { O_needle = O_needle; }
  
  inline void
  setAffineGraspFrameWrtGripperFrame(Eigen::Affine3d affine) { affine_grasp_frame_wrt_gripper_frame = affine; }
  
  inline void setAffineNeedleFrameWrtTissue(Eigen::Affine3d affine) { affine_needle_frame_wrt_tissue = affine; }
  
  inline void setGraspDepth(double depth) { grasp_depth = depth; }
  
  inline void set_grab_needle_plus_minus_y(int needle_plus_minus_y) {
    if (GRASP_W_NEEDLE_POSITIVE_GRIPPER_Y == grab_needle_plus_minus_y)
      grab_needle_plus_minus_y = needle_plus_minus_y;
    else if (GRASP_W_NEEDLE_POSITIVE_GRIPPER_Y == grab_needle_plus_minus_y)
      grab_needle_plus_minus_y = needle_plus_minus_y;
    else
      ROS_WARN("grasp status not legal; not being changed");
  }
  
  inline void set_grab_needle_plus_minus_z(int needle_plus_minus_z) {
    if (GRASP_W_NEEDLE_POSITIVE_GRIPPER_Z == grab_needle_plus_minus_z)
      grab_needle_plus_minus_z = needle_plus_minus_z;
    else if (GRASP_W_NEEDLE_NEGATIVE_GRIPPER_Z == grab_needle_plus_minus_z)
      grab_needle_plus_minus_z = needle_plus_minus_z;
    else
      ROS_WARN("grasp status needle z-axis sign not legal; not being changed");
  }
  
  inline void set_affine_needle_frame_wrt_gripper_frame(Eigen::Affine3d affine) {
    affine_needle_frame_wrt_gripper_frame = affine;
  }
  
  inline void set_affine_lcamera_to_psm_one(Eigen::Affine3d affine) {
    default_affine_lcamera_to_psm_one = affine;
  }
  
  inline void set_affine_lcamera_to_psm_two(Eigen::Affine3d affine) {
    default_affine_lcamera_to_psm_two = affine;
  }
  
  inline Eigen::Affine3d transformTFToEigen(const tf::Transform &t) {
    Eigen::Affine3d e;
    for (int i = 0; i < 3; i++) {
      e.matrix()(i, 3) = t.getOrigin()[i];
      for (int j = 0; j < 3; j++) {
        e.matrix()(i, j) = t.getBasis()[i][j];
      }
    }
    // Fill in identity in last row
    for (int col = 0; col < 3; col++)
      e.matrix()(3, col) = 0;
    e.matrix()(3, 3) = 1;
    return e;
  }
  
  void compute_grasp_transform();
  
  void compute_grasp_transform(double phi_x, double phi_y);
  
  void compute_tissue_frame_wrt_camera(Eigen::Vector3d entrance_pt,
                                       Eigen::Vector3d exit_pt,
                                       Eigen::Vector3d tissue_normal);
  
  void compute_needle_drive_gripper_affines(std::vector<Eigen::Affine3d> &gripper_affines_wrt_camera,
                                            Eigen::VectorXi &ik_ok_array,
                                            int &ik_score);
  
  void init_poses(Eigen::Affine3d &g_affine_lcamera_to_psm_one,
                  Eigen::Affine3d &g_affine_lcamera_to_psm_two,
                  Eigen::Affine3d &g_psm1_start_pose,
                  Eigen::Affine3d &g_psm2_start_pose);
  
  
  
  
  void inPointCallback(const geometry_msgs::Point &pt_msg);
  
  inline void print_affine(Eigen::Affine3d affine) {
    cout << "Rotation: " << endl;
    cout << affine.linear() << endl;
    cout << "origin: " << affine.translation().transpose() << endl;
  }
  
  inline Eigen::Matrix3d Rotz(double phi) {
    Eigen::Matrix3d Rx;
    Rx(0, 0) = 1.0; //
    Rx(0, 1) = 0.0;
    Rx(0, 2) = 0.0;
    Rx(1, 0) = 0.0;
    Rx(1, 1) = cos(phi);
    Rx(1, 2) = -sin(phi);
    Rx(2, 0) = 0.0;
    Rx(2, 1) = sin(phi);
    Rx(2, 2) = cos(phi);
    return Rx;
  }
  
  inline Eigen::Matrix3d Roty(double phi) {
    Eigen::Matrix3d Rz;
    Rz(0, 0) = cos(phi);
    Rz(0, 1) = -sin(phi);
    Rz(0, 2) = 0.0;
    Rz(1, 0) = sin(phi);
    Rz(1, 1) = cos(phi);
    Rz(1, 2) = 0.0;
    Rz(2, 0) = 0.0;
    Rz(2, 1) = 0.0;
    Rz(2, 2) = 1.0;
    return Rz;
  }
  
  inline Eigen::Matrix3d Rotx(double phi) {
    Eigen::Matrix3d Roty;
    Roty(0, 0) = cos(phi);
    Roty(0, 1) = 0;
    Roty(0, 2) = sin(phi);
    Roty(1, 0) = 0;
    Roty(1, 1) = 1.0;
    Roty(1, 2) = 0.0;
    Roty(2, 0) = -sin(phi);
    Roty(2, 1) = 0.0;
    Roty(2, 2) = cos(phi);
    return Roty;
  }
  
  inline Eigen::Matrix3d Rot_k_phi(Eigen::Vector3d k_vec, double phi) {
    Eigen::Matrix3d R_k_phi;
    double kx = k_vec(0);
    double ky = k_vec(1);
    double kz = k_vec(2);
    Eigen::Matrix3d K;
    K(0, 0) = 0.0;
    K(0, 1) = -kz;
    K(0, 2) = ky;
    K(1, 0) = kz;
    K(1, 1) = 0.0;
    K(1, 2) = -kx;
    K(2, 0) = -ky;
    K(2, 1) = kx;
    K(2, 2) = 0;
    Eigen::Matrix3d I = Eigen::MatrixXd::Identity(3, 3);
    R_k_phi = I + sin(phi) * K + (1 - cos(phi)) * K * K;
    return R_k_phi;
  }
  
  void driveNeedle(const geometry_msgs::Point &needle_entrance_pt,
                   const double suture_depth,
                   std::vector<geometry_msgs::Point> &needle_exit_pts_list,
                   std::vector<geometry_msgs::Transform> &grasp_transform_list,
                   std::vector<trajectory_msgs::JointTrajectory> &trajectory_list);
  
  bool driveNeedle(const geometry_msgs::Point &needle_entrance_pt,
                   const geometry_msgs::Point &needle_exit_pt,
                   geometry_msgs::Transform &grasp_transform,
                   trajectory_msgs::JointTrajectory &trajectory);
  
  bool hasTrajectory(const geometry_msgs::Point &needle_entrance_pt,
                     const geometry_msgs::Point &needle_exit_pt,
                     const geometry_msgs::Transform &grasp_transform,
                     trajectory_msgs::JointTrajectory &trajectory);

  
protected:
  
  // TODO format names
  
  // TODO see which one is NOT necessary
  ros::NodeHandle nh;
  // ros::Subscriber thePoint;
  ros::Publisher exit_pt_publisher;
  ros::Publisher exit_pt_score_publisher;
  ros::Publisher exit_pt_array_publisher;
  
  // These are initial tfs
  //g_affine_lcamera_to_psm_one;
  //g_affine_lcamera_to_psm_two;
  //g_psm1_start_pose;
  //g_psm2_start_pose;
  
  
  
  double needle_radius;
  double needle_axis_ht;
  double psi_needle_axis_tilt_wrt_tissue;
  double grasp_depth;
  int grab_needle_plus_minus_y;
  int grab_needle_plus_minus_z;
  double phi_grab_;
  double dist_entrance_to_exit;
  
  // Define needle grasp
  // phi_grab_=0 --> grab at center
  // phi_grab_=pi/2 --> grab at tail
  // phi_grab = -pi/2 --> grab at tip
  double phi_insertion;
  
  Eigen::Vector3d kvec_needle;
  
  // O_needle is arbitrary frame...e.g. psm1_base
  Eigen::Vector3d O_needle;
  
  // Describe how the needle is held by the gripper
  Eigen::Affine3d affine_grasp_frame_wrt_gripper_frame;
  Eigen::Vector3d O_needle_frame_wrt_grasp_frame;
  
  Eigen::Affine3d affine_needle_frame_wrt_tissue;
  
  // Specify needle orientation
  // e.g., needle z-axis typically parallel or anti-parallel to gripper bvec
  Eigen::Vector3d bvec_needle_wrt_grasp_frame;
  Eigen::Vector3d nvec_needle_wrt_grasp_frame;
  Eigen::Vector3d tvec_needle_wrt_grasp_frame;
  
  // TODO what are R and R0, O and O0? Comment them in the right places
  // R rotation (therefore a matrix)
  // O origin (therefore a vector)
  
  Eigen::Matrix3d R_needle_frame_wrt_grasp_frame;
  
  Eigen::Matrix3d R0_N_wrt_Grasp;
  Eigen::Vector3d O0_N_wrt_Grasp;
  
  Eigen::Matrix3d R0_N_wrt_G;
  Eigen::Vector3d O0_N_wrt_G;
  Eigen::Affine3d affine_needle_frame_wrt_grasp_frame;
  Eigen::Affine3d affine_needle_frame_wrt_gripper_frame;
  
  Eigen::Vector3d nvec_tissue_frame_wrt_camera, tvec_tissue_frame_wrt_camera, bvec_tissue_frame_wrt_camera;
  Eigen::Vector3d desired_needle_entrance_point;
  Eigen::Vector3d repaired_exit_pt;
  
  Eigen::Matrix3d R_tissue_frame_wrt_camera_frame;
  Eigen::Affine3d affine_tissue_frame_wrt_camera_frame;
  
  Eigen::Vector3d bvec_needle_wrt_tissue_frame;
  Eigen::Vector3d nvec_needle_wrt_tissue_frame;
  Eigen::Vector3d tvec_needle_wrt_tissue_frame;
  
  // Initial orientation of needle frame w/rt tissue;
  Eigen::Matrix3d R0_needle_wrt_tissue;
  
  Eigen::Vector3d O_needle_wrt_tissue;
  
  Eigen::Affine3d affine_init_needle_frame_wrt_tissue;
  
  Eigen::Matrix3d R_needle_wrt_tissue;
  
  Eigen::Affine3d affine_gripper_frame_wrt_tissue;
  Eigen::Affine3d affine_needle_frame_wrt_camera;
  Eigen::Affine3d affine_gripper_frame_wrt_camera_frame;
  
  Eigen::Affine3d default_affine_lcamera_to_psm_one;
  Eigen::Affine3d default_affine_lcamera_to_psm_two;
  
  Davinci_fwd_solver davinci_fwd_solver;
  Davinci_IK_solver ik_solver;
  
  
}; // end of class TODO delete after debug



#endif

