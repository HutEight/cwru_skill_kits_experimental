

#include <needle_planner.h>


using std::vector;

NeedlePlanner::NeedlePlanner(ros::NodeHandle &nodeHandle) : nh(nodeHandle) {
  ROS_INFO_STREAM("Constructing a needle planner..");
  
  Eigen::Matrix3d R;
  Eigen::Vector3d nvec, tvec, bvec;
  Eigen::Vector3d O_grasp_frame;
  
  needle_radius = DEFAULT_NEEDLE_RADIUS;
  needle_axis_ht = DEFAULT_NEEDLE_AXIS_HT;
  grasp_depth = DEFAULT_NEEDLE_GRASP_DEPTH;
  grab_needle_plus_minus_y = GRASP_W_NEEDLE_POSITIVE_GRIPPER_Y;
  phi_grab_ = DEFAULT_PHI_GRAB;
  dist_entrance_to_exit = 2 * sqrt(needle_radius * needle_radius - needle_axis_ht * needle_axis_ht);
  
  // Default choice: needle origin in +y half space
  grab_needle_plus_minus_y = GRASP_W_NEEDLE_POSITIVE_GRIPPER_Y;
  // Default choice: needle z-axis parallel to gripper z axis
  grab_needle_plus_minus_z = GRASP_W_NEEDLE_POSITIVE_GRIPPER_Z;
  
  //hard-coded camera-to-base transform, useful for simple testing/debugging
  default_affine_lcamera_to_psm_one.translation() << -0.155, -0.03265, 0.0;
  
  nvec << -1, 0, 0;
  tvec << 0, 1, 0;
  bvec << 0, 0, -1;
  
  R.col(0) = nvec;
  R.col(1) = tvec;
  R.col(2) = bvec;
  default_affine_lcamera_to_psm_one.linear() = R;
  
  O_needle_wrt_tissue << 0.5 * dist_entrance_to_exit, 0, needle_axis_ht;
  bvec_needle_wrt_tissue_frame << 0, -1, 0;
  
  nvec_needle_wrt_tissue_frame << -1, 0, 0;
  tvec_needle_wrt_tissue_frame = bvec_needle_wrt_tissue_frame.cross(nvec_needle_wrt_tissue_frame);
  
  R0_needle_wrt_tissue.col(0) = nvec_needle_wrt_tissue_frame;
  R0_needle_wrt_tissue.col(1) = tvec_needle_wrt_tissue_frame;
  R0_needle_wrt_tissue.col(2) = bvec_needle_wrt_tissue_frame;
  
  affine_init_needle_frame_wrt_tissue.linear() = R0_needle_wrt_tissue;
  affine_init_needle_frame_wrt_tissue.translation() = O_needle_wrt_tissue;
  
  R << 1, 0, 0,
          0, 1, 0,
          0, 0, 1;
  // TODO can one clear a Eigen Matrix?
  
  affine_grasp_frame_wrt_gripper_frame.linear() = R;
  
  O_grasp_frame << 0, 0, -grasp_depth;
  affine_grasp_frame_wrt_gripper_frame.translation() = O_grasp_frame;
  
  compute_grasp_transform();
  
  R0_N_wrt_Grasp = affine_needle_frame_wrt_grasp_frame.linear();
  O0_N_wrt_Grasp = affine_needle_frame_wrt_grasp_frame.translation();
  
  // w/rt G(ripper)
  R0_N_wrt_G = affine_needle_frame_wrt_gripper_frame.linear();
  O0_N_wrt_G = affine_needle_frame_wrt_gripper_frame.translation();
  
  // g_got_new_entry_point = false; // TODO delete
  
  // thePoint = nh.subscribe("/thePoint", 5, &NeedlePlanner::inPointCallback, this);
  exit_pt_publisher = nh.advertise<geometry_msgs::Point>("exit_points", 1);
  exit_pt_score_publisher = nh.advertise<std_msgs::Int32MultiArray>("exit_points_score", 1);
  exit_pt_array_publisher = nh.advertise<geometry_msgs::Polygon>("exit_point_array", 1);
  
}


void NeedlePlanner::driveNeedle(const geometry_msgs::Point &needle_entrance_pt,
                                const double suture_depth,
                                std::vector<geometry_msgs::Point> &needle_exit_pts_list,
                                std::vector<geometry_msgs::Transform> &grasp_transform_list,
                                std::vector<trajectory_msgs::JointTrajectory> &trajectory_list) {
  
  // TODO try to put them in to the .h
  // TODO rearrange sequence
  const int g_npts_good = 32;
  Eigen::Affine3d g_affine_lcamera_to_psm_one;
  Eigen::Affine3d g_affine_lcamera_to_psm_two;
  Eigen::Affine3d g_psm1_start_pose;
  Eigen::Affine3d g_psm2_start_pose;
  const double r_needle = 0.012;
  const double needle_height_above_tissue = r_needle - suture_depth;
  const double d_to_exit_pt = 2 * r_needle / sqrt(2.0);
  const double z_tissue = 0.10;
  Eigen::Vector3d g_O_entry_point;
  bool g_got_new_entry_point = false;
  
  Eigen::Vector3d O_needle;
  Eigen::Vector3d O_entrance_pt;
  Eigen::Vector3d O_exit_pt;
  
  geometry_msgs::Polygon polygon_msg;
  
  std_msgs::Int32MultiArray exit_pts_score;
  
  // std::ofstream outfile;
  
  vector<Eigen::Affine3d> gripper_affines_wrt_camera;
  
  Eigen::Vector3d gripper1_motion;
  Eigen::Vector3d gripper2_motion;
  
  Eigen::VectorXi ik_ok_array(45);
  int ik_score = 0;
  
  vector<geometry_msgs::Point> exit_points;
  geometry_msgs::Point exitPoint;
  geometry_msgs::Point32 p32;
  
  double needle_x, needle_y;
  Eigen::Vector3d v_entrance_to_exit, v_entrance_to_exit0, tissue_normal;
  
  tissue_normal << 0, 0, -1;
  
  v_entrance_to_exit0 << 0, -1, 0;
  
  
  g_O_entry_point(0) = needle_entrance_pt.x;
  g_O_entry_point(1) = needle_entrance_pt.y;
  g_O_entry_point(2) = needle_entrance_pt.z;
  
  init_poses(g_affine_lcamera_to_psm_one,
             g_affine_lcamera_to_psm_two,
             g_psm1_start_pose,
             g_psm2_start_pose);
  
  set_affine_lcamera_to_psm_one(g_affine_lcamera_to_psm_one);
  set_affine_lcamera_to_psm_two(g_affine_lcamera_to_psm_two);
  
  for (needle_x = 0; needle_x < 0.1; needle_x += 0.7854) {

    polygon_msg.points.clear();
    exit_pts_score.data.clear();
    
    
    double kvec_yaw = 0.0; // rotation of needle z-axis w/rt camera x-axis //TODO RN CORRECTION: needle z (kvec_needle) about camera z (pointing down)
    
    O_entrance_pt = g_O_entry_point;
    
    //compute the tissue frame in camera coords, based on point-cloud selections, when test use 0.1, normal 6.28:
    for (kvec_yaw = 1.5708; kvec_yaw < 1.5709; kvec_yaw += 0.1) {
      v_entrance_to_exit = Rotz(kvec_yaw) * v_entrance_to_exit0; //rotate the needle axis about camera z-axis
      O_exit_pt = O_entrance_pt + d_to_exit_pt * v_entrance_to_exit;
      O_needle = 0.5 * (O_exit_pt + O_entrance_pt);
      O_needle(2) -= needle_height_above_tissue;
      std::cout << "O_entrance_pt = " << O_entrance_pt.transpose() << std::endl;
      std::cout << "O_needle = " << O_needle.transpose() << std::endl;
      std::cout << "O_exit_pt = " << O_exit_pt.transpose() << std::endl;
      gripper_affines_wrt_camera.clear();
      
      // needle_x = 0;
      needle_y = 0;
      
      ik_score = 0;
      
      compute_grasp_transform(needle_x, needle_y);
      
      compute_tissue_frame_wrt_camera(O_entrance_pt, O_exit_pt, tissue_normal);
      
      // needlePlanner.compute_needle_drive_gripper_affines is computing circular needle driving
      
      compute_needle_drive_gripper_affines(gripper_affines_wrt_camera, ik_ok_array, ik_score);

      int nposes = gripper_affines_wrt_camera.size();
      std::cout << "ik_ok_point:: " << ik_ok_array.transpose() << std::endl;
      std::cout << "score:: " << ik_score << std::endl;
      ROS_WARN("at kvec_yaw = %f, computed %d needle-drive gripper poses, score %d ", kvec_yaw, nposes, ik_score);
      
      
      if ((nposes >= g_npts_good) & (ik_score > 0)) {
        exitPoint.x = O_exit_pt(0);
        exitPoint.y = O_exit_pt(1);
        exitPoint.z = O_exit_pt(2);
        p32.x = O_exit_pt(0);
        p32.y = O_exit_pt(1);
        p32.z = O_exit_pt(2);
        
        // outfile << ik_score << ", " << O_entrance_pt(0) << ", " << O_entrance_pt(1) << ", " << O_exit_pt(0) << ", "
        //        << O_exit_pt(1) << std::endl;
        
        exit_pt_publisher.publish(exitPoint);
        polygon_msg.points.push_back(p32);
        exit_pts_score.data.push_back(ik_score);
        exit_points.push_back(exitPoint); //not used...
      }
      
      
    }
    
    if (polygon_msg.points.size() > 0) {
//RN EXPERIMENT STARTS HERE
      ROS_INFO("HMS DRAGON. PUBLISHING TO /exit_point_array"); //RN ADDS
      exit_pt_score_publisher.publish(exit_pts_score);
      exit_pt_array_publisher.publish(polygon_msg);// for rviz display
      std::cout << "size of exit points: " << polygon_msg.points.size() << std::endl;
    }

    
  }
  
  
}


bool NeedlePlanner::driveNeedle(const geometry_msgs::Point &needle_entrance_pt,
                                const geometry_msgs::Point &needle_exit_pt,
                                geometry_msgs::Transform &grasp_transform,
                                trajectory_msgs::JointTrajectory &trajectory) {
  
  
}


bool NeedlePlanner::hasTrajectory(const geometry_msgs::Point &needle_entrance_pt,
                                  const geometry_msgs::Point &needle_exit_pt,
                                  const geometry_msgs::Transform &grasp_transform,
                                  trajectory_msgs::JointTrajectory &trajectory) {
  
  
}


void NeedlePlanner::compute_grasp_transform() {
  ROS_INFO_STREAM("computing grasp transform..");
  
  // Needle Origin w/ respect to grasp frame
  O_needle_frame_wrt_grasp_frame << 0, grab_needle_plus_minus_y * needle_radius, 0;
  
  bvec_needle_wrt_grasp_frame << 0, 0, grab_needle_plus_minus_z;
  nvec_needle_wrt_grasp_frame << 0, grab_needle_plus_minus_y, 0;
  tvec_needle_wrt_grasp_frame = bvec_needle_wrt_grasp_frame.cross(nvec_needle_wrt_grasp_frame);
  
  // Rotation Matrix
  R_needle_frame_wrt_grasp_frame.col(0) = nvec_needle_wrt_grasp_frame;
  R_needle_frame_wrt_grasp_frame.col(1) = tvec_needle_wrt_grasp_frame;
  R_needle_frame_wrt_grasp_frame.col(2) = bvec_needle_wrt_grasp_frame;
  
  affine_needle_frame_wrt_grasp_frame.linear() = R_needle_frame_wrt_grasp_frame;
  affine_needle_frame_wrt_grasp_frame.translation() = O_needle_frame_wrt_grasp_frame;
  
  affine_needle_frame_wrt_gripper_frame = affine_grasp_frame_wrt_gripper_frame * affine_needle_frame_wrt_grasp_frame;
}


/// TODO add comment
/// \param phi_x
/// \param phi_y
void NeedlePlanner::compute_grasp_transform(double phi_x, double phi_y) {
  Eigen::Matrix3d R_N_wrt_G, Rx, Ry;
  Eigen::Vector3d O_N_wrt_G;
  Eigen::Matrix3d R_N_wrt_Grasp;
  Eigen::Vector3d O_N_wrt_Grasp;
  
  Rx = Rotx(phi_x);
  Ry = Roty(phi_y);
  
  R_N_wrt_Grasp = Rx * Ry * R0_N_wrt_Grasp;
  affine_needle_frame_wrt_grasp_frame.linear() = R_N_wrt_Grasp;
  
  O_N_wrt_Grasp = Rx * O0_N_wrt_Grasp;
  
  affine_needle_frame_wrt_grasp_frame.translation() = O_N_wrt_Grasp;
  
  affine_needle_frame_wrt_gripper_frame =
          affine_grasp_frame_wrt_gripper_frame * affine_needle_frame_wrt_grasp_frame;
}


/// TODO add more comment
/// Return a vector full of affines describing desired gripper frames w/rt camera frame.
/// \param entrance_pt
/// \param exit_pt
/// \param tissue_normal
void NeedlePlanner::compute_tissue_frame_wrt_camera(Eigen::Vector3d entrance_pt,
                                                    Eigen::Vector3d exit_pt,
                                                    Eigen::Vector3d tissue_normal) {
  //set up tissue frame w/rt camera
  bvec_tissue_frame_wrt_camera = tissue_normal;
  nvec_tissue_frame_wrt_camera = (exit_pt - entrance_pt);
  double nvec_norm = nvec_tissue_frame_wrt_camera.norm();
  if (nvec_norm < 0.001) {
    ROS_WARN("specified entrance and exit points are within 1mm; no path will be planned");
    return;
  }
  nvec_tissue_frame_wrt_camera = nvec_tissue_frame_wrt_camera / nvec_norm;
  tvec_tissue_frame_wrt_camera = bvec_tissue_frame_wrt_camera.cross(nvec_tissue_frame_wrt_camera);
  repaired_exit_pt = entrance_pt + nvec_tissue_frame_wrt_camera * dist_entrance_to_exit;
  R_tissue_frame_wrt_camera_frame.col(0) = nvec_tissue_frame_wrt_camera;
  R_tissue_frame_wrt_camera_frame.col(1) = tvec_tissue_frame_wrt_camera;
  R_tissue_frame_wrt_camera_frame.col(2) = bvec_tissue_frame_wrt_camera;
  affine_tissue_frame_wrt_camera_frame.linear() = R_tissue_frame_wrt_camera_frame;
  affine_tissue_frame_wrt_camera_frame.translation() = entrance_pt;
  
  // cout<<"FIXED: affine_tissue_frame_wrt_camera_frame_"<<endl;
  // print_affine(affine_tissue_frame_wrt_camera_frame_);
}

/// TODO add comment
// TODO make sure wrt camera or psm1/2
/// \param gripper_affines_wrt_camera
/// \param ik_ok_array
/// \param ik_score
void NeedlePlanner::compute_needle_drive_gripper_affines(std::vector<Eigen::Affine3d> &gripper_affines_wrt_camera,
                                                         Eigen::VectorXi &ik_ok_array,
                                                         int &ik_score) {
  
  Eigen::Affine3d des_gripper1_wrt_base;
  // pi is the translation part of the des_gripper1_wrt_base
  // r1 and r2 are the gripper x and z directions (vectors)
  // these are necessary to generate a cartesian trajectory.
  Eigen::Vector3d p1, r1, r2;
  Eigen::Vector3d kvec_needle;
  Eigen::Matrix3d Rot_needle;
  Eigen::Matrix3d R0_needle_wrt_tissue;
  // TODO this overrides the varible with the same name defined in the hearder. But why do we need to override?
  
  double dphi = M_PI / (2.0 * (NSAMPS_DRIVE_PLAN - 1));
  phi_insertion = 0.0; //start drive from here
  std::ofstream outfile;
  
  affine_needle_frame_wrt_tissue = affine_init_needle_frame_wrt_tissue;
  // Rotate the needle about the tissue-frame x-axis to tilt the needle bvec
  affine_needle_frame_wrt_tissue.linear() =
          Rotx(psi_needle_axis_tilt_wrt_tissue) * affine_needle_frame_wrt_tissue.linear();
  affine_needle_frame_wrt_tissue.translation() =
          Rotx(psi_needle_axis_tilt_wrt_tissue) * affine_needle_frame_wrt_tissue.translation();
  // TODO why is it that the needle only rotate about tissue x? Because rotatation about other 2 will change entrance/exit.
  
  R0_needle_wrt_tissue = affine_needle_frame_wrt_tissue.linear();
  kvec_needle = affine_needle_frame_wrt_tissue.linear().col(2);
  
  outfile.open("new_trajectory.csp");
  
  double t = 2;
  int nsolns = 0;
  int ik_index = 0;
  ik_ok_array.setZero();
  
  // TODO rename 45
  for (int ipose = 0; ipose < 45; ipose++) {
    Rot_needle = Rot_k_phi(kvec_needle, phi_insertion);
    
    affine_needle_frame_wrt_tissue.linear() = Rot_needle * R0_needle_wrt_tissue;
    
    affine_gripper_frame_wrt_tissue =
            affine_needle_frame_wrt_tissue * affine_needle_frame_wrt_gripper_frame.inverse();
    
    affine_gripper_frame_wrt_camera_frame =
            affine_tissue_frame_wrt_camera_frame * affine_gripper_frame_wrt_tissue;
    
    des_gripper1_wrt_base = default_affine_lcamera_to_psm_one.inverse() * affine_gripper_frame_wrt_camera_frame;
    
    if (ik_solver.ik_solve(des_gripper1_wrt_base)) {
      nsolns++;
      std::cout << ipose << ",";
      gripper_affines_wrt_camera.push_back(affine_gripper_frame_wrt_camera_frame);
      ik_ok_array(ik_index) = 1;
      
    }
    
    ik_index++;
    
    p1 = des_gripper1_wrt_base.translation();
    r1 = des_gripper1_wrt_base.linear().col(0);
    r2 = des_gripper1_wrt_base.linear().col(2);
    
    outfile << p1(0) << ", " << p1(1) << ", " << p1(2) << "," << r1(0) << "," << r1(1) << "," << r1(2) << "," << r2(0)
            << "," << r2(1) << "," << r2(2) << ", 0.2, ";
    outfile << " 0, 0, -0.050, 0, 1, 0, 0, 0, -1,  0, ";
    outfile << t << std::endl;
    
    t += 0.5;
    phi_insertion += dphi;
  }
  
  outfile.close();
  
  bool start_count;
  int score = 0;
  int start_index = 0;
  int stop_index = 0;
  
  // Score
  // 1. Number of points must greater than or equal to 32 (nposes >= 32)
  // 2. No interruption in between points
  // 3. Points near 40 get higher score
  
  if (ik_ok_array.sum() >= 32) {
    
    start_count = false;
    
    for (int array_index = 44; array_index > 0; array_index -= 1) {
      
      if ((ik_ok_array(array_index) == 1) && (start_count == false)) {
        start_count = true;
        start_index = array_index;
      }
      
      if ((ik_ok_array(array_index) == 0) && (start_count == true)) {
        start_count == false;
        stop_index = array_index;
        array_index = 0;
      }
      
      if (start_count == true) score = score + array_index;
      
    }
    
    if ((stop_index) > (start_index - 32))
      score = 0;
    
  }
  
  if ((ik_ok_array.sum() >= 15) && (ik_ok_array.sum() < 32)) {
    score = 120;
  }
  
  ik_score = score;
  std::cout << "Start: " << start_index << std::endl;
  std::cout << "Stop: " << stop_index << std::endl;
  std::cout << "score: " << ik_score << std::endl;
  
}

/// TODO add more commnet
/// Obtains the tf from camera to PSMs and the tf from camera to grippers
/// \param g_affine_lcamera_to_psm_one
/// \param g_affine_lcamera_to_psm_two
/// \param g_psm1_start_pose
/// \param g_psm2_start_pose
void NeedlePlanner::init_poses(Eigen::Affine3d &g_affine_lcamera_to_psm_one,
                               Eigen::Affine3d &g_affine_lcamera_to_psm_two,
                               Eigen::Affine3d &g_psm1_start_pose,
                               Eigen::Affine3d &g_psm2_start_pose) {
  
  tf::TransformListener tfListener;
  tf::StampedTransform tfResult_one, tfResult_two;
  bool tferr = true;
  int ntries = 0;
  
  ROS_INFO_STREAM("Getting transforms from camera to PSMs..");
  ROS_INFO_STREAM("Waiting for tf between base and camera..");
  
  while (tferr) {
    if (ntries > 5) break;
    tferr = false;
    try {
      
      tfListener.lookupTransform("left_camera_optical_frame",
                                 "one_psm_base_link",
                                 ros::Time(0),
                                 tfResult_one);
      
      tfListener.lookupTransform("left_camera_optical_frame",
                                 "two_psm_base_link",
                                 ros::Time(0),
                                 tfResult_two);
      
    } catch (tf::TransformException &exception) {
      ROS_WARN("%s", exception.what());
      tferr = true;
      ros::Duration(0.5).sleep();
      ros::spinOnce();
      ntries++;
    }
  }
  
  if (tferr) {
    
    Eigen::Vector3d nvec, tvec, bvec;
    nvec << -1, 0, 0;
    tvec << 0, 1, 0;
    bvec << 0, 0, -1;
    Eigen::Matrix3d R;
    R.col(0) = nvec;
    R.col(1) = tvec;
    R.col(2) = bvec;
    
    g_affine_lcamera_to_psm_one.translation() << LCAMERA_TO_PSM_ONE_TRANSLATION[0],
            LCAMERA_TO_PSM_ONE_TRANSLATION[1],
            LCAMERA_TO_PSM_ONE_TRANSLATION[2];
    g_affine_lcamera_to_psm_one.linear() = R;
    
    g_affine_lcamera_to_psm_two.translation() << LCAMERA_TO_PSM_TWO_TRANSLATION[0],
            LCAMERA_TO_PSM_TWO_TRANSLATION[1],
            LCAMERA_TO_PSM_TWO_TRANSLATION[2];
    g_affine_lcamera_to_psm_two.linear() = R;
    
    ROS_WARN("using default transform");
    
  } else {
    ROS_INFO("tf is good");
    g_affine_lcamera_to_psm_one = transformTFToEigen(tfResult_one);
    g_affine_lcamera_to_psm_two = transformTFToEigen(tfResult_two);
  }
  
  tferr = true;
  ntries = 0;
  
  ROS_INFO("Waiting for tf between grippers and camera..");
  
  while (tferr) {
    if (ntries > 5) break;
    tferr = false;
    try {
      
      tfListener.lookupTransform("left_camera_optical_frame",
                                 "one_tool_tip_link",
                                 ros::Time(0),
                                 tfResult_one);
      
      tfListener.lookupTransform("left_camera_optical_frame",
                                 "two_tool_tip_link",
                                 ros::Time(0),
                                 tfResult_two);
      
    } catch (tf::TransformException &exception) {
      ROS_WARN("%s", exception.what());
      tferr = true;
      ros::Duration(0.5).sleep();
      ros::spinOnce();
      ntries++;
    }
  }
  
  if (tferr) {
    
    Eigen::Vector3d nvec, tvec, bvec;
    nvec << -1, 0, 0;
    tvec << 0, 1, 0;
    bvec << 0, 0, -1;
    Eigen::Matrix3d R;
    R.col(0) = nvec;
    R.col(1) = tvec;
    R.col(2) = bvec;
    
    g_psm1_start_pose.translation() << -0.02, 0, 0.04;
    g_psm1_start_pose.linear() = R;
    g_psm2_start_pose.translation() << 0.02, 0, 0.04;
    g_psm2_start_pose.linear() = R;
    ROS_WARN("using default start poses");
  } else {
    ROS_INFO("tf is good");
    g_psm1_start_pose = transformTFToEigen(tfResult_one);
    g_psm2_start_pose = transformTFToEigen(tfResult_two);
  }
  
  
}





