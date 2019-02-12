#include <urdf/model.h>
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <math.h>

Eigen::MatrixXd axis2rot(urdf::Vector3 axis,double theta){
    Eigen::AngleAxisd angle = Eigen::AngleAxisd(theta,Eigen::Vector3d(axis.x,axis.y,axis.z));
    Eigen::MatrixXd Rot_mat = angle.toRotationMatrix();
    Eigen::MatrixXd Trans_mat = Eigen::MatrixXd::Identity(4,4);
    
    Trans_mat.block(0,0,3,3) = Rot_mat;
    return Trans_mat;
}

Eigen::MatrixXd pose2rot(urdf::Pose pose){
    urdf::Rotation rot = pose.rotation;
    urdf::Vector3 pos = pose.position;
    
    Eigen::MatrixXd Trans_mat = Eigen::MatrixXd::Identity(4,4);
    Eigen::Quaterniond quat = Eigen::Quaterniond(rot.w,rot.x,rot.y,rot.z);
    Eigen::MatrixXd Rot_mat = quat.toRotationMatrix();
    
    Trans_mat.block(0,0,3,3) = Rot_mat;
    Trans_mat.block(0,3,3,1) = Eigen::Vector3d(pos.x,pos.y,pos.z);   
    return Trans_mat;
}

Eigen::MatrixXd Transform(urdf::Joint joint, double theta){
  Eigen::MatrixXd axis = axis2rot(joint.axis,theta);
  Eigen::MatrixXd orig = pose2rot(joint.parent_to_joint_origin_transform);
  Eigen::MatrixXd trans = orig*axis;
  return trans;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "my_parser");
  if (argc != 2){
    ROS_ERROR("Need a urdf file as argument");
    return -1;
  }
  std::string urdf_file = argv[1];

  urdf::Model model;
  if (!model.initFile(urdf_file)){
    ROS_ERROR("Failed to parse urdf file");
    return -1;
  }
  ROS_INFO("Successfully parsed urdf file");
  double angle[5] = {0.727,1.073,0.694,-0.244,1.302};
  std::string joints[5] = {"joint_1","joint_2","joint_3","joint_4","joint_5"};
  urdf::Joint joint;
  Eigen::MatrixXd trans = Eigen::MatrixXd::Identity(4,4);
  std::vector<Eigen::MatrixXd> trans_vec; 
  for(int i = 0; i < 5; i++){
     joint =  *model.getJoint(joints[i]);
     trans = trans*Transform(joint,angle[i]);
     trans_vec.push_back(trans);
  }
  /* std::cout<<trans_vec[4]; */
    
  Eigen::Vector3d diff(3,1),rot_current(3,1),lin_current(3,1);
  Eigen::MatrixXd Jac = Eigen::MatrixXd::Zero(6,5);
  for(int i = 0;i<5;i++){
      joint =  *model.getJoint(joints[i]);
      rot_current = trans_vec[i].block(0,0,3,3)*Eigen::Vector3d(joint.axis.x,joint.axis.y,joint.axis.z);
      diff = trans_vec[4].block(0,3,3,1)-trans_vec[i].block(0,3,3,1);
      lin_current = rot_current.cross(diff);
      Jac.block(0,i,3,1) = lin_current;
      Jac.block(3,i,3,1) = rot_current;
  }
  std::cout<<Jac;
   
  return 0;
}
