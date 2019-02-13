#include <iostream>
#include <vector>
#include <urdf/model.h>
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <math.h>

/**
 * Get the wrist pose for given joint angle configuration.
 * list of 16 values which represent the joint end effector pose obtained from the end-effector transformation matrix
 * using column-major ordering.
 * @param joint_angles list of joint angles to get final end effector pose for
 * @return
 */

urdf::Model getmodel(){
    std::string urdf_file = "src/locobot_description_v3/urdf/locobot_description_v3.urdf";
    urdf::Model model;
    if (!model.initFile(urdf_file)){
      ROS_ERROR("Failed to parse urdf file");
      exit(0);
    }
    ROS_INFO("Successfully parsed urdf file");
    return model;
}

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


std::vector<double> getWristPose(std::vector<double> joint_angles) {
  /**
  * TODO: You can change the signature of this method to pass in other objects, such as the path to the URDF file or a
  * configuration of your URDF file that has been read previously into memory.
  */
   urdf::Model model = getmodel();
   std::vector<double> trans_vec_std;
   joint_angles;
   /* double angle[5] = {0.727,1.073,0.694,-0.244,1.302}; */
   std::string joints[5] = {"joint_1","joint_2","joint_3","joint_4","joint_5"};
   urdf::Joint joint;
   Eigen::MatrixXd trans = Eigen::MatrixXd::Identity(4,4);
   std::vector<Eigen::MatrixXd> trans_vec; 
   for(int i = 0; i < 5; i++){
      joint =  *model.getJoint(joints[i]);
      trans = trans*Transform(joint,joint_angles[i]);
      trans_vec.push_back(trans);
   } 
   std::vector<double> trans_mat(trans.data(), trans.data() + trans.rows() * trans.cols());
   std::cout<<trans<<"\n\n";
   return trans_mat;
}

/**
 * Get the wrist pose for given joint angle configuration.
 * list of 16 values which represent the joint end effector pose obtained from the end-effector transformation matrix
 * using column-major ordering.
 * @param joint_angles list of joint angles to get final end effector pose for
 * @return
 */
std::vector<double> getWristJacobian(std::vector<double> joint_angles) {
  /**
  * TODO: You can change the signature of this method to pass in other objects, such as the path to the URDF file or a
  * configuration of your URDF file that has been read previously into memory.
  */ 
   
   urdf::Model model = getmodel(); 
   std::vector<double> trans_vec_std;
   joint_angles;
   std::string joints[5] = {"joint_1","joint_2","joint_3","joint_4","joint_5"};
   urdf::Joint joint;
   Eigen::MatrixXd trans = Eigen::MatrixXd::Identity(4,4);
   std::vector<Eigen::MatrixXd> trans_vec; 
   for(int i = 0; i < 5; i++){
      joint =  *model.getJoint(joints[i]);
      trans = trans*Transform(joint,joint_angles[i]);
      trans_vec.push_back(trans);
   } 
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
   std::vector<double> jac_mat(Jac.data(), Jac.data() + Jac.rows() * Jac.cols());
   std::cout<<Jac<<"\n\n";
   return jac_mat;
}

int main(int argc, char** argv){
         
    std::cout.precision(3);
    ros::init(argc, argv, "for_kin");
    std::vector<double> angle_ex{0.727,1.073,0.694,-0.244,1.302};
    std::vector<double> angle_1{-0.9501,0.8786,0.513,-1.4157,-0.1997};
    std::vector<double> angle_2{-1.5684,0.08128,-0.4975,0.3102,-1.2215};
    std::vector<double> angle_3{-0.9307,-0.717,-0.6461,0.3086,-0.9533};
    
    std::cout<<"\n"<<"Wrist Pose are:"<<"\n";
    std::vector<double> pose1  = getWristPose(angle_1);
    std::vector<double> pose2  = getWristPose(angle_2);
    std::vector<double> pose3  = getWristPose(angle_3);

    std::cout<<"\n"<<"Wrist Jacobian are:"<<"\n";
    std::vector<double> jac1  = getWristJacobian(angle_1);
    std::vector<double> jac2  = getWristJacobian(angle_2);
    std::vector<double> jac3  = getWristJacobian(angle_3);
    
    return 0;
}
