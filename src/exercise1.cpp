//
// Created by Jemin Hwangbo on 2022/03/17.
//

#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x

// #include "exercise1_20190673.hpp" //implement in raw for now.
#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include <iostream>

// excercise1_20190673.hpp (cut/paste from here!)
#include <Eigen/Core>
#include <utility>
#include <vector>

using namespace std;
using namespace Eigen;

// Your goal is to write a function that computes the position of the “LH_shank_fixed_LH_FOOT”
// given any joint angles. You can find the description of the robot in
// “resource/anymal_c/urdf/anymal.urdf”. You can find about the URDF convention here:
// http://wiki.ros.org/urdf/XML

// kinematic chain of interest:
// world
  // (implicit floating joint) (gc[0]~gc[6])

// "BASE"
  // base
  // <base_LH_HAA> (fixed) <origin rpy="-2.61799387799 0 -3.14159265359" xyz="-0.2999 0.104 0.0"/>

// "LF BASE"
  // LH_HAA
  // <<LH_HAA>> (revolute) <axis xyz="-1 0 0"/> (gc[13])

// "HIP"
  // LH_HIP
  // <LH_HIP_LH_hip_fixed> (fixed) <origin rpy="-2.61799387799 0 -3.14159265359" xyz="0 0 0"/>
  // LH_hip_fixed
  // <LH_hip_fixed_LH_HFE> (fixed) <origin rpy="0 0 1.57079632679" xyz="-0.0599 0.08381 0.0"/>
  // LH_HFE
  // <<LH_HFE>> (revolute) <axis xyz="1 0 0"/> (gc[14])

// "THIGH"
  // LH_THIGH
  // <LH_THIGH_LH_thigh_fixed> (fixed) <origin rpy="0 0 -1.57079632679" xyz="0 0 0"/>
  // LH_thigh_fixed
  // <LH_thigh_fixed_LH_KFE> (fixed) <origin rpy="0 0 1.57079632679" xyz="-0.0 0.1003 -0.285"/>
  // LH_KFE
  // <<LH_KFE>> (revolute) <axis xyz="1 0 0"/> (gc[15])

// "SHANK"
  // LH_SHANK
  // <LH_SHANK_LH_shank_fixed> (fixed) <origin rpy="0 0 -1.57079632679" xyz="0 0 0"/>
  // LH_shank_fixed
  // <LH_shank_fixed_LH_FOOT> (fixed) <origin rpy="0 0 0" xyz="-0.08795 0.01305 -0.33797"/>
  // LH_FOOT <-- (objective joint origin)

// TODOs
// <CLASS / recursive position implementation for each link>
// LINK!
// Important Parameters:
// parent --> points to parent frame
// CONVENTION: actuated (unfixed joint) --> fixed
// wPos
// wOri
// Important methods:
// constructor
// populate everything except for
// calculateKinematics --> populates framePos / frameRot (vector / rotation matrix)
// get gc as input
//    --> getFramePos / getFrameRot --> gets frame values
//    --> recursively calls parents
// cases
// world: trivial
// base: do quat to rpy
// get
// base: get pos

// independent methods
// rpyToRot
// quatToRot

Eigen::Matrix3d quatToRot(Eigen::Vector4d q){
  // we assume q is normalized and valid (q.norm() > 0)
  // Calculate rotation matrix elements
  double w = q(0);
  double x = q(1);
  double y = q(2);
  double z = q(3);
  double xx = x * x;
  double yy = y * y;
  double zz = z * z;
  double xy = x * y;
  double xz = x * z;
  double xw = x * w;
  double yz = y * z;
  double yw = y * w;
  double zw = z * w;

  Eigen::Matrix3d R; // TODO: simplify
  R << 1 - 2 * (yy + zz), 2 * (xy - zw)    , 2 * (xz + yw)    ,
       2 * (xy + zw)    , 1 - 2 * (xx + zz), 2 * (yz - xw)    ,
       2 * (xz - yw)    , 2 * (yz + xw)    , 1 - 2 * (xx + yy);

  return R;
}

Eigen::Matrix3d rpyToRot(double r,double p,double y){
  Eigen::Matrix3d Rx, Ry, Rz;

  // Individual rotation matrices
  //roll: x axis
  Rx << 1, 0, 0,
    0, cos(r), -sin(r),
    0, sin(r), cos(r);
  // pitch: y axis
  Ry << cos(p), 0, sin(p),
    0, 1, 0,
    -sin(p), 0, cos(p);
  // yaw: z axis
  Rz << cos(y), -sin(y), 0,
    sin(y), cos(y), 0,
    0, 0, 1;

  // Combine rotations (roll, then pitch, then yaw)
  // return Rz * Ry * Rx;
  return Rz * Ry * Rx; // Passive Rotation
}

Eigen::Matrix3d rpyToRot(Eigen::Vector3d rpy){
  return rpyToRot(rpy(0),rpy(1),rpy(2));
}

class Trans {
public:
  char typ; //'f' fixed, 'r' revolute (to be added)
  Eigen::Vector3d originPos;
  Eigen::Matrix<double, 3, 3> originRot;
  Eigen::Vector3d axis;
  int gcIdx; // index of gc to use in kinematics

  Trans(const char t, Eigen::Vector3d xyz, Eigen::Vector3d rpy, Eigen::Vector3d ax, int gcIndex = -1) {
    typ = t;
    originPos = std::move(xyz);
    originRot = rpyToRot(std::move(rpy));
    axis = std::move(ax);
    gcIdx = gcIndex;

    if(typ == 'f'){
      axis << 0,0,0;
      gcIdx = -1;
    }
  }

  // evaluate the full transform for the given generalized coord.
  void evalTrans(const Eigen::VectorXd &gc,Eigen::Vector3d &r,Eigen::Matrix<double,3,3> &R){
    // urdf convention does the "origin move" first, then the rotation.
    if (typ == 'f'){r = originPos;R = originRot;return;}
    else if (typ == 'r'){
      r = originPos;

      Eigen::Vector4d quat; // rotation about an axis can be expressed as quat.
      quat << cos(gc[gcIdx]/2),sin(gc[gcIdx]/2)*(axis/axis.norm());
      R = originRot * quatToRot(quat);
      return;
    }
  }
};
class Link{
public:
  std::string name; //name
  char typ; //'b': base link, 'a': articulated link
  Link* parent; //pointer to parent link
  vector<Trans*> transforms;
  bool calc; // 'true': Kinematics Calculated 'false': Kinematics not calculated yet
  Eigen::Vector3d r; // location of the final transformation (expressed in world frame)
  Eigen::Matrix3d R; // orientation of the final transformation (expressed in world frame)

  Link(const std::string& n,const char t,Link* p){
    name = n;
    typ = t;
    parent = p;
    calc = false;
    r = Eigen::Vector3d::Zero();
    R = Eigen::Matrix3d::Identity();
  }

  Link(const std::string& n,const char t): Link(n,t,nullptr){}

  ~Link() {
    // Delete dynamically allocated memory within the Link object
    for (auto & transform : transforms) {
      delete transform;
    }
    transforms.clear();
  }

  void addTrans(Trans* t){
    transforms.push_back(t);
  }

  void calculateKinematics(const Eigen::VectorXd& gc){
    if(typ == 'b'){ //base case (no parent link, get r / R from base-pos)
      // std::cout<< "base floating body: " << gc.transpose() << std::endl;
      r = gc.segment(0,3);
      R = quatToRot(gc.segment(3,4));

    }
    else if(!parent->calc){
      parent->calculateKinematics(gc);
      r = parent->r;
      R = parent->R;
    } //recursive calls for articulated links

    // traverse and calculate through transforms
    Eigen::Vector3d  tempr;
    Eigen::Matrix<double,3,3> tempR;

    for (auto trans:transforms){
      trans ->evalTrans(gc,tempr,tempR);
      r = r+R*tempr;
      R = R*tempR;
    }

  }

};

// TODO change to Eigen::Vector3d later

Link* getEndEffectorPosition (const Eigen::VectorXd& gc) {

  // all the "hard-coding" is done in this function.

  auto base    = new Link("BASE",'b');
  auto lfBase  = new Link("LF_BASE",'a',base);
  auto lfHip   = new Link("LF_HIP",'a',lfBase);
  auto lfThigh = new Link("LF_THIGH",'a',lfHip);
  auto lfShank = new Link("LF_SHANK",'a',lfThigh);

  Eigen::Vector3d xyz_;
  Eigen::Vector3d rpy_;
  Eigen::Vector3d ax_;

  xyz_ << -0.2999,0.104,0.0;
  rpy_ << -2.61799387799,0,-3.14159265359;
  // rpy_ << -2.61799387799,1.4,-0.69;
  ax_  << 0,0,0;
  base -> addTrans(new Trans('f',xyz_,rpy_,ax_,-1));

  xyz_ << 0,0.0;
  rpy_ << 0,0,0;
  // ax_  << -1,1,3;
  ax_  << -1,0,0;
  base -> addTrans(new Trans('r',xyz_,rpy_,ax_,13));

  base -> calculateKinematics(gc);

  // auto final_pos = base -> r;
  // auto final_ori = base -> R;
  // std::cout << "CALCULATION RESULTS" << std::endl;
  // std::cout << "CALC_POS:" << std::endl << final_pos.transpose() << std::endl;
  // std::cout << "CALC_ORI:" << std::endl << final_ori << std::endl;

  return baseLF;
  // return final_pos;
}

// header file ends here!

int main(int argc, char* argv[]) {
  // create raisim world
  raisim::World world; // physics world
  raisim::RaisimServer server(&world); // visualization server
  // world.addGround(); //we aren't integrating. we don't really need ground

  // anymal
  auto anymal = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) + "/anymal_c/urdf/anymal_TEST.urdf");
  anymal->setName("anymal");
  server.focusOn(anymal);

  // anymal configuration
  // for randomness

  Eigen::VectorXd jointNominalConfig(anymal->getGeneralizedCoordinateDim());

  jointNominalConfig <<
    0.5,  //[ 0]Body Pos z
    0,    //[ 1]Body Pos y
    0.54, //[ 2]Body Pos z
    1.0,  //[ 3]Body Ori w
    0.0,  //[ 4]Body Ori x
    0.0,  //[ 5]Body Ori y
    0.0,  //[ 6]Body Ori z
    -0.03,//[ 7][LF leg] Hip Abduction / Adduction
    0.4,  //[ 8][LF leg] Hip Flexsion / Extension
    -0.8, //[ 9][LF leg] Knee Flexion / Extension
    -0.03,//[10][RF leg]
    0.4,
    -0.8,
    -0.03, //[13][LH leg] (Left "Hind")
    -0.4,
    0.8,
    -0.03,//[16][RH leg] (Right "Hind")
    -0.4,
    0.8
    ;

  std::srand((unsigned int) time(0));
  int gcDims = int(anymal->getGeneralizedCoordinateDim());
  Eigen::VectorXd joint_noise = Eigen::VectorXd::Random(gcDims);
  Eigen::Vector4d quat_raw = Eigen::Vector4d::Random(4);
  Eigen::Vector3d quat_xyz = quat_raw.segment(1,3)/quat_raw.segment(1,3).norm();
  double quat_angle = quat_raw(0)*M_PI;

  Eigen::Vector4d quat_rand;
  quat_rand << cos(quat_angle/2),sin(quat_angle/2)*quat_xyz;
  // std::cout << "random quaternion" << quat_rand.transpose() << " (norm: " << quat_rand.norm() << ")" << std::endl;
  // std::cout << "quat axis" << quat_xyz.transpose() << " (norm: " << quat_xyz.norm() << ")"  << std::endl;
  // std::cout << "quat raw" << quat_raw.transpose() << " (norm: " << quat_raw.norm() << ")"  << std::endl;

  Eigen::VectorXd gcInput = jointNominalConfig+joint_noise;
  gcInput(3) = quat_rand(0);
  gcInput(4) = quat_rand(1);
  gcInput(5) = quat_rand(2);
  gcInput(6) = quat_rand(3);

  // auto gcInput = jointNominalConfig;
  std::cout << "Input gc: " << gcInput.transpose() << std::endl;

  anymal->setGeneralizedCoordinate(gcInput);
  anymal->updateKinematics();

  // debug sphere

  auto debugO = server.addVisualSphere("debug_O", 0.02);
  auto debugX = server.addVisualSphere("debug_X", 0.015);
  auto debugY = server.addVisualSphere("debug_Y", 0.015);
  auto debugZ = server.addVisualSphere("debug_Z", 0.015);

  debugO->setColor(1,1,1,1);
  debugX->setColor(1,0,0,1);
  debugY->setColor(0,1,0,1);
  debugZ->setColor(0,0,1,1);

  // THE SOLUTION FUNCTION
  auto L = getEndEffectorPosition(gcInput);
  Eigen::Vector3d ex{0.2,0,0};
  Eigen::Vector3d ey{0,0.2,0};
  Eigen::Vector3d ez{0,0,0.2};

  Eigen::Vector3d solO = L->r;
  Eigen::Vector3d solX = solO + L->R * ex;
  Eigen::Vector3d solY = solO + L->R * ey;
  Eigen::Vector3d solZ = solO + L->R * ez;

  debugO->setPosition(solO);
  debugX->setPosition(solX);
  debugY->setPosition(solY);
  debugZ->setPosition(solZ);

  auto final_pos = L -> r;
  auto final_ori = L -> R;
  std::cout << "CALCULATION RESULTS" << std::endl;
  std::cout << "CALC_POS:" << std::endl << final_pos.transpose() << std::endl;
  std::cout << "CALC_ORI:" << std::endl << final_ori << std::endl;


  // solution sphere
  // auto target_name = "LH_shank_fixed_LH_FOOT";
  // auto target_name = "base_top_shell";// use "base_top_shell" for base
  // auto target_name = "base_LH_HAA";
  auto target_name = "LH_HAA";

  auto answerSphere = server.addVisualSphere("answer_sphere", 0.04);
  answerSphere->setColor(0,1,0,1);
  raisim::Vec<3> pos;
  raisim::Mat<3,3> ori{0,0,0,0,0,0,0,0,0};
  anymal->getFramePosition(target_name, pos);
  anymal->getFrameOrientation(target_name,ori);
  answerSphere->setPosition(pos.e());

  std::cout << std::endl;
  // status
  std::cout << "ACTUAL [target: " << target_name << "]" << std::endl;
  std::cout << "POS: " << std::endl << pos.e().transpose() << std::endl;
  std::cout << "ORI :" << std::endl << ori << std::endl;

  auto pos_diff = solO - pos.e();
  std::cout << "POS DIFF. : " << pos_diff.transpose() << "( norm: " << pos_diff.norm() << ")" <<std::endl;

  // base
  std::cout << std::endl;
  std::cout << "ADDITIONAL CALCS" << std::endl;
  raisim::Mat<3,3> ori2{0,0,0,0,0,0,0,0,0};
  raisim::quatToRotMat(gcInput.segment(3,4),ori2);
  raisim::Vec<4> quat;
  anymal->getBaseOrientation(quat);
  std::cout << "Converted Base Orientation :" << std::endl << ori2 << std::endl;
  std::cout << "Base Orientation in quat: " << quat.e().transpose() << std::endl;

  // visualization
  server.launchServer();
  for (int i=0; i<2000000; i++)
    std::this_thread::sleep_for(std::chrono::microseconds(1000));

  server.killServer();
}

// Skeleton
// #define _MAKE_STR(x) __MAKE_STR(x)
// #define __MAKE_STR(x) #x
//
// #include "exercise1_20190673.hpp"
// #include "raisim/RaisimServer.hpp"
//
//
// int main(int argc, char* argv[]) {
//   // create raisim world
//   raisim::World world; // physics world
//   raisim::RaisimServer server(&world); // visualization server
//   world.addGround();
//
//   // anymal
//   auto anymal = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) + "/anymal_c/urdf/anymal.urdf");
//   anymal->setName("anymal");
//   server.focusOn(anymal);
//
//   // anymal configuration
//   Eigen::VectorXd jointNominalConfig(anymal->getGeneralizedCoordinateDim());
//   jointNominalConfig << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
//   anymal->setGeneralizedCoordinate(jointNominalConfig);
//   anymal->updateKinematics();
//
//   // debug sphere
//   auto debugSphere = server.addVisualSphere("debug_sphere", 0.02);
//   debugSphere->setColor(1,0,0,1);
//   debugSphere->setPosition(getEndEffectorPosition(jointNominalConfig));
//
//   // solution sphere
//   auto answerSphere = server.addVisualSphere("answer_sphere", 0.04);
//   answerSphere->setColor(0,1,0,1);
//   raisim::Vec<3> pos;
//   anymal->getFramePosition("LH_shank_fixed_LH_FOOT", pos);
//   answerSphere->setPosition(pos.e());
//
//   // visualization
//   server.launchServer();
//   for (int i=0; i<2000000; i++)
//     std::this_thread::sleep_for(std::chrono::microseconds(1000));
//
//   server.killServer();
// }
