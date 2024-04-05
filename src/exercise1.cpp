//
// Created by Jemin Hwangbo on 2022/03/17.
//

#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x

#include "exercise1_20190673.hpp"
#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include <iostream>

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
    -0.03,//[ 7][LH leg] Hip Abduction / Adduction
    0.4,  //[ 8][LH leg] Hip Flexion   / Extension
    -0.8, //[ 9][LH leg] Knee Flexion  / Extension
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

  std::srand((unsigned int) time(nullptr));
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

  // SOLUTION VERIFICATION
  auto L = solveLinks(gcInput);
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
  delete L; // prevent memory leak


  // solution sphere
  // auto target_name = "base_top_shell";// use "base_top_shell" for base
  // auto target_name = "base_LH_HAA";
  // auto target_name = "LH_HAA";
  auto target_name = "LH_shank_fixed_LH_FOOT";

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

  std::cout << "this was also tested on VSCode!!!!~" << std::endl;

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
