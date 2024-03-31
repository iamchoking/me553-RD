//
// Created by Jemin Hwangbo on 2022/03/17.
//

#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x

// #include "exercise1_20190673.hpp" //implement in raw for now.
#include "raisim/RaisimServer.hpp"
#include <iostream>
#include <Eigen/Core>


Eigen::Vector3d getEndEffectorPosition (const Eigen::VectorXd& gc) {
  //////////////////////////
  ///// Your Code Here /////
  //////////////////////////

  return 0.2*Eigen::Vector3d::Ones(); /// replace this
}



int main(int argc, char* argv[]) {
  // create raisim world
  raisim::World world; // physics world
  raisim::RaisimServer server(&world); // visualization server
  // world.addGround(); //we aren't integrating. we don't need ground

  // anymal
  auto anymal = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) + "/anymal_c/urdf/anymal.urdf");
  anymal->setName("anymal");
  server.focusOn(anymal);

  // anymal configuration
  // for randomness
  std::default_random_engine engine(std::time(0));
  std::uniform_real_distribution<double> rand_joint(-1.0,1.0);

  Eigen::VectorXd jointNominalConfig(anymal->getGeneralizedCoordinateDim());
  // TODO: Randomize joint positions

  jointNominalConfig <<
    0.5,    //Body Pos x
    0,    //Body Pos y
    0.54, //Body Pos z
    1.0,  //Body Ori w
    0.0,  //Body Ori x
    0.0,  //Body Ori y
    0.0,  //Body Ori z
    -0.03,//[LF leg] Hip Abduction / Adduction
    0.4,  //[LF leg] Hip Flexsion / Extension
    -0.8, //[LF leg] Knee Flexion / Extension
    -0.03,//[RF leg]
    0.4,
    -0.8,
    0.03, //[LH leg] (Left "Hind")
    -0.4,
    0.8,
    -0.03,//[RH leg] (Right "Hind")
    -0.4,
    0.8
    ;

  // TODO set better seeds
  std::srand((unsigned int) time(0));
  auto jointRandomizedConfig = jointNominalConfig+Eigen::VectorXd::Random(int(anymal->getGeneralizedCoordinateDim()));
  std::cout << "Input Position: " << jointRandomizedConfig.transpose() << std::endl;
  anymal->setGeneralizedCoordinate(jointRandomizedConfig);
  anymal->updateKinematics();

  // debug sphere
  auto debugSphere = server.addVisualSphere("debug_sphere", 0.02);
  debugSphere->setColor(1,0,0,1);
  auto sol_pos = getEndEffectorPosition(jointNominalConfig);
  debugSphere->setPosition(sol_pos);

  // solution sphere
  auto answerSphere = server.addVisualSphere("answer_sphere", 0.04);
  answerSphere->setColor(0,1,0,1);
  raisim::Vec<3> pos;
  anymal->getFramePosition("LH_shank_fixed_LH_FOOT", pos);
  answerSphere->setPosition(pos.e());

  // sat
  std::cout << "Computed Position: " << sol_pos.transpose() << std::endl;
  std::cout << "Actual Position  : " << pos.e().transpose() << std::endl;
  auto pos_diff = sol_pos - pos.e();
  std::cout << "Difference       : " << pos_diff.transpose() << "( norm: " << pos_diff.norm() << ")" <<std::endl;

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
