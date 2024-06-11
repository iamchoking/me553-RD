#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x
#include "raisim/RaisimServer.hpp"
#include "random_coordinates.hpp"

#include "fin-prep_20190673.hpp"
// HEADER START

// HEADER END

/// CHECKING SCRIPT START

bool END_IF_FAIL = false;
bool INIT_RANDOMIZE = true;
bool SUPPLY_GF      = true;
bool PRISMATIC = true;

size_t SIM_STEPS = 2000;

bool analyzeStep(const Eigen::VectorXd& gc, const Eigen::VectorXd& gv, const Eigen::VectorXd& gf, size_t t, raisim::RaisimServer* server, raisim::ArticulatedSystem* anymal){
  std::cout << "STEP[" << t << "]" << std::endl;
  /// TEMPLATE (do some testing here)

  auto r = initRobot();

  r->setState(gc,gv); // consolidated gc,gv into state var.s (ex4)
  r->setForce(gf);
  r->calculateAll();

  auto aCalc = r->udot;

  Eigen::MatrixXd MTrue = anymal->getMassMatrix().e(); // required for other mass-related calculations
  Eigen::MatrixXd bTrue = anymal->getNonlinearities({0,0,-9.81}).e(); // required for other calculations
  Eigen::MatrixXd aTrue = MTrue.inverse() * (gf-bTrue);

  auto inertias = anymal->getInertia();
  auto masses   = anymal->getMass();
  auto coms     = anymal->getBodyCOM_B();
  auto wComs    = anymal->getBodyCOM_W();
  
  auto names    = anymal->getBodyNames();
  auto comWs    = anymal->getBodyCOM_W();

  auto compositeInertias = anymal->getCompositeInertia();
  auto compositeMasses   = anymal->getCompositeMass();
  auto compositeComs     = anymal->getCompositeCOM();

  // for (size_t i = 0; i < inertias.size() ;i++){
  //   std::cout << "RAISIM: [" << i <<"] " << names[i] << std::endl;
  //   std::cout << "m: " << masses[i] << "  com: " << coms[i].e().transpose() << std::endl;
  //   std::cout << inertias[i] <<std::endl;
  // }

  auto err = 0;
  // std::cout << "Error : " << err << std::endl;


  std::cout << "------[SANITY-CHECK]------" << std::endl;

    raisim::Vec<3> rsBase, rsLF, rsRF, rsLH, rsRH, rsDebug;
    rsBase = anymal->getBasePosition();
    anymal->getFramePosition("LF_shank_fixed_LF_FOOT", rsLF);
    anymal->getFramePosition("RF_shank_fixed_RF_FOOT", rsRF);
    anymal->getFramePosition("LH_shank_fixed_LH_FOOT", rsLH);
    anymal->getFramePosition("RH_shank_fixed_RH_FOOT", rsRH);

    raisim::Vec<3> rsvBase, rsvLF, rsvRF, rsvLH, rsvRH, rsvDebug;
    rsvBase = gv.segment(0,3);
    anymal->getFrameVelocity("LF_shank_fixed_LF_FOOT", rsvLF);
    anymal->getFrameVelocity("RF_shank_fixed_RF_FOOT", rsvRF);
    anymal->getFrameVelocity("LH_shank_fixed_LH_FOOT", rsvLH);
    anymal->getFrameVelocity("RH_shank_fixed_RH_FOOT", rsvRH);

    Eigen::Vector3d myBase, myLF, myRF, myLH, myRH, myDebug;
    myBase = r->getPos("base");
    myLF   = r->getPos("LF_FOOT");
    myRF   = r->getPos("RF_FOOT");
    myLH   = r->getPos("LH_FOOT");
    myRH   = r->getPos("RH_FOOT");

    Eigen::Vector3d myvBase, myvLF, myvRF, myvLH, myvRH, myvDebug;
    myvBase = r->getVel("base");
    myvLF   = r->getVel("LF_FOOT");
    myvRF   = r->getVel("RF_FOOT");
    myvLH   = r->getVel("LH_FOOT");
    myvRH   = r->getVel("RH_FOOT");

    // Kinematics related
      // std::cout << "FOOT POSITIONS : " << std::endl;
      // std::cout << "   LF: " << myLF.transpose() << " (err: " << (myLF - rsLF.e()).norm() << ")" << std::endl;
      // std::cout << "   RF: " << myRF.transpose() << " (err: " << (myRF - rsRF.e()).norm() << ")" << std::endl;
      // std::cout << "   LH: " << myLH.transpose() << " (err: " << (myLH - rsLH.e()).norm() << ")" << std::endl;
      // std::cout << "   RH: " << myRH.transpose() << " (err: " << (myRH - rsRH.e()).norm() << ")" << std::endl;

      // anymal->getFramePosition("LH_HAA", rsDebug);
      // std::cout << "[DEBUG] Position: " << rsDebug.e().transpose() << std::endl;


    // Diff. kinematics related
      // std::cout << "Base VELOCITY : " << myvBase.transpose() << " (err: " << (myvBase - rsvBase.e()).norm() << ")" << std::endl;
      // std::cout << "FOOT VELOCITIES : " << std::endl;
      // std::cout << "   LF: " << myvLF.transpose() << " (err: " << (myvLF - rsvLF.e()).norm() << ")" << std::endl;
      // std::cout << "   RF: " << myvRF.transpose() << " (err: " << (myvRF - rsvRF.e()).norm() << ")" << std::endl;
      // std::cout << "   LH: " << myvLH.transpose() << " (err: " << (myvLH - rsvLH.e()).norm() << ")" << std::endl;
      // std::cout << "   RH: " << myvRH.transpose() << " (err: " << (myvRH - rsvRH.e()).norm() << ")" << std::endl;


      // anymal->getFrameVelocity("base_LF_HAA", rsvDebug);
      // myvDebug = r->getLinkByName("LF_HIP")->worldV;
      // std::cout << "Debug VELOCITY (LF_HIP (parent-side)) : " << myvDebug.transpose() << " (err: " << (myvDebug - rsvDebug.e()).transpose() << ")" << std::endl;

      // anymal->getFrameAngularVelocity("base_LF_HAA",rsvDebug);
      // myvDebug = r->getLinkByName("LF_HIP")->worldOm;
      // std::cout << "Debug ANG-VEL (LF_HIP (parent-side)) : " << myvDebug.transpose() << " (err: " << (myvDebug - rsvDebug.e()).transpose() << ")" << std::endl;

      // anymal->getFrameVelocity("LF_HAA", rsvDebug);
      // myvDebug = r->getLinkByName("LF_HIP")->fullV;
      // std::cout << "Debug VELOCITY (LF_HIP (child-side)) : " << myvDebug.transpose() << " (err: " << (myvDebug - rsvDebug.e()).transpose() << ")" << std::endl;
      // std::cout << "(raisim says " << rsvDebug.e().transpose() << " )" << std::endl;

      // anymal->getFrameAngularVelocity("LF_HAA",rsvDebug);
      // myvDebug = r->getLinkByName("LF_HIP")->fullOm;
      // std::cout << "Debug ANG-VEL (LF_HIP (child-side)) : " << myvDebug.transpose() << " (err: " << (myvDebug - rsvDebug.e()).transpose() << ")" << std::endl;

    // Composite Mass Related
      // std::cout << "ROBOT COM (MINE)    : " << r->getLinkByName("base")->compI.com.originPos.transpose() << std::endl;
      // std::cout << "ROBOT COM (RAISIM)  : " << compositeComs[0].e().transpose() << std::endl;
      // std::cout<<std::endl;

    // Nonlinear term related
      std::cout << "Nonlinear Term (MINE)   : " << r->b.transpose() << std::endl;
      std::cout << "Nonlinear Term (RAISIM) : " << bTrue.transpose() << std::endl;
      std::cout << "Difference : " << (r->b - bTrue).transpose() << std::endl;
      std::cout << std::endl;

      raisim::Vec<3> rsaBase, rsaLF, rsaRF, rsaLH, rsaRH, rsaDebug;
      // anymal->getFrameAcceleration("base_to_base_inertia"  , rsaBase);
      anymal->getFrameAcceleration("base_to_base_inertia",rsaBase);
      anymal->getFrameAcceleration("LF_shank_fixed_LF_FOOT", rsaLF  );
      anymal->getFrameAcceleration("RF_shank_fixed_RF_FOOT", rsaRF  );
      anymal->getFrameAcceleration("LH_shank_fixed_LH_FOOT", rsaLH  );
      anymal->getFrameAcceleration("RH_shank_fixed_RH_FOOT", rsaRH  );

      // this is only useful in the first step, and gf is set to zero!
      // std::cout << "[[[Accelerations]]]" << std::endl;
      // std::cout << "input force: " << anymal->getGeneralizedForce().e().transpose() << std::endl;
      // std::cout << "raisim says base acc is: " << rsaBase.e().transpose() << std::endl;

      // anymal->getFrameAcceleration("base_LF_HAA",rsaDebug);
      // std::cout << "raisim says base_LF_HAA acc is  : " << rsaDebug.e().transpose()   << std::endl;
      // anymal->getFrameAcceleration("LF_HAA",rsaDebug);
      // std::cout << "raisim says LF_HAA acc is  : " << rsaDebug.e().transpose()   << std::endl;
      // std::cout << "raisim says LF_FOOT acc is  : " << rsaLF.e().transpose()   << std::endl;

      // anymal->getFrameAcceleration("base_RH_HAA",rsaDebug);
      // std::cout << "raisim says base_RH_HAA acc is  : " << rsaDebug.e().transpose()   << std::endl;
      // anymal->getFrameAcceleration("RH_HAA",rsaDebug);
      // std::cout << "raisim says RH_HAA acc is  : " << rsaDebug.e().transpose()   << std::endl;

      // std::cout << "raisim says LF_FOOT acc is  : " << rsaLF.e().transpose()   << std::endl;
      // std::cout << "raisim says RF_FOOT acc is  : " << rsaRF.e().transpose()   << std::endl;
      // std::cout << "raisim says LH_FOOT acc is  : " << rsaLH.e().transpose()   << std::endl;
      // std::cout << "raisim says RH_FOOT acc is  : " << rsaRH.e().transpose()   << std::endl;

      // std::cout << std::endl;

      // anymal->getFrameAcceleration("RH_KFE",rsaDebug);
      // std::cout << "raisim says RH shank acc is  : " << rsaDebug.e().transpose()   << std::endl;

    // acceleration (ABA) related
      // std::cout << "------ COMPUTED ACCELERATION ------" << std::endl;
      // std::cout << "Input gf: " << gf.transpose() << std::endl << std::endl;
      // std::cout << "MINE" << std::endl;
      // // std::cout << "(easy)" << aCalc.transpose() << std::endl;
      // std::cout << aCalc.transpose() << std::endl;

      // std::cout << "RAISIM" << std::endl;
      // std::cout << aTrue.transpose() << std::endl;

      // // std::cout << "Error : " << (bCalc - bTrue).block<12,12>(6,6).norm() << std::endl;

    // setting relevant check-visuals
      server->getVisualObject("p0_mine")->setPosition(myBase);
      server->getVisualObject("p1_mine")->setPosition(myLF);
      server->getVisualObject("p2_mine")->setPosition(myRF);
      server->getVisualObject("p3_mine")->setPosition(myLH);
      server->getVisualObject("p4_mine")->setPosition(myRH);

      double vScale = 0.1;
      server->getVisualObject("v0_mine")->setPosition(myBase);
      server->getVisualObject("v0_mine")->setOrientation(quatPointing(myvBase));
      server->getVisualObject("v0_mine")->setCylinderSize(0.1,vScale*(myvBase.norm()));

      server->getVisualObject("v1_mine")->setPosition(myLF);
      server->getVisualObject("v1_mine")->setOrientation(quatPointing(myvLF - myvBase));
      server->getVisualObject("v1_mine")->setCylinderSize(0.1,vScale*((myvLF - myvBase).norm()));
      server->getVisualObject("v2_mine")->setPosition(myRF);
      server->getVisualObject("v2_mine")->setOrientation(quatPointing(myvRF - myvBase));
      server->getVisualObject("v2_mine")->setCylinderSize(0.1,vScale*((myvRF - myvBase).norm()));
      server->getVisualObject("v3_mine")->setPosition(myLH);
      server->getVisualObject("v3_mine")->setOrientation(quatPointing(myvLH - myvBase));
      server->getVisualObject("v3_mine")->setCylinderSize(0.1,vScale*((myvLH - myvBase).norm()));
      server->getVisualObject("v4_mine")->setPosition(myRH);
      server->getVisualObject("v4_mine")->setOrientation(quatPointing(myvRH - myvBase));
      server->getVisualObject("v4_mine")->setCylinderSize(0.1,vScale*((myvRH - myvBase).norm()));
    

    // Sanity Check essentials
    double ePos    = (myLF - rsLF.e()).norm()+(myRF - rsRF.e()).norm()+(myLH - rsLH.e()).norm()+(myRH - rsRH.e()).norm();
    double eVel    = (myvLF - rsvLF.e()).norm()+(myvRF - rsvRF.e()).norm()+(myvLH - rsvLH.e()).norm()+(myvRH - rsvRH.e()).norm();
    double eMass   = (MTrue - r->M).norm();
    double eNonlin = (bTrue - r->b).norm();
    double eAcc    = (aTrue - r->udot).norm();
    std::cout << "FOOT POSITIONS ERROR : "<< ePos    << std::endl;
    std::cout << "FOOT VELOCITIES ERROR: "<< eVel    << std::endl;
    std::cout << "MASS MATRIX ERROR    : "<< eMass   << std::endl;
    std::cout << "NONLINEARITY ERROR   : "<< eNonlin << std::endl;
    std::cout << "ACCELERATION ERROR   : "<< eAcc    << std::endl;

    if(ePos + eVel + eMass + eNonlin + eAcc < 1e-10){std::cout << std::endl;}
    else{std::cout << "!!SANITY CHECK FAILED!!" << std::endl;}


  /// Return condition
  if(err >= 1e-9){std::cout << "STEP FAILED" << std::endl;}
  return (err < 1e-9);
}

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  // create raisim world
  raisim::World world; // physics world
  raisim::RaisimServer server(&world);

  raisim::ArticulatedSystem* anymal;

  if(PRISMATIC){
    anymal = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) + "/anymal_c/urdf/anymal_prismatic.urdf");
  }
  else{
    anymal = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) + "/anymal_c/urdf/anymal.urdf"); //vanilla urdf
  }

  anymal -> setComputeInverseDynamics(true); // required to get ground truth in accleleration

  world.addGround();
  world.setTimeStep(0.001);

  // some position spheres
  // auto p1RAISIM = server.addVisualSphere("p1_raisim");
  auto p0MINE   = server.addVisualSphere("p0_mine", 0.08);
  auto p1MINE   = server.addVisualSphere("p1_mine", 0.03);
  auto p2MINE   = server.addVisualSphere("p2_mine", 0.03);
  auto p3MINE   = server.addVisualSphere("p3_mine", 0.03);
  auto p4MINE   = server.addVisualSphere("p4_mine", 0.03);
  p0MINE -> setColor(.1,.1,.1,1);
  p1MINE -> setColor(.5,.5,.5,1);
  p2MINE -> setColor(.5,.5,.5,1);
  p3MINE -> setColor(.5,.5,.5,1);
  p4MINE -> setColor(.5,.5,.5,1);

  // some velocity arrows
  auto v0MINE   = server.addVisualArrow("v0_mine", 0.1, 0.5, 1,1,1,1);
  auto v1MINE   = server.addVisualArrow("v1_mine", 0.1, 0.5, 1,1,1,1);
  auto v2MINE   = server.addVisualArrow("v2_mine", 0.1, 0.5, 1,1,1,1);
  auto v3MINE   = server.addVisualArrow("v3_mine", 0.1, 0.5, 1,1,1,1);
  auto v4MINE   = server.addVisualArrow("v4_mine", 0.1, 0.5, 1,1,1,1);

  // debug sphere
  auto debugRAISIM  = server.addVisualSphere("debug_raisim", 0.05);
  auto debugMINE    = server.addVisualSphere("debug_mine", 0.06); //when coincident, the sphere looks green
  debugRAISIM ->setColor(1,0,0,1);
  debugMINE   ->setColor(0,1,0,1);

  Eigen::VectorXd gc(anymal->getGeneralizedCoordinateDim()), gv(anymal->getDOF()), gf(anymal->getDOF());
  gc << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  gv << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8;
  // gv << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.0, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8;

  gf << 0.15, 0.21, 0.36, 0.24, 0.35, 0.46, 0.57, 0.18, 0.29, 1.0, 1.1, 1.5, 1.1, 1.2, 1.3, 1.6, 1.7, 1.8;

  if(INIT_RANDOMIZE){
    utils::gcRandomize(gc);
    gc[2] = gc[2] + 3;
    utils::gvRandomize(gv,15);
    utils::gvRandomize(gf,10);
  }

  anymal->setState(gc, gv);
  if(SUPPLY_GF){
    anymal->setGeneralizedForce(gf);
  }

  server.launchServer();
  server.focusOn(anymal);

  /// if you are using an old version of Raisim, you need this line
  world.integrate1();
  bool correct = analyzeStep(gc,gv,gf,0,&server,anymal);

  for (int sec=5; sec>0; sec--){
    std::cout << "Dropping in [" << sec << "]..." << std::endl;
    raisim::USLEEP(1000000);
  }
  std::cout << "DROP!" << std::endl;

  // std::cout<<"mass matrix should be \n"<< anymal->getMassMatrix().e()<<std::endl;
  for (size_t i = 0; i<SIM_STEPS; i++){
    RS_TIMED_LOOP(world.getTimeStep()*2e6)
    if(i%10 == 0){
      correct = analyzeStep(gc,gv,gf,i,&server,anymal) && correct;
      if(!correct && END_IF_FAIL){break;}

      utils::gvRandomize(gf,0.5);
    }

    server.integrateWorldThreadSafe();

    anymal->getState(gc, gv);
  }

  server.killServer();

  if(correct) {
    std::cout<<"TEST PASSED"<<std::endl;
  } else {
    std::cout<<"TEST FAILED"<<std::endl;
  }

  return 0;
}

