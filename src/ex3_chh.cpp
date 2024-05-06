//
// Created by Jemin Hwangbo on 2022/04/08.
//

#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x
#include "raisim/RaisimServer.hpp"
#include "random_coordinates.hpp"

// #include "exercise_3_20190673.hpp"
// START HEADER FILE
// Sanga saved my life
/// do not change the name of the method

#include <Eigen/Core>
#include <utility>
#include <vector>
#include <iostream>

// SETUP
// Kinematic chain (of interest):
// (world)
// (implicit floating joint) (gc[0]~gc[6])
//
// "BASE"
// base
// <base_LH_HAA> (fixed) <origin rpy="-2.61799387799 0 -3.14159265359" xyz="-0.2999 0.104 0.0"/>
//
// "LH_BASE"
// LH_HAA
// <<LH_HAA>> (revolute) <axis xyz="-1 0 0"/> (gc[13])
//
// "HIP"
// LH_HIP
// <LH_HIP_LH_hip_fixed> (fixed) <origin rpy="-2.61799387799 0 -3.14159265359" xyz="0 0 0"/>
// LH_hip_fixed
// <LH_hip_fixed_LH_HFE> (fixed) <origin rpy="0 0 1.57079632679" xyz="-0.0599 0.08381 0.0"/>
// LH_HFE
// <<LH_HFE>> (revolute) <axis xyz="1 0 0"/> (gc[14])
//
// "THIGH"
// LH_THIGH
// <LH_THIGH_LH_thigh_fixed> (fixed) <origin rpy="0 0 -1.57079632679" xyz="0 0 0"/>
// LH_thigh_fixed
// <LH_thigh_fixed_LH_KFE> (fixed) <origin rpy="0 0 1.57079632679" xyz="-0.0 0.1003 -0.285"/>
// LH_KFE
// <<LH_KFE>> (revolute) <axis xyz="1 0 0"/> (gc[15])
//
// "SHANK"
// LH_SHANK
// <LH_SHANK_LH_shank_fixed> (fixed) <origin rpy="0 0 -1.57079632679" xyz="0 0 0"/>
// LH_shank_fixed
// <LH_shank_fixed_LH_FOOT> (fixed) <origin rpy="0 0 0" xyz="-0.08795 0.01305 -0.33797"/>
// LH_FOOT <-- (objective joint origin)

Eigen::Matrix3d quatToRot(Eigen::Vector4d q){
  // ***we assume q is normalized and valid (q.norm() > 0)
  // from formula
  Eigen::Matrix3d R;
  R << 1 - 2 * (q(2)*q(2) + q(3)*q(3)), 2 * (q(1)*q(2) - q(3)*q(0))    , 2 * (q(1)*q(3) + q(2)*q(0))    ,
    2 * (q(1)*q(2) + q(3)*q(0))    , 1 - 2 * (q(1)*q(1) + q(3)*q(3)), 2 * (q(2)*q(3) - q(1)*q(0))    ,
    2 * (q(1)*q(3) - q(2)*q(0))    , 2 * (q(2)*q(3) + q(1)*q(0))    , 1 - 2 * (q(1)*q(1) + q(2)*q(2));
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
  return Rz * Ry * Rx; // Passive Rotation + urdf convention
}

Eigen::Matrix3d rpyToRot(Eigen::Vector3d rpy){
  return rpyToRot(rpy(0),rpy(1),rpy(2));
}

Eigen::Matrix3d skew3d(const Eigen::Vector3d& w){
  Eigen::Matrix3d R;
  R <<   0 ,-w[2], w[1],
    w[2],   0 ,-w[0],
    -w[1], w[0],   0 ;
  // just one typo here set me back 3 hours... :(
  return R;
}

Eigen::Matrix3d hcInertia(double ixx, double ixy, double ixz, double iyy, double iyz, double izz){
  Eigen::Matrix3d I;
  I << ixx,ixy,ixz,
    ixy,iyy,iyz,
    ixz,iyz,izz
    ;
  return I;
}

Eigen::MatrixXd hc6x6(double m,double ixx, double ixy, double ixz, double iyy, double iyz, double izz){
  Eigen::MatrixXd I;
  I.resize(6,6);
  I.setZero();
  I.block<3,3>(0,0) = m * Eigen::Matrix3d::Identity();
  I.block<3,3>(3,3) = hcInertia(ixx,ixy,ixz,iyy,iyz,izz);

  return I;
}

class Trans {
public:
  char typ; //'f' fixed, 'r' revolute (to be added: 'p' prismatic)
  Eigen::Vector3d originPos;
  Eigen::Matrix3d originRot;
  Eigen::Vector3d axis;
  int gcIdx; // index of gc to use in kinematics
  int gvIdx; // index of gv to use in diff. kinematics (usually gcIdx-1)

  Trans(const char t, Eigen::Vector3d xyz, Eigen::Vector3d rpy, Eigen::Vector3d ax, int gcIndex = -1, int gvIndex = -1) {
    typ = t;
    originPos = std::move(xyz);
    originRot = rpyToRot(std::move(rpy));
    axis = std::move(ax);
    gcIdx = gcIndex;
    gvIdx = gvIndex;

    if(typ == 'f'){
      initFixed();
    }
  }

  Trans(Eigen::Vector3d r, Eigen::Matrix3d R){ //simple creation (result of evalTrans)
    initFixed();
    originPos = r;
    originRot = R;
  }

  Trans(){ // trivial trans
    initFixed();
    originPos << 0,0,0;
    originRot = Eigen::Matrix3d::Identity();
  }

  // "set" functions for streamlining hard-coding
  void setXyz (double x,double y,double z){originPos << x,y,z;}
  void setRpy (double r,double p,double y){originRot = rpyToRot(r,p,y);}
  void setAxis(double ax,double ay,double az){axis << ax,ay,az;}
  void setTyp (char newTyp = 'f'){typ=newTyp;}
  void setIdx (int gcIndex = -1,int gvIndex = -1){gcIdx = gcIndex;gvIdx = gvIndex;}

  // axis given as an array
  void setProfile(double* xyz,double* rpy,char newTyp = 'f',double* ax3d = nullptr,int gcIndex = -1,int gvIndex = -1){
    setXyz (xyz[1] ,xyz[2] ,xyz[3] );
    setRpy (rpy[1] ,rpy[2] ,rpy[3] );
    setAxis(ax3d[1],ax3d[2],ax3d[3]);
    setTyp(newTyp);
    setIdx(gcIndex,gvIndex);
  }

  // axis given as 3 numbers
  void setProfile(double x,double y,double z, double R,double P,double Y,char newTyp = 'f',double ax=0,double ay=0,double az=0,int gcIndex = -1,int gvIndex = -1){
    setXyz (x,y,z);
    setRpy (R,P,Y);
    setAxis(ax,ay,az);
    setTyp(newTyp);
    setIdx(gcIndex,gvIndex);
    if(typ == 'f'){initFixed();}
  }

  void initFixed(){ // pattern for fixed transformations
    typ = 'f';
    gcIdx = -1;
    gvIdx = -1;
    axis << 0,0,0;
  }

  void attachTrans(const Trans& newT){ //attach transformation to the end of this one (modifier)
    if(typ != 'f'){ //error: middle actuation
      throw std::invalid_argument("middle-actuation detected!");
    }
    originPos = originPos + originRot*newT.originPos;
    originRot = originRot*newT.originRot;
    if(newT.typ != 'f'){
      // if(typ != 'f'){ //error: double actuation! (this is actually impossible)
      //   throw std::invalid_argument("double-actuation detected!");
      // }
      typ = newT.typ;
      axis = newT.axis;
      gcIdx = newT.gcIdx;
      gvIdx = newT.gvIdx;
    }
  }

  // evaluate the full transform for the given generalized coord (pure function).
  // returns a fixed transformation
  Trans* evalTrans(const Eigen::VectorXd &gc){
    // urdf convention does the "origin move" first, then the actuation wrt axis.
    Eigen::Vector3d newPos = originPos;
    Eigen::Matrix3d newRot = originRot;


    if(typ == 'r'){
      Eigen::Vector4d quat; // rotation about an axis can be expressed as quat.
      quat << cos(gc[gcIdx]/2),sin(gc[gcIdx]/2)*(axis/axis.norm());
      newRot = originRot * quatToRot(quat);
    }
    else if(typ == 'p'){ //UNTESTED
      newPos = originPos + gc[gcIdx] * (originRot*axis);
    }
    return new Trans(newPos,newRot);
  }
};

//TODO: throw warning/error if an articulated link does not have a gcindex

class Link{
public:
  std::string name; //name
  char typ; //'b': base link, 'a': articulated link, 'e': end-effector (no actuation)
  Link* parent; //pointer to parent link
  std::vector<Link*> children; //pointer(s) to children link

  // ex2: !! changed this from "final" to "root" references!!

  Trans bodyT; // full transform from (parent^2<>parent) to (parent<>this) (r,R,p,gcIdx,gvIdx) ("assume parent^2 is fixed to world")
  // transform order (urdf convention) r -> R -> p
  // translate by r (expressed in (p^2<>p) frame)
  // rotate by R (expressed in (p^2<>p) frame)
  // actuate wrt p by (gc[gcIdx]) (expressed in new frame(r -> R))

  // state variable (another transform)
  Trans worldT;  // transform from world to i (world --> "root" of this link)

  // flags
  bool calcKin;  // pass 1: kinematics calculated (root->leaf)
  bool calcComp; // pass 2: composite inertia calculated (leaf -> root)
  //*: non-constant

  Link(const std::string& n,const char linkTyp){
    name = n;
    typ = linkTyp;
    calcKin = false; // pass 1: kinematics calculated (root->leaf)
    calcComp = false; // pass 2: composite inertia calculated (leaf -> root)

    bodyT  = *(new Trans());
    worldT = *(new Trans());
  }

  void addTrans(const Trans& newT){
    // add an additional transformation at the end of the link
    // only modifies constant properties
    bodyT.attachTrans(newT);
  }

  Trans* propLinkKin(const Eigen::VectorXd& gc){ //returns frame for (i') frame (instead of (i))
    return worldT.evalTrans(gc);
  }

  void calcLinkKin(const Eigen::VectorXd& gc){
    if(typ == 'b'){ //base case (no parent link, get wr / wR from base-pos)
      // std::cout<< "base floating body: " << gc.transpose() << std::endl;
      worldT.originPos = gc.segment(0,3);
      worldT.originRot = quatToRot(gc.segment(3,4));
      calcKin = true;
      return;
    }
    else if(!parent->calcKin){
      throw(std::invalid_argument("Parent Kinematics Not Calculated!"));
    }
    // std::cout << "parent pos" << worldT.originPos << std::endl;
    // std::cout << "parent ori" << worldT.originRot << std::endl;
    // std::cout << "parent typ" << worldT.typ << std::endl;

    worldT = *(parent->propLinkKin(gc));
    worldT.attachTrans(bodyT); // just attach my own transform!

    // although each links have to keep axis information (for J), the output for kinematics must be evaluated!
    calcKin = true;
    // return;
  }

  /// add influence of this link to the provided Jp
  void augmentJp(Eigen::MatrixXd& Jp,const Eigen::VectorXd& gc, const Eigen::VectorXd& wree){
    // wree: the position of end effector (point of question) in world coordinates.
    if (!calcKin){throw(std::invalid_argument("This link's kinematics is not calculated!"));}

    if(typ == 'b'){
      Jp.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
      Jp.block<3,3>(0,3) = -skew3d(wree-worldT.originPos);
      // std::cout << "computed arm : " << (wree-worldT.originPos).transpose() << std::endl;
      // std::cout << "BASE vJ" << std::endl << vJ << std::endl;
    }
    else if(typ == 'a' || typ == 'e'){
      if (worldT.gvIdx >= 0){
        if (worldT.typ == 'r'){
          Jp.block<3,1>(0,worldT.gvIdx) = skew3d(worldT.originRot*worldT.axis)*(wree-worldT.originPos);
          // std::cout << "computed axis: " << (worldT.originRot*worldT.axis).transpose() << std::endl;
          // std::cout << "computed arm : " << (wree-worldT.originPos).transpose() << std::endl;
          // std::cout << "computed block: " << vJ.block<3,1>(0,worldT.gvIdx).transpose() << std::endl;
        } //TODO: prismatic jacobian
      }

      // vJ.block<3,1>(0,worldT.gvIdx) = skew3d(worldT.axis)*(wree-worldT.originPos);
    }
  }

  void augmentJa(Eigen::MatrixXd Ja,const Eigen::VectorXd& gc, const Eigen::VectorXd& wree){
    // wree: the position of end effector (point of question) in world coordinates.
    if (!calcKin){throw(std::invalid_argument("This link's kinematics is not calculated!"));}

    if(typ == 'b'){
      // wJ.block<3,3>(0,0) = Eigen::Matrix3d::Zero()); //redundant
      Ja.block<3,3>(0,3) = Eigen::Matrix3d::Identity();
    }
    else if(typ == 'a' || typ == 'e'){
      if (worldT.gvIdx >= 0){
        if (worldT.typ == 'r'){
          Ja.block<3,1>(0,worldT.gvIdx) = worldT.originRot*worldT.axis;
        } //TODO: prismatic jacobian
      }

      // Ja.block<3,1>(0,worldT.gvIdx) = skew3d(worldT.axis)*(wree-worldT.originPos);
    }
  }

};

class Robot{
public:
  std::vector<Link*> links; //follows "parent-first" convention
  size_t gvDim; // (==dof)
  size_t dof;
  size_t gcDim;

  // flags
  bool calcKin;

  Link* root;

  Robot(size_t gcDimensions,size_t gvDimensions){
    gcDim = gcDimensions;
    gvDim = gvDimensions;
    dof = int(gvDimensions);
    root = nullptr;
    calcKin = false;
  }

  int findLinkIdx(Link* l){
    auto it = std::find(links.begin(),links.end(),l);
    if (it == links.end()){
      return -1;
    }
    else{
      return int(std::distance(links.begin(),it));
    }
  }

  int findLinkIdx(std::string n){
    for(size_t i = 0;i<links.size();i++){
      if(links[i]->name == n){return int(i);}
    }
    return -1;
  }

  void addLink(Link* l,Link* p = nullptr){
    // validity check
    if ( ( p == nullptr ) && ( !links.empty() ) ){throw std::invalid_argument("double-root detected!");}
    else if( (p) && findLinkIdx(p) == -1){throw std::invalid_argument("parent not found!");}
    links.push_back(l);
    l->parent = p;
    if( p != nullptr){p->children.push_back(l);}
    else{root=l;}

    // std::cout << "added link " << l->name << std::endl;
  }

  // important stuff
  // all of the following functions assume a "parent-first" indexing of [links]

  void calculateKinematics(const Eigen::VectorXd& gc){
    /// TODO move the link version here (root -> leaf)
    if(calcKin){return;}
    for(auto l:links){
      l->calcLinkKin(gc);
    }
    calcKin = true;
  }

  [[maybe_unused]] void resetKinematics(){
    calcKin = false;
    for(auto l:links){
      l -> calcKin = false;
    }
  }

  /// Positional Jacobian (modifier)
  void calculateJp(Eigen::MatrixXd& Jp,const Eigen::VectorXd& gc,const std::string& linkName,const Eigen::Vector3d &wree){
    // initialize
    Jp.resize(3,long(gvDim));
    Jp.setZero();

    Link* l = links[findLinkIdx(linkName)];
    while(l->parent != nullptr){
      l->augmentJp(Jp,gc,wree);
      l = l->parent;
    }
    //final time for base
    l->augmentJp(Jp,gc,wree);
  }

  /// Angular Jacobian
  void calculateJa(Eigen::MatrixXd& Ja,const Eigen::VectorXd& gc,const std::string& linkName,const Eigen::Vector3d &wree){
    // initialize
    Ja.resize(3,long(gvDim));
    Ja.setZero();

    Link* l = links[findLinkIdx(linkName)];
    while(l->parent != nullptr){
      l->augmentJa(Ja,gc,wree);
      l = l->parent;
    }
    //final time for base
    l->augmentJa(Ja,gc,wree);
  }

  [[maybe_unused]] void calculateJ(Eigen::MatrixXd& J,const Eigen::VectorXd& gc,const std::string& linkName,const Eigen::Vector3d &wree){
    J.resize(6,long(gvDim));
    J.setZero();

    Eigen::MatrixXd Jp;
    Eigen::MatrixXd Ja;
    calculateJp(Jp,gc,linkName,wree);
    calculateJa(Ja,gc,linkName,wree);

    J << Jp,Ja;
  }


  Trans* getTrans(const std::string &linkName){
    auto l = links[findLinkIdx(linkName)];
    if(!(l->calcKin)){throw(std::invalid_argument("Link Kinematics not yet calculated!"));}
    return &(l->worldT);
  }

  Eigen::Vector3d getPos(const Eigen::VectorXd &gc,const std::string &linkName){
    calculateKinematics(gc);
    return getTrans(linkName)->originPos;
  }

};

Robot* initRobot() {
  // all the "hard-coding" is done in this function.
  auto robot = new Robot(19,18);

  Trans tempT = *(new Trans());

  // note: though this can be further simplified, this is left as-is to mimic the workflow in [anymal.urdf].
  // "BASE"
  auto base    = new Link("BASE",'b');
  robot->addLink(base);
  // base

  // "HIP"
  auto lhHip = new Link("LH_HIP",'a');
  robot->addLink(lhHip,base);
  // base
  // <base_LH_HAA> (fixed) <origin rpy="-2.61799387799 0 -3.14159265359" xyz="-0.2999 0.104 0.0"/>
  // order: x y z   R P Y  (type = f) (ax ay az) (gcIdx gvIdx)
  tempT.setProfile(-0.2999,0.104,0.0,  -2.61799387799,0,-3.14159265359);
  lhHip -> addTrans(tempT);
  // LH_HAA

  // <<LH_HAA>> (revolute) <axis xyz="-1 0 0"/> (gc[13])
  tempT.setProfile(0.0,0.0,0.0,  0.0,0.0,0.0, 'r', -1,0.0,0.0, 13,12);
  lhHip -> addTrans(tempT);
  // LH_HIP

  // "THIGH"
  auto lhThi   = new Link("LH_THIGH",'a');
  robot->addLink(lhThi,lhHip);
  // LH_HIP
  // <LH_HIP_LH_hip_fixed> (fixed) <origin rpy="-2.61799387799 0 -3.14159265359" xyz="0 0 0"/>
  tempT.setProfile(0.0,0.0,0.0,  -2.61799387799,0.0,-3.14159265359);
  lhThi -> addTrans(tempT);
  // LH_hip_fixed
  // <LH_hip_fixed_LH_HFE> (fixed) <origin rpy="0 0 1.57079632679" xyz="-0.0599 0.08381 0.0"/>
  tempT.setProfile(-0.0599,0.08381,0.0,  0.0,0.0,1.57079632679);
  lhThi -> addTrans(tempT);
  // LH_HFE
  // <<LH_HFE>> (revolute) <axis xyz="1 0 0"/> (gc[14])
  tempT.setProfile(0.0,0.0,0.0,  0.0,0.0,0.0, 'r', 1,0.0,0.0, 14,13);
  lhThi -> addTrans(tempT);

  // "SHANK"
  auto lhSha = new Link("LH_SHANK",'a');
  robot->addLink(lhSha,lhThi);
  // LH_THIGH
  // <LH_THIGH_LH_thigh_fixed> (fixed) <origin rpy="0 0 -1.57079632679" xyz="0 0 0"/>
  tempT.setProfile(0.0,0.0,0.0,  0.0,0.0,-1.57079632679);
  lhSha -> addTrans(tempT);
  // LH_thigh_fixed
  // <LH_thigh_fixed_LH_KFE> (fixed) <origin rpy="0 0 1.57079632679" xyz="-0.0 0.1003 -0.285"/>
  tempT.setProfile(0.0,0.1003,-0.285,  0.0,0.0,1.57079632679);
  lhSha -> addTrans(tempT);
  // LH_KFE
  // <<LH_KFE>> (revolute) <axis xyz="1 0 0"/> (gc[15])
  tempT.setProfile(0.0,0.0,0.0,  0.0,0.0,0.0, 'r', 1,0.0,0.0, 15,14);
  lhSha -> addTrans(tempT);

  // "FOOT"
  auto lhFoot = new Link("LH_FOOT",'e');
  robot->addLink(lhFoot,lhSha);
  // LH_SHANK
  // <LH_SHANK_LH_shank_fixed> (fixed) <origin rpy="0 0 -1.57079632679" xyz="0 0 0"/>
  tempT.setProfile(0.0,0.0,0.0,  0.0,0.0,-1.57079632679);
  lhFoot -> addTrans(tempT);
  // LH_shank_fixed
  // <LH_shank_fixed_LH_FOOT> (fixed) <origin rpy="0 0 0" xyz="-0.08795 0.01305 -0.33797"/>
  tempT.setProfile(-0.08795,0.01305,-0.33797,  0.0,0.0,0.0);
  lhFoot -> addTrans(tempT);
  // LH_FOOT <-- (objective joint origin)

  // return base;
  // return lhHip;
  return robot;
}


inline Eigen::MatrixXd getMassMatrix (const Eigen::VectorXd& gc) {

  /// !!!!!!!!!! NO RAISIM FUNCTIONS HERE !!!!!!!!!!!!!!!!!


  return Eigen::MatrixXd::Ones(18,18);
}

// HEADER END

bool analyzeStep(const Eigen::VectorXd& gc, size_t t, raisim::RaisimServer* server, raisim::ArticulatedSystem* anymal){
  std::cout << "STEP[" << t << "]" << std::endl;
  /// TEMPLATE (do some testing here)

  auto r = initRobot();

  raisim::Vec<3> pos;
  anymal->getFramePosition("LH_shank_fixed_LH_FOOT", pos);

  std::cout << "Foot POS: " << r->getPos(gc,"LH_FOOT").transpose() << std::endl;
  std::cout << "True POS: " << pos.e().transpose() << std::endl ;

  std::cout << std::endl;
  /// TEMPLATE (add return condition here)
  return true;
}

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  // create raisim world
  raisim::World world; // physics world
  raisim::RaisimServer server(&world);

  // kinova
  auto anymal = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) + "/anymal_c/urdf/anymal.urdf");

  world.addGround();
  world.setTimeStep(0.001);

  // kinova configuration
  Eigen::VectorXd gc(anymal->getGeneralizedCoordinateDim()), gv(anymal->getDOF());
  gc << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8; /// Jemin: I'll randomize the gc, gv when grading
  gv << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8;

  utils::gcRandomize(gc);
  utils::gvRandomize(gv);
  anymal->setState(gc, gv);
  server.launchServer();

  /// if you are using an old version of Raisim, you need this line
  world.integrate1();
  bool correct = analyzeStep(gc,0,&server,anymal);

  for (int sec=5; sec>0; sec--){
    std::cout << "Dropping in [" << sec << "]..." << std::endl;
    raisim::USLEEP(1000000);
  }
  std::cout << "DROP!" << std::endl;

  // std::cout<<"mass matrix should be \n"<< anymal->getMassMatrix().e()<<std::endl;
  for (size_t i = 0; i<2000; i++){
    RS_TIMED_LOOP(world.getTimeStep()*2e6);
    if(i%10 == 0){
      correct = correct && analyzeStep(gc,i,&server,anymal);
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
