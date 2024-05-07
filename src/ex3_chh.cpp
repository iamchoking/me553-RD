//e
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
// [BASE]
// (floating base)

// [LH_HIP]
// <base_LH_HAA> (fixed) <origin rpy="-2.61799387799 0 -3.14159265359" xyz="-0.2999 0.104 0.0"/>
// LH_HAA
// <<LH_HAA>> (revolute) <axis xyz="-1 0 0"/> (gc[13])
// LH_HIP
//
// [LH_THIGH]
// <LH_HIP_LH_hip_fixed> (fixed) <origin rpy="-2.61799387799 0 -3.14159265359" xyz="0 0 0"/>
// LH_hip_fixed
// <LH_hip_fixed_LH_HFE> (fixed) <origin rpy="0 0 1.57079632679" xyz="-0.0599 0.08381 0.0"/>
// LH_HFE
// <<LH_HFE>> (revolute) <axis xyz="1 0 0"/> (gc[14])
// LH_THIGH
//
// [LH_SHANK]
// <LH_THIGH_LH_thigh_fixed> (fixed) <origin rpy="0 0 -1.57079632679" xyz="0 0 0"/>
// LH_thigh_fixed
// <LH_thigh_fixed_LH_KFE> (fixed) <origin rpy="0 0 1.57079632679" xyz="-0.0 0.1003 -0.285"/>
// LH_KFE
// <<LH_KFE>> (revolute) <axis xyz="1 0 0"/> (gc[15])
// LH_SHANK
//
// [LH_FOOT]
// <LH_SHANK_LH_shank_fixed> (fixed) <origin rpy="0 0 -1.57079632679" xyz="0 0 0"/>
// LH_shank_fixed
// <LH_shank_fixed_LH_FOOT> (fixed) <origin rpy="0 0 0" xyz="-0.08795 0.01305 -0.33797"/>
// LH_FOOT

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

[[maybe_unused]] Eigen::MatrixXd hc6x6(double m,double ixx, double ixy, double ixz, double iyy, double iyz, double izz){
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

  Trans(const Eigen::Vector3d& r, const Eigen::Matrix3d& R){ //simple creation (result of evalTrans)
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
  [[maybe_unused]] void setProfile(double* xyz,double* rpy,char newTyp = 'f',double* ax3d = nullptr,int gcIndex = -1,int gvIndex = -1){
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
  Trans evalTrans(const Eigen::VectorXd &gc){
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
    return {newPos,newRot}; // equivalent to evalTrans(newPos,newRot)
  }
};

class Inertia{
public:
  double m;
  Eigen::Matrix3d I;
  Trans com;

  bool isEmpty;

  Inertia(double mass,const Eigen::Matrix3d& momentInertia,const Trans& centerOfMass){ // standard init
    m = mass;
    I = momentInertia;
    com = centerOfMass;
    isEmpty = false;

    // rotate I right away if the com's rotation is not trivial
    straighten();

  }

  Inertia(const Inertia& i){
    m = i.m;
    I = i.I;
    com = i.com;
    isEmpty = i.isEmpty;
  }

  Inertia(){ // empty init
    clear();
  }

  void straighten(){
    if (!com.originRot.isIdentity()) {I = com.originRot * I * com.originRot.transpose();}
    com.originRot = Eigen::Matrix3d::Identity();
  }

  void clear(){
    m = 0;
    I = Eigen::Matrix3d::Zero();
    com = Trans();
    isEmpty = true;
  }

  void merge(Inertia* i2){
    // i2 must be in the same coordinate frame as i2
    straighten();
    i2->straighten();
    if(i2->isEmpty){return;}
    double m1 = m;
    double m2 = i2 -> m;
    Eigen::Vector3d rcom = (m1*com.originPos + m2*(i2->com.originPos))/(m1+m2);
    Eigen::Vector3d r1 = com.originPos-rcom;
    Eigen::Vector3d r2 = i2->com.originPos - rcom;

    m = m + i2->m;
    I = I + i2->I - m1* skew3d(r1) * skew3d(r1) - m2*skew3d(r2)* skew3d(r2);
    com.originPos = rcom;
  }

  void addInertia(Inertia* i){
    if(isEmpty){
      m = i->m;
      I = i->I;
      com = i->com;
      isEmpty = false;
    }
    else{
      merge(i);
    }
  }

  void setProfile(double x,double y,double z,double R,double P,double Y,
                  double mass,double ixx, double ixy, double ixz, double iyy, double iyz, double izz){
    com.setProfile(x,y,z,R,P,Y);
    m = mass;
    I = hcInertia(ixx,ixy,ixz,iyy,iyz,izz);
    isEmpty=false;
    straighten(); //important to straighten the orientation
  }

  /// (pure function) returns another inertia that is expressed on another frame
  [[nodiscard]] Inertia expressedIn(Trans expT) const{ // express inertia in this frame
    expT.attachTrans(com); //expT comes first (attach com to expT)
    return {m,I,expT};
    // return Inertia(m,I,expT); // (equivalent)
  }
};


class Link{
public:
  std::string name; //name
  char typ; //'b': base link, 'a': articulated link, 'e': end-effector (no actuation)
  Link* parent; //pointer to parent link
  std::vector<Link*> children; //pointer(s) to children link

  // ex2: !! changed this from "final" to "root" references!!

  Trans bodyT; // full transform from (parent^2<>parent) to (parent<>this) (r,R,p,gcIdx,gvIdx) ("assume parent^2 is fixed to world")
  Inertia bodyI; // body inertia (of the current body (expressed with body origin as origin)
  // transform order (urdf convention) r -> R -> p
  // translate by r (expressed in (p^2<>p) frame)
  // rotate by R (expressed in (p^2<>p) frame)
  // actuate wrt p by (gc[gcIdx]) (expressed in new frame(r -> R))

  // state variable (another transform)
  Trans worldT;    // transform from world to i  (world --> frame attached to parent (used for jacobian))
  Trans fullT;     // transform from world to i' (world --> frame attached to self)
  Inertia worldI;  // transform body inertia to world inertia
  Inertia compI;   // composite inertia including own body and all supporting bodies (in world coordinates)

  // flags
  bool calcKin;  // pass 1: kinematics calculated (root->leaf)
  bool calcComp; // pass 2: composite inertia calculated (leaf -> root)
  //*: non-constant

  Link(const std::string& n,const char linkTyp){
    name = n;
    typ = linkTyp;
    calcKin = false; // pass 1: kinematics calculated (root->leaf)
    calcComp = false; // pass 2: composite inertia calculated (leaf -> root)

    bodyT  = Trans();
    bodyI  = Inertia();

    worldT = Trans();
    fullT  = Trans();

    worldI = Inertia();
    compI  = Inertia();

    parent = nullptr; //this will be modified within the Robot Class
  }

  void addTrans(const Trans& newT){
    // add a transformation at the end of the link
    // only modifies constant properties
    bodyT.attachTrans(newT);
  }

  void addInertia(Inertia newI){
    bodyI.addInertia(&newI);
  }

  // Trans propLinkKin(const Eigen::VectorXd& gc){ //returns frame for (i') frame (instead of (i))
  //   return worldT.evalTrans(gc);
  // }

  void calcLinkKin(const Eigen::VectorXd& gc){
    if(typ == 'b'){ //base case (no parent link, get wr / wR from base-pos)
      // std::cout<< "base floating body: " << gc.transpose() << std::endl;
      worldT.originPos = gc.segment(0,3);
      worldT.originRot = quatToRot(gc.segment(3,4));
      fullT = worldT;
      resolveWorldI();
      calcKin = true;
      return;
    }
    else if(!parent->calcKin){
      throw(std::invalid_argument("Parent Kinematics Not Calculated!"));
    }
    // std::cout << "parent pos" << worldT.originPos << std::endl;
    // std::cout << "parent ori" << worldT.originRot << std::endl;
    // std::cout << "parent typ" << worldT.typ << std::endl;

    // worldT = parent->propLinkKin(gc);
    worldT = parent->fullT; //get full transform of parent (transform of parent's "parent joint" (attached to parent))
    worldT.attachTrans(bodyT); // just attach my own transform!
    fullT = worldT.evalTrans(gc);
    // although each links have to keep axis information (for J), the output for kinematics must be evaluated!
    resolveWorldI();
    calcKin = true;
    // return;
  }

  void calcCompInertia(){
    for(auto child:children){
      if(!(child->calcComp) || !(child->calcKin)){throw(std::invalid_argument("The children's composite inertia is not calculated!"));}
    }
    std::cout << "[" << name << "] input check complete" << std::endl;
    compI = worldI;
    for(auto child:children){
      compI.merge(&(child->compI));
    }

    calcComp = true;
    // return;
  }

  /// only call this inside calcLinkKin!
  void resolveWorldI(){
    worldI = bodyI.expressedIn(fullT);
  }

  /// add influence of this link to the provided Jp
  void augmentJp(Eigen::MatrixXd& Jp, const Eigen::VectorXd& wree){
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

  void augmentJa(Eigen::MatrixXd Ja, const Eigen::VectorXd& wree){
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
  bool calcComp;

  Link* root;

  Robot(size_t gcDimensions,size_t gvDimensions){
    gcDim = gcDimensions;
    gvDim = gvDimensions;
    dof = int(gvDimensions);
    root = nullptr;
    calcKin = false;
    calcComp = false;
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

  int findLinkIdx(const std::string& n){
    for(size_t i = 0;i<links.size();i++){
      if(links[i]->name == n){return int(i);}
    }
    return -1;
  }

  int findLinkIdx(const int& gvIndex){ //find index with gv index
    for(size_t i = 0;i<links.size();i++){
      if(links[i]->bodyT.gvIdx == gvIndex){return int(i);}
    }
    return -1;
  }

  Link* getLinkByName(const std::string& n){
    return links[findLinkIdx(n)];
  }

  Link* getLinkByGvIdx(const int& gvIndex){
    return links[findLinkIdx(gvIndex)];
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

  void addLink(Link* l,const std::string& pName){
    if(pName == ""){return addLink(l,nullptr);}
    if(findLinkIdx(pName) < 0){throw std::invalid_argument("no such parent with name" + pName);}
    return addLink(l,links[findLinkIdx(pName)]);
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
    calcComp = false;
    for(auto l:links){
      l -> calcKin = false;
      l -> calcComp = false;
    }
  }

  /// Positional Jacobian (modifier)
  void calculateJp(Eigen::MatrixXd& Jp,const Eigen::VectorXd& gc,const std::string& linkName,const Eigen::Vector3d &wree){
    // initialize
    if(!calcKin){calculateKinematics(gc);}
    Jp.resize(3,long(gvDim));
    Jp.setZero();

    Link* l = links[findLinkIdx(linkName)];
    while(l->parent != nullptr){
      l->augmentJp(Jp,wree);
      l = l->parent;
    }
    //final time for base
    l->augmentJp(Jp,wree);
  }

  /// Angular Jacobian
  void calculateJa(Eigen::MatrixXd& Ja,const Eigen::VectorXd& gc,const std::string& linkName,const Eigen::Vector3d &wree){
    // initialize
    if(!calcKin){calculateKinematics(gc);}
    Ja.resize(3,long(gvDim));
    Ja.setZero();

    Link* l = links[findLinkIdx(linkName)];
    while(l->parent != nullptr){
      l->augmentJa(Ja,wree);
      l = l->parent;
    }
    //final time for base
    l->augmentJa(Ja,wree);
  }

  [[maybe_unused]] void calculateJ(Eigen::MatrixXd& J,const Eigen::VectorXd& gc,const std::string& linkName,const Eigen::Vector3d &wree){
    J.resize(6,long(gvDim));
    J.setZero();

    Eigen::MatrixXd Jp;
    Eigen::MatrixXd Ja;
    calculateJp(Jp,gc,linkName,wree);
    calculateJa(Ja,gc,linkName,wree);

    J << Jp,Ja;

  void calculateCompositeInertia(){
    if(!calcKin){throw(std::invalid_argument("This robot's kinematics was not yet calculated!"));}
    // reverse calculation!
    for(int idx = links.size()-1;idx >= 0;--idx){
      links[idx]->calcCompInertia();
      std::cout << "Comp Inertia for " << links[idx] -> name << " finished" << std::endl;
    }
    calcComp = true;
  }

  Eigen::MatrixXd calculateMassMatrix(){
    if(!calcKin ){throw(std::invalid_argument("This robot's kinematics was not yet calculated!"));}
    if(!calcComp){throw(std::invalid_argument("This robot's composite inertia isn't populated!"));} 

    Eigen::MatrixXd M;
    M.resize(gvDim,gvDim);

  }

  }

  Trans* getTrans(const std::string &linkName){
    auto l = links[findLinkIdx(linkName)];
    if(!(l->calcKin)){throw(std::invalid_argument("Link Kinematics not yet calculated!"));}
    return &(l->fullT);
  }

  Inertia* getInertia(const std::string &linkName){
    auto l = links[findLinkIdx(linkName)];
    if(!(l->calcKin)){throw(std::invalid_argument("Link Kinematics not yet calculated!"));}
    return &(l->worldI);
  }

  Eigen::Vector3d getPos(const Eigen::VectorXd &gc,const std::string &linkName){
    calculateKinematics(gc);
    return getTrans(linkName)->originPos;
  }

  Eigen::Vector3d getCom(const Eigen::VectorXd &gc,const std::string &linkName){
    calculateKinematics(gc);
    return getInertia(linkName)->com.originPos;
  }

};

void initRobotTrans(Robot& robot) {
  // HARD-CODING: Link transformations.

  Trans tempT = Trans();

  ////////////////////// LEFT-HIND LEG START  ////////////////////////
  // note: though this can be further simplified, this is left as-is to mimic the workflow in [anymal.urdf].
  // "BASE"
  auto base    = new Link("BASE",'b');
  robot.addLink(base);
  // base

  // "HIP"
  auto lhHip = new Link("LH_HIP",'a');
  robot.addLink(lhHip,base);
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
  robot.addLink(lhThi,lhHip);
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
  robot.addLink(lhSha,lhThi);
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
  robot.addLink(lhFoot,lhSha);
  // LH_SHANK
  // <LH_SHANK_LH_shank_fixed> (fixed) <origin rpy="0 0 -1.57079632679" xyz="0 0 0"/>
  tempT.setProfile(0.0,0.0,0.0,  0.0,0.0,-1.57079632679);
  lhFoot -> addTrans(tempT);
  // LH_shank_fixed
  // <LH_shank_fixed_LH_FOOT> (fixed) <origin rpy="0 0 0" xyz="-0.08795 0.01305 -0.33797"/>
  tempT.setProfile(-0.08795,0.01305,-0.33797,  0.0,0.0,0.0);
  lhFoot -> addTrans(tempT);
  // LH_FOOT <-- (objective joint origin)

  ////////////////////// LEFT-HIND LEG FINISH ////////////////////////
}

void initRobotInertia(Robot& robot){
  auto tempT  = Trans(); // the "cumulating" trans
  auto tempTT = Trans(); // the "adding" trans
  auto tempI  = Inertia();

  ////////////////////// LEFT-HIND LEG START  ////////////////////////
  //---attached to BASE ---
  // (base --"base_LH_HAA" --> LH_HAA) (base added later)
    // <base_LH_HAA> (fixed) <origin rpy="-2.61799387799 0 -3.14159265359" xyz="-0.2999 0.104 0.0"/>
    tempT.setProfile(-0.2999,0.104,0.0,  -2.61799387799,0,-3.14159265359);
    // [LH_HAA]
    // <origin rpy="0 0 0" xyz="-0.063 7e-05 0.00046"/>
    // <mass value="2.04"/>
    // <inertia ixx="0.001053013" ixy="4.527e-05" ixz="8.855e-05" iyy="0.001805509" iyz="9.909e-05" izz="0.001765827"/>
    tempI.setProfile(-0.063,7e-05,0.00046,  0,0,0,  2.04,  0.001053013,4.527e-05,8.855e-05,0.001805509,9.909e-05,0.001765827);
    robot.getLinkByName("BASE")->addInertia(tempI.expressedIn(tempT));

  //---attached to LH_HIP ---
  // (LH_HIP --"LH_HIP_LH_hip_fixed"--> LH_hip_fixed --> "LH_hip_fixed_LH_HFE" --> LH_HFE)
    // [LH_HIP]
    // <origin rpy="0 0 0" xyz="0 0 0"/>
    // <mass value="0.001"/>
    // <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.000001"/>
    tempI.setProfile(0,0,0,  0,0,0,  0.001,  0.000001,0.0,0.0,0.000001,0.0,0.000001);
    robot.getLinkByName("LH_HIP")->addInertia(tempI);

    // <LH_HIP_LH_hip_fixed> (fixed) <origin rpy="-2.61799387799 0 -3.14159265359" xyz="0 0 0"/>
    tempT.setProfile(0.0,0.0,0.0,  -2.61799387799,0.0,-3.14159265359);
    // [LH_hip_fixed]
    // <origin rpy="0 0 0" xyz="-0.048 0.008 -0.003"/>
    // <mass value="0.74"/>
    // <inertia ixx="0.001393106" ixy="-8.4012e-05" ixz="-2.3378e-05" iyy="0.003798579" iyz="7.1319e-05" izz="0.003897509"/>
    tempI.setProfile(-0.048,0.008,-0.003,  0,0,0,  0.74,0.001393106,-8.4012e-05,-2.3378e-05,0.003798579,7.1319e-05,0.003897509);
    robot.getLinkByName("LH_HIP")->addInertia(tempI.expressedIn(tempT));

    // <LH_hip_fixed_LH_HFE> (fixed) <origin rpy="0 0 1.57079632679" xyz="-0.0599 0.08381 0.0"/>
    tempTT.setProfile(-0.0599,0.08381,0.0,  0.0,0.0,1.57079632679);
    tempT.attachTrans(tempTT);
    // [LH_HFE]
    // <origin rpy="0 0 0" xyz="-0.063 7e-05 0.00046"/>
    // <mass value="2.04"/>
    // <inertia ixx="0.001053013" ixy="4.527e-05" ixz="8.855e-05" iyy="0.001805509" iyz="9.909e-05" izz="0.001765827"/>
    tempI.setProfile(-0.063,7e-05,0.00046,  0,0,0,  2.04,  0.001053013,4.527e-05,8.855e-05,0.001805509,9.909e-05,0.001765827);
    robot.getLinkByName("LH_HIP")->addInertia(tempI.expressedIn(tempT));

  //---attached to LH_THIGH ---
  // (LH_THIGH --"LH_THIGH_LH_thigh_fixed"--> LH_thigh_fixed --> "LH_thigh_fixed_LH_KFE" --> LH_KFE)
    //[LH_THIGH]
    // <origin rpy="0 0 0" xyz="0 0 0"/>
    // <mass value="0.001"/>
    // <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.000001"/>
    tempI.setProfile(0,0,0,0,0,0,0.001,0.000001,0.0,0.0,0.000001,0.0,0.000001);
    robot.getLinkByName("LH_THIGH")->addInertia(tempI);

    // <LH_THIGH_LH_thigh_fixed> (fixed) <origin rpy="0 0 -1.57079632679" xyz="0 0 0"/>
    tempT.setProfile(0.0,0.0,0.0,  0.0,0.0,-1.57079632679);
    // [LH_thigh_fixed]
    // <origin rpy="0 0 0" xyz="-0.0 0.018 -0.169"/>
    // <mass value="1.03"/>
    // <inertia ixx="0.018644469" ixy="-5.2e-08" ixz="-1.0157e-05" iyy="0.019312599" iyz="0.002520077" izz="0.002838361"/>
    tempI.setProfile(0,0.018,-0.169, 0,0,0,  1.03,  0.018644469,-5.2e-08,-1.0157e-05,0.019312599,0.002520077,0.002838361);
    robot.getLinkByName("LH_THIGH")->addInertia(tempI.expressedIn(tempT));

    // <LH_thigh_fixed_LH_KFE> (fixed) <origin rpy="0 0 1.57079632679" xyz="-0.0 0.1003 -0.285"/>
    tempTT.setProfile(0.0,0.1003,-0.285,  0.0,0.0,1.57079632679);
    tempT.attachTrans(tempTT);
    // [LH_KFE]
    // <origin rpy="0 0 0" xyz="-0.063 7e-05 0.00046"/>
    // <mass value="2.04"/>
    // <inertia ixx="0.001053013" ixy="4.527e-05" ixz="8.855e-05" iyy="0.001805509" iyz="9.909e-05" izz="0.001765827"/>
    tempI.setProfile(-0.063,7e-05,0.00046,0,0,0,2.04 ,0.001053013,4.527e-05,8.855e-05,0.001805509,9.909e-05,0.001765827);
    robot.getLinkByName("LH_THIGH")->addInertia(tempI.expressedIn(tempT));

  //---attached to LH_SHANK ---
  // (LH_SHANK --"LH_SHANK_LH_shank_fixed"--> LH_shank_fixed --> "LH_shank_fixed_LH_FOOT" --> LH_FOOT)

    // [LH_SHANK] <origin rpy="0 0 0" xyz="0 0 0"/>
    // mass value="0.001"/> <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.000001" izz="0.000001"/>
    tempI.setProfile(0,0,0,  0,0,0,  0.001,  0.000001, 0.0, 0.0, 0.000001, 0.0, 0.000001);
    robot.getLinkByName("LH_SHANK")->addInertia(tempI);

    // <LH_shank_LH_shank_fixed> (fixed) <origin rpy="0 0 -1.57079632679" xyz="0 0 0"/>
    tempT.setProfile(0.0, 0, 0,   0, 0, -1.57079632679);
    // [LH_shank_fixed] <origin rpy="0 0 0" xyz="-0.03463 0.00688 0.00098"/>
    // mass value="0.33742"/> <inertia ixx="0.00032748005" ixy="-2.142561e-05" ixz="-1.33942e-05" iyy="0.00110974122" iyz="0.00110974122" izz="0.00089388521"/>
    tempI.setProfile(-0.03463,0.00688,0.00098,  0,0,0,  0.33742,  0.00032748005, -2.142561e-05, -1.33942e-05, 0.00110974122, 7.601e-08, 0.00089388521);
    robot.getLinkByName("LH_SHANK")->addInertia(tempI.expressedIn(tempT));

    // <LH_shank_fixed_LH_FOOT> (fixed) <origin rpy="0 0 0" xyz="-0.08795 0.01305 -0.33797"/>
    tempTT.setProfile(-0.08795, 0.01305, -0.33797,   0, 0, 0);
    tempT.attachTrans(tempTT);
    // [LH_FOOT] <origin rpy="0 0 0" xyz="-0.00948 -0.00948 0.1468"/>
    // mass value="0.25"/> <inertia ixx="0.00317174097" ixy="-2.63048e-06" ixz="-6.815581e-05" iyy="0.00317174092" iyz="0.00317174092" izz="8.319196e-05"/>
    tempI.setProfile(-0.00948,-0.00948,0.1468,  0,0,0,  0.25,  0.00317174097, -2.63048e-06, -6.815581e-05, 0.00317174092, 6.815583e-05, 8.319196e-05);
    robot.getLinkByName("LH_SHANK")->addInertia(tempI.expressedIn(tempT));

  //---attached to LH_FOOT --- (<- "ghost" link (nothing attached)) ---
  ////////////////////// LEFT-HIND LEG FINISH ////////////////////////

}

Robot initRobot(){
  Robot robot = Robot(19,18);
  initRobotTrans(robot);
  initRobotInertia(robot);

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
  r.calculateKinematics(gc);
  r.calculateCompositeInertia();

  raisim::Vec<3> pos;
  anymal->getFramePosition("LH_shank_fixed_LH_FOOT", pos);
  auto MTrue = anymal->getMassMatrix().e(); // required for other calculations

  auto inertias = anymal->getInertia();
  auto masses = anymal->getMass();
  auto coms   = anymal->getBodyCOM_B();
  auto names  = anymal->getBodyNames();
  auto comWs  = anymal->getBodyCOM_W();

  auto compositeInertias = anymal->getCompositeInertia();
  auto compositeMasses   = anymal->getCompositeMass();
  auto compositeComs     = anymal->getCompositeCOM();

  // for (size_t i = 0; i < inertias.size() ;i++){
  //   std::cout << "RAISIM: [" << i <<"] " << names[i] << std::endl;
  //   std::cout << "m: " << masses[i] << "  com: " << coms[i].e().transpose() << std::endl;
  //   std::cout << inertias[i] <<std::endl;
  // }
  std::string target = "LH_HIP";

  auto l = r.getLinkByName(target);
  size_t bodyIdx = anymal->getBodyIdx(target);

  std::cout << "------COMPOSITE-INERTIA------"<<std::endl;
  std::cout << "MINE: [" << l->name << "]" << std::endl;
  std::cout << "m: " << l->compI.m << "  com: " << l->compI.com.originPos.transpose() << std::endl;
  std::cout << l->compI.I <<std::endl;

  std::cout << "RAISIM: [" << bodyIdx <<"] " << names[bodyIdx] << std::endl;
  std::cout << "m: " << compositeMasses[bodyIdx] << "  com: " << compositeComs[bodyIdx].e().transpose() << std::endl;
  std::cout << compositeInertias[bodyIdx] <<std::endl;


  // marking positions
  server->getVisualObject("debug_X")->setPosition(l->compI.com.originPos);
  server->getVisualObject("debug_O")->setPosition(compositeComs[bodyIdx].e());
  
  std::cout << "------[SANITY-CHECK]------" << std::endl;
  std::cout << "LH_FOOT POS (MINE)  : " << r.getPos(gc,"LH_FOOT").transpose() << std::endl;
  std::cout << "LH_FOOT POS (RAISIM): " << pos.e().transpose() << std::endl ;


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

  // debug spheres
  auto debugO = server.addVisualSphere("debug_O", 0.05);
  auto debugX = server.addVisualSphere("debug_X", 0.05);
  debugO->setColor(0,1,0,1);
  debugX->setColor(1,0,0,1);

  // kinova configuration
  Eigen::VectorXd gc(anymal->getGeneralizedCoordinateDim()), gv(anymal->getDOF());
  gc << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8; /// Jemin: I'll randomize the gc, gv when grading
  gv << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8;

  utils::gcRandomize(gc);
  gc[2] = gc[2] + 3;
  utils::gvRandomize(gv,15);
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
    RS_TIMED_LOOP(world.getTimeStep()*2e6)
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
