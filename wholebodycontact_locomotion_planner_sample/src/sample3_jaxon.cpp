#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <ros/package.h>

#include <wholebodycontact_locomotion_planner/RobotState.h>
#include <wholebodycontact_locomotion_planner/wholebodycontact_locomotion_planner.h>
#include <choreonoid_bullet/choreonoid_bullet.h>
#include <choreonoid_cddlib/choreonoid_cddlib.h>
#include <ik_constraint2_bullet/ik_constraint2_bullet.h>
#include "world_common.h"

namespace wholebodycontact_locomotion_planner_sample{
  void sample3_jaxon(){
    cnoid::BodyPtr obstacle;
    std::shared_ptr<wholebodycontact_locomotion_planner::Environment> environment;
    generateStepWorld(obstacle, environment);

    std::shared_ptr<wholebodycontact_locomotion_planner::WBLPParam> param = std::make_shared<wholebodycontact_locomotion_planner::WBLPParam>();
    cnoid::BodyPtr abstractRobot;

    // load robot
    std::string modelfile = ros::package::getPath("msl_hand_models") + "/JAXON_RED_WITH_MSLHAND/JAXON_REDmain.wrl";
    cnoid::BodyLoader bodyLoader;
    param->robot = bodyLoader.load(modelfile);
    if(!(param->robot)) std::cerr << "!robot" << std::endl;
    std::vector<std::string> contactableLinkNames{
    "BODY",
    "CHEST_JOINT0",
    "CHEST_JOINT1",
    "CHEST_JOINT2",
    "HEAD_JOINT0",
    "HEAD_JOINT1",
    "LARM_JOINT0",
    "LARM_JOINT1",
    "LARM_JOINT2",
    "LARM_JOINT3",
    "LARM_JOINT4",
    "LARM_JOINT5",
    "LARM_JOINT6",
    "LARM_JOINT7",
    "RARM_JOINT0",
    "RARM_JOINT1",
    "RARM_JOINT2",
    "RARM_JOINT3",
    "RARM_JOINT4",
    "RARM_JOINT5",
    "RARM_JOINT6",
    "RARM_JOINT7",
    "LLEG_JOINT0",
    "LLEG_JOINT1",
    "LLEG_JOINT2",
    "LLEG_JOINT3",
    "LLEG_JOINT4",
    "LLEG_JOINT5",
    "RLEG_JOINT0",
    "RLEG_JOINT1",
    "RLEG_JOINT2",
    "RLEG_JOINT3",
    "RLEG_JOINT4",
    "RLEG_JOINT5",
    "HANDBASE_L",
    "L_THUMB_JOINT0",
    "L_THUMB_JOINT1",
    "L_INDEX_JOINT0",
    "L_INDEX_JOINT1",
    "L_MIDDLE_JOINT0",
    "L_LOCK_JOINT0",
    "HANDBASE_R",
    "R_THUMB_JOINT0",
    "R_THUMB_JOINT1",
    "R_INDEX_JOINT0",
    "R_INDEX_JOINT1",
    "R_MIDDLE_JOINT0",
    "R_LOCK_JOINT0",
    };
    // reset manip pose
    param->robot->rootLink()->p() = cnoid::Vector3(0,0,1.0);
    param->robot->rootLink()->v().setZero();
    param->robot->rootLink()->R() = cnoid::Matrix3::Identity();
    param->robot->rootLink()->w().setZero();
    std::vector<double> reset_manip_pose{
      0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0,// rleg
        0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0,// lleg
        0.0, 0.0, 0.0, // torso
        0.0, 0.0, // head
        0.0, 0.959931, -0.349066, -0.261799, -1.74533, -0.436332, 0.0, -0.785398,// rarm
        0.0, 0.959931, 0.349066, 0.261799, -1.74533, 0.436332, 0.0, -0.785398,// larm
        0.0, 1.5708, M_PI*2/3, M_PI/2, M_PI*2/3, 0.0, // right_hand
        0.0, 1.5708, -M_PI*2/3, -M_PI/2, -M_PI*2/3, 0.0 // left_hand
        };

    for(int j=0; j < param->robot->numJoints(); ++j){
      param->robot->joint(j)->q() = reset_manip_pose[j];
    }
    param->robot->calcForwardKinematics();
    param->robot->calcCenterOfMass();
    wholebodycontact_locomotion_planner::createAbstractRobot(param, contactableLinkNames, abstractRobot);

    // variables
    {
      param->variables.push_back(param->robot->rootLink());
      for(int i=0;i<param->robot->numJoints();i++){
        param->variables.push_back(param->robot->joint(i));
      }
    }
    // task: nominal constairnt
    {
      for(int i=0;i<param->robot->numJoints();i++){
        std::shared_ptr<ik_constraint2::JointAngleConstraint> constraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
        constraint->joint() = param->robot->joint(i);
        constraint->targetq() = reset_manip_pose[i];
        constraint->precision() = 1e10; // always satisfied
        param->nominals.push_back(constraint);
      }
    }
    param->currentContactPoints.clear();
    {
      {
        std::shared_ptr<wholebodycontact_locomotion_planner::Contact> lleg = std::make_shared<wholebodycontact_locomotion_planner::Contact>();
        lleg->name = "LLEG_JOINT5";
        lleg->link1 = param->robot->link("LLEG_JOINT5");
        lleg->localPose1.translation() = cnoid::Vector3(0.0,0.0,-0.1);
        lleg->localPose2 = lleg->link1->T() * lleg->localPose1;
        Eigen::SparseMatrix<double,Eigen::RowMajor> C(11,6);
        C.insert(0,2) = 1.0;
        C.insert(1,0) = 1.0; C.insert(1,2) = 0.2;
        C.insert(2,0) = -1.0; C.insert(2,2) = 0.2;
        C.insert(3,1) = 1.0; C.insert(3,2) = 0.2;
        C.insert(4,1) = -1.0; C.insert(4,2) = 0.2;
        C.insert(5,2) = 0.05; C.insert(5,3) = 1.0;
        C.insert(6,2) = 0.05; C.insert(6,3) = -1.0;
        C.insert(7,2) = 0.05; C.insert(7,4) = 1.0;
        C.insert(8,2) = 0.05; C.insert(8,4) = -1.0;
        C.insert(9,2) = 0.005; C.insert(9,5) = 1.0;
        C.insert(10,2) = 0.005; C.insert(10,5) = -1.0;
        lleg->C = C;
        cnoid::VectorX dl = Eigen::VectorXd::Zero(11);
        lleg->dl = dl;
        cnoid::VectorX du = 1e10 * Eigen::VectorXd::Ones(11);
        du[0] = 2000.0;
        lleg->du = du;
        param->currentContactPoints.push_back(lleg);
      }
      {
        std::shared_ptr<wholebodycontact_locomotion_planner::Contact> rleg = std::make_shared<wholebodycontact_locomotion_planner::Contact>();
        rleg->name = "RLEG_JOINT5";
        rleg->link1 = param->robot->link("RLEG_JOINT5");
        rleg->localPose1.translation() = cnoid::Vector3(0.0,0.0,-0.1);
        rleg->localPose2 = rleg->link1->T() * rleg->localPose1;
        Eigen::SparseMatrix<double,Eigen::RowMajor> C(11,6);
        C.insert(0,2) = 1.0;
        C.insert(1,0) = 1.0; C.insert(1,2) = 0.2;
        C.insert(2,0) = -1.0; C.insert(2,2) = 0.2;
        C.insert(3,1) = 1.0; C.insert(3,2) = 0.2;
        C.insert(4,1) = -1.0; C.insert(4,2) = 0.2;
        C.insert(5,2) = 0.05; C.insert(5,3) = 1.0;
        C.insert(6,2) = 0.05; C.insert(6,3) = -1.0;
        C.insert(7,2) = 0.05; C.insert(7,4) = 1.0;
        C.insert(8,2) = 0.05; C.insert(8,4) = -1.0;
        C.insert(9,2) = 0.005; C.insert(9,5) = 1.0;
        C.insert(10,2) = 0.005; C.insert(10,5) = -1.0;
        rleg->C = C;
        cnoid::VectorX dl = Eigen::VectorXd::Zero(11);
        rleg->dl = dl;
        cnoid::VectorX du = 1e10 * Eigen::VectorXd::Ones(11);
        du[0] = 2000.0;
        rleg->du = du;
        param->currentContactPoints.push_back(rleg);
      }
    }
    param->constraints.clear();
    // environmental collision
    for (int i=0; i<param->robot->numLinks(); i++) {
      std::shared_ptr<ik_constraint2_distance_field::DistanceFieldCollisionConstraint> constraint = std::make_shared<ik_constraint2_distance_field::DistanceFieldCollisionConstraint>();
      constraint->A_link() = param->robot->link(i);
      constraint->field() = environment->obstacles;
      constraint->tolerance() = 0.06; // ちょうど干渉すると法線ベクトルが変になることがあるので, 1回のiterationで動きうる距離よりも大きくせよ.
      constraint->precision() = 0.05; // 角で不正確になりがちなので, toleranceを大きくしてprecisionも大きくして、best effort的にする. precisionはdistanceFieldのサイズの倍数より大きくする. 大きく動くのでつま先が近かったときにつま先は近くならないがかかとが地面にめり込む、ということは起こりうる.
      constraint->ignoreDistance() = 0.5; // 大きく動くので、ignoreも大きくする必要がある
      //      constraint->maxError() = 0.1; // めり込んだら一刻も早く離れたい
      constraint->updateBounds(); // キャッシュを内部に作る. キャッシュを作ったあと、10スレッドぶんコピーする方が速い
      param->constraints.push_back(constraint);
    }
    // task: self collision
    {
            std::vector<std::vector<std::string> > pairs {
        std::vector<std::string>{"RLEG_JOINT2","LLEG_JOINT2"}, std::vector<std::string>{"RLEG_JOINT2","LLEG_JOINT3"}, std::vector<std::string>{"RLEG_JOINT2","LLEG_JOINT5"}, std::vector<std::string>{"RLEG_JOINT2","RARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT2","RARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT2","RARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT2","RARM_JOINT6"}, std::vector<std::string>{"RLEG_JOINT2","LARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT2","LARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT2","LARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT2","LARM_JOINT6"}, std::vector<std::string>{"RLEG_JOINT3","LLEG_JOINT2"}, std::vector<std::string>{"RLEG_JOINT3","LLEG_JOINT3"}, std::vector<std::string>{"RLEG_JOINT3","LLEG_JOINT5"}, std::vector<std::string>{"RLEG_JOINT3","RARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT3","RARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT3","RARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT3","RARM_JOINT6"}, std::vector<std::string>{"RLEG_JOINT3","LARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT3","LARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT3","LARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT3","LARM_JOINT6"}, std::vector<std::string>{"RLEG_JOINT5","LLEG_JOINT2"}, std::vector<std::string>{"RLEG_JOINT5","LLEG_JOINT3"}, std::vector<std::string>{"RLEG_JOINT5","LLEG_JOINT5"}, std::vector<std::string>{"RLEG_JOINT5","RARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT5","RARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT5","RARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT5","RARM_JOINT6"}, std::vector<std::string>{"RLEG_JOINT5","LARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT5","LARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT5","LARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT5","LARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT2","RARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT2","RARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT2","RARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT2","RARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT2","LARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT2","LARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT2","LARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT2","LARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT3","RARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT3","RARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT3","RARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT3","RARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT3","LARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT3","LARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT3","LARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT3","LARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT5","RARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT5","RARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT5","RARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT5","RARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT5","LARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT5","LARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT5","LARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT5","LARM_JOINT6"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT2"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT3"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT4"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT5"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT6"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT2"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT3"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT4"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT5"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT6"}, std::vector<std::string>{"HEAD_JOINT1","RARM_JOINT3"}, std::vector<std::string>{"HEAD_JOINT1","RARM_JOINT4"}, std::vector<std::string>{"HEAD_JOINT1","RARM_JOINT5"}, std::vector<std::string>{"HEAD_JOINT1","RARM_JOINT6"}, std::vector<std::string>{"HEAD_JOINT1","LARM_JOINT3"}, std::vector<std::string>{"HEAD_JOINT1","LARM_JOINT4"}, std::vector<std::string>{"HEAD_JOINT1","LARM_JOINT5"}, std::vector<std::string>{"HEAD_JOINT1","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT0","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT0","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT0","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT2","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT2","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT2","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT2","WAIST"}, std::vector<std::string>{"RARM_JOINT3","LARM_JOINT3"}, std::vector<std::string>{"RARM_JOINT3","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT3","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT3","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT3","WAIST"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT0"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT2"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT3"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT4","WAIST"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT0"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT2"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT3"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT5","WAIST"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT0"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT2"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT3"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT6","WAIST"}, std::vector<std::string>{"LARM_JOINT2","WAIST"}, std::vector<std::string>{"LARM_JOINT3","WAIST"}, std::vector<std::string>{"LARM_JOINT4","WAIST"}, std::vector<std::string>{"LARM_JOINT5","WAIST"}, std::vector<std::string>{"LARM_JOINT6","WAIST"}, std::vector<std::string>{"RLEG_JOINT2","LARM_JOINT7"}, std::vector<std::string>{"RLEG_JOINT3","LARM_JOINT7"}, std::vector<std::string>{"RLEG_JOINT5","LARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT2","LARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT3","LARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT5","LARM_JOINT7"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT7"}, std::vector<std::string>{"HEAD_JOINT1","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT0","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT2","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT3","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT7"}, std::vector<std::string>{"LARM_JOINT7","WAIST"}, std::vector<std::string>{"RLEG_JOINT2","RARM_JOINT7"}, std::vector<std::string>{"RLEG_JOINT3","RARM_JOINT7"}, std::vector<std::string>{"RLEG_JOINT5","RARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT2","RARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT3","RARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT5","RARM_JOINT7"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT7"}, std::vector<std::string>{"HEAD_JOINT1","RARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT0"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT2"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT3"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT7","WAIST"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT3"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT4"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT5"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT6"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT7"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT3"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT4"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT5"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT6"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT7"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT2"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT2"}, std::vector<std::string>{"RLEG_JOINT2","HANDBASE_R"}, std::vector<std::string>{"RLEG_JOINT3","HANDBASE_R"}, std::vector<std::string>{"RLEG_JOINT5","HANDBASE_R"}, std::vector<std::string>{"LLEG_JOINT2","HANDBASE_R"}, std::vector<std::string>{"LLEG_JOINT3","HANDBASE_R"}, std::vector<std::string>{"LLEG_JOINT5","HANDBASE_R"}, std::vector<std::string>{"WAIST","HANDBASE_R"}, std::vector<std::string>{"CHEST_JOINT1","HANDBASE_R"}, std::vector<std::string>{"CHEST_JOINT2","HANDBASE_R"}, std::vector<std::string>{"HEAD_JOINT1","HANDBASE_R"}, std::vector<std::string>{"HANDBASE_R","LARM_JOINT0"}, std::vector<std::string>{"HANDBASE_R","LARM_JOINT2"}, std::vector<std::string>{"HANDBASE_R","LARM_JOINT3"}, std::vector<std::string>{"HANDBASE_R","LARM_JOINT4"}, std::vector<std::string>{"HANDBASE_R","LARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT2","R_THUMB_JOINT1"}, std::vector<std::string>{"RLEG_JOINT3","R_THUMB_JOINT1"}, std::vector<std::string>{"RLEG_JOINT5","R_THUMB_JOINT1"}, std::vector<std::string>{"LLEG_JOINT2","R_THUMB_JOINT1"}, std::vector<std::string>{"LLEG_JOINT3","R_THUMB_JOINT1"}, std::vector<std::string>{"LLEG_JOINT5","R_THUMB_JOINT1"}, std::vector<std::string>{"WAIST","R_THUMB_JOINT1"}, std::vector<std::string>{"CHEST_JOINT1","R_THUMB_JOINT1"}, std::vector<std::string>{"CHEST_JOINT2","R_THUMB_JOINT1"}, std::vector<std::string>{"HEAD_JOINT1","R_THUMB_JOINT1"}, std::vector<std::string>{"R_THUMB_JOINT1","LARM_JOINT0"}, std::vector<std::string>{"R_THUMB_JOINT1","LARM_JOINT2"}, std::vector<std::string>{"R_THUMB_JOINT1","LARM_JOINT3"}, std::vector<std::string>{"R_THUMB_JOINT1","LARM_JOINT4"}, std::vector<std::string>{"R_THUMB_JOINT1","LARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT2","HANDBASE_L"}, std::vector<std::string>{"RLEG_JOINT3","HANDBASE_L"}, std::vector<std::string>{"RLEG_JOINT5","HANDBASE_L"}, std::vector<std::string>{"LLEG_JOINT2","HANDBASE_L"}, std::vector<std::string>{"LLEG_JOINT3","HANDBASE_L"}, std::vector<std::string>{"LLEG_JOINT5","HANDBASE_L"}, std::vector<std::string>{"WAIST","HANDBASE_L"}, std::vector<std::string>{"CHEST_JOINT1","HANDBASE_L"}, std::vector<std::string>{"CHEST_JOINT2","HANDBASE_L"}, std::vector<std::string>{"HEAD_JOINT1","HANDBASE_L"}, std::vector<std::string>{"HANDBASE_L","LARM_JOINT0"}, std::vector<std::string>{"HANDBASE_L","LARM_JOINT2"}, std::vector<std::string>{"HANDBASE_L","LARM_JOINT3"}, std::vector<std::string>{"HANDBASE_L","LARM_JOINT4"}, std::vector<std::string>{"HANDBASE_L","LARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT2","L_THUMB_JOINT1"}, std::vector<std::string>{"RLEG_JOINT3","L_THUMB_JOINT1"}, std::vector<std::string>{"RLEG_JOINT5","L_THUMB_JOINT1"}, std::vector<std::string>{"LLEG_JOINT2","L_THUMB_JOINT1"}, std::vector<std::string>{"LLEG_JOINT3","L_THUMB_JOINT1"}, std::vector<std::string>{"LLEG_JOINT5","L_THUMB_JOINT1"}, std::vector<std::string>{"WAIST","L_THUMB_JOINT1"}, std::vector<std::string>{"CHEST_JOINT1","L_THUMB_JOINT1"}, std::vector<std::string>{"CHEST_JOINT2","L_THUMB_JOINT1"}, std::vector<std::string>{"HEAD_JOINT1","L_THUMB_JOINT1"}, std::vector<std::string>{"L_THUMB_JOINT1","LARM_JOINT0"}, std::vector<std::string>{"L_THUMB_JOINT1","LARM_JOINT2"}, std::vector<std::string>{"L_THUMB_JOINT1","LARM_JOINT3"}, std::vector<std::string>{"L_THUMB_JOINT1","LARM_JOINT4"}, std::vector<std::string>{"L_THUMB_JOINT1","LARM_JOINT5"}, std::vector<std::string>{"HANDBASE_R","HANDBASE_L"}, std::vector<std::string>{"R_THUMB_JOINT1","L_THUMB_JOINT1"}
      };
      std::unordered_map<cnoid::LinkPtr, std::shared_ptr<btConvexShape> > collisionModels;
      for(int i=0;i<param->robot->numLinks();i++){
        collisionModels[param->robot->link(i)] = choreonoid_bullet::convertToBulletModel(param->robot->link(i)->collisionShape());
      }

      for(int i=0;i<pairs.size();i++){
        std::shared_ptr<ik_constraint2_bullet::BulletCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletCollisionConstraint>();
        constraint->A_link() = param->robot->link(pairs[i][0]);
        constraint->B_link() = param->robot->link(pairs[i][1]);
        constraint->A_link_bulletModel() = constraint->A_link();
        constraint->A_bulletModel().push_back(collisionModels[constraint->A_link()]);
        constraint->B_link_bulletModel() = constraint->B_link();
        constraint->B_bulletModel().push_back(collisionModels[constraint->B_link()]);
        constraint->tolerance() = 0.01;
        constraint->updateBounds(); // キャッシュを内部に作る. キャッシュを作ったあと、10スレッドぶんコピーする方が速い
        param->constraints.push_back(constraint);
      }
    }

    param->modes.clear();
    {
      std::shared_ptr<wholebodycontact_locomotion_planner::Mode> mode = std::make_shared<wholebodycontact_locomotion_planner::Mode>();
      param->modes["wholebody"] = mode;
      mode->score = 2.0;
      {
        // reachability
        for (int i=0; i<param->robot->numLinks(); i++) {
          if(std::find(contactableLinkNames.begin(),contactableLinkNames.end(),param->robot->link(i)->name()) == contactableLinkNames.end()) continue;
          std::shared_ptr<ik_constraint2_distance_field::DistanceFieldCollisionConstraint> constraint = std::make_shared<ik_constraint2_distance_field::DistanceFieldCollisionConstraint>();
          constraint->A_link() = param->robot->link(i);
          constraint->field() = environment->obstacles;
          constraint->tolerance() = 0.1; // ちょうど干渉すると法線ベクトルが変になることがあるので, 1回のiterationで動きうる距離よりも大きくせよ.
          //          constraint->precision() = 0.01;
          constraint->ignoreDistance() = 0.5; // 大きく動くので、ignoreも大きくする必要がある
          //      constraint->maxError() = 0.1; // めり込んだら一刻も早く離れたい
          constraint->invert() = true; // 距離をtolerance以下にする
          constraint->updateBounds(); // キャッシュを内部に作る. キャッシュを作ったあと、10スレッドぶんコピーする方が速い
          mode->reachabilityConstraints.push_back(constraint);
        }
      }
    } // mode1

    std::string contactFileName = ros::package::getPath("tactile_demo_config") + "/config/tactile_sensor.yaml";
    wholebodycontact_locomotion_planner::createContactPoints(param, contactFileName);

    // setup viewer
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    viewer->objects(param->robot);
    viewer->objects(abstractRobot);
    viewer->objects(obstacle);
    viewer->drawObjects();

  }
}
