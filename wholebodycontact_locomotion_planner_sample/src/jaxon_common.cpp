#include "jaxon_common.h"

#include <cnoid/BodyLoader>
#include <ros/package.h>
#include <iostream>
#include <choreonoid_bullet/choreonoid_bullet.h>
#include <choreonoid_cddlib/choreonoid_cddlib.h>
#include <ik_constraint2_bullet/ik_constraint2_bullet.h>

namespace wholebodycontact_locomotion_planner_sample{
  void generateJAXON(const std::shared_ptr<moveit_extensions::InterpolatedPropagationDistanceField>& field,
                           std::shared_ptr<wholebodycontact_locomotion_planner::WBLPParam>& param,
                           cnoid::BodyPtr& abstractRobot, // for visual
                           bool quadruped
                           ) {
    cnoid::BodyLoader bodyLoader;
    param->robot = bodyLoader.load(ros::package::getPath("msl_hand_models") + "/JAXON_RED_WITH_MSLHAND/JAXON_REDmain.wrl");
    if(!(param->robot)) std::cerr << "!robot" << std::endl;
    std::vector<std::string> contactableLinkNames{
    // "BODY",
    // "CHEST_JOINT0",
    // "CHEST_JOINT1",
    // "CHEST_JOINT2",
    // "HEAD_JOINT0",
    // "HEAD_JOINT1",
    // "LARM_JOINT0",
    // "LARM_JOINT1",
    // "LARM_JOINT2",
    // "LARM_JOINT3",
    // "LARM_JOINT4",
    "LARM_JOINT5",
    // "LARM_JOINT6",
    // "LARM_JOINT7",
    // "RARM_JOINT0",
    // "RARM_JOINT1",
    // "RARM_JOINT2",
    // "RARM_JOINT3",
    // "RARM_JOINT4",
    "RARM_JOINT5",
    // "RARM_JOINT6",
    // "RARM_JOINT7",
    // "LLEG_JOINT0",
    // "LLEG_JOINT1",
    // "LLEG_JOINT2",
    // "LLEG_JOINT3",
    // "LLEG_JOINT4",
    "LLEG_JOINT5",
    // "RLEG_JOINT0",
    // "RLEG_JOINT1",
    // "RLEG_JOINT2",
    // "RLEG_JOINT3",
    // "RLEG_JOINT4",
    "RLEG_JOINT5",
    "HANDBASE_L",
    // "L_THUMB_JOINT0",
    // "L_THUMB_JOINT1",
    // "L_INDEX_JOINT0",
    // "L_INDEX_JOINT1",
    // "L_MIDDLE_JOINT0",
    // "L_LOCK_JOINT0",
    "HANDBASE_R",
    // "R_THUMB_JOINT0",
    // "R_THUMB_JOINT1",
    // "R_INDEX_JOINT0",
    // "R_INDEX_JOINT1",
    // "R_MIDDLE_JOINT0",
    // "R_LOCK_JOINT0",
      };
    if (quadruped) {
      std::vector<std::string> quadruped_contactableLinkNames{
        // "BODY",
        // "CHEST_JOINT0",
        // "CHEST_JOINT1",
        // "CHEST_JOINT2",
        // "HEAD_JOINT0",
        // "HEAD_JOINT1",
        // "LARM_JOINT0",
        // "LARM_JOINT1",
        // "LARM_JOINT2",
        // "LARM_JOINT3",
        //"LARM_JOINT4",
        // "LARM_JOINT5",
        // "LARM_JOINT6",
        // "LARM_JOINT7",
        // "RARM_JOINT0",
        // "RARM_JOINT1",
        // "RARM_JOINT2",
        // "RARM_JOINT3",
        //"RARM_JOINT4",
        // "RARM_JOINT5",
        // "RARM_JOINT6",
        // "RARM_JOINT7",
        // "LLEG_JOINT0",
        // "LLEG_JOINT1",
        "LLEG_JOINT2",
        // "LLEG_JOINT3",
        // "LLEG_JOINT4",
        // "LLEG_JOINT5",
        // "RLEG_JOINT0",
        // "RLEG_JOINT1",
        "RLEG_JOINT2",
        // "RLEG_JOINT3",
        // "RLEG_JOINT4",
        // "RLEG_JOINT5",
        "HANDBASE_L",
        // "L_THUMB_JOINT0",
        // "L_THUMB_JOINT1",
        // "L_INDEX_JOINT0",
        // "L_INDEX_JOINT1",
        // "L_MIDDLE_JOINT0",
        // "L_LOCK_JOINT0",
        "HANDBASE_R",
        // "R_THUMB_JOINT0",
        // "R_THUMB_JOINT1",
        // "R_INDEX_JOINT0",
        // "R_INDEX_JOINT1",
        // "R_MIDDLE_JOINT0",
        // "R_LOCK_JOINT0",
      };
      contactableLinkNames = quadruped_contactableLinkNames;
    }
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
    if (quadruped) {
      param->robot->rootLink()->p() = cnoid::Vector3(0.3,0,0.25);
      param->robot->rootLink()->v().setZero();
      param->robot->rootLink()->R() = cnoid::rotFromRpy(0.0,M_PI/2,0.0);
      param->robot->rootLink()->w().setZero();
      std::vector<double> quadruped_reset_manip_pose{
        0.0, 0.0, -0.549066, 0.898132, -0.349066, 0.0,// rleg
          0.0, 0.0, -0.549066, 0.898132, -0.349066, 0.0,// lleg
          0.0, 0.0, 0.0, // torso
          0.0, -0.1, // head
          0.0, -2.459931, -0.299066, 0.1, -0.74533, -0.236332, 0.2, -0.045398,// rarm
          0.0, -2.459931, 0.299066, -0.1, -0.74533, 0.236332, -0.2, -0.045398,// larm
          0.0, 1.5708, M_PI*2/3, M_PI/2, M_PI*2/3, 0.0, // right_hand
          0.0, 1.5708, -M_PI*2/3, -M_PI/2, -M_PI*2/3, 0.0 // left_hand
          };
      reset_manip_pose = quadruped_reset_manip_pose;
    }

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
        if ((param->robot->joint(i)->name() == "L_THUMB_JOINT0") ||
            (param->robot->joint(i)->name() == "L_THUMB_JOINT1") ||
            (param->robot->joint(i)->name() == "L_INDEX_JOINT0") ||
            (param->robot->joint(i)->name() == "L_INDEX_JOINT1") ||
            (param->robot->joint(i)->name() == "L_MIDDLE_JOINT0") ||
            (param->robot->joint(i)->name() == "L_LOCK_JOINT0") ||
            (param->robot->joint(i)->name() == "R_THUMB_JOINT0") ||
            (param->robot->joint(i)->name() == "R_THUMB_JOINT1") ||
            (param->robot->joint(i)->name() == "R_INDEX_JOINT0") ||
            (param->robot->joint(i)->name() == "R_INDEX_JOINT1") ||
            (param->robot->joint(i)->name() == "R_MIDDLE_JOINT0") ||
            (param->robot->joint(i)->name() == "R_LOCK_JOINT0")) continue;
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
      if(!quadruped) {
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
      } else {
        {
          std::shared_ptr<wholebodycontact_locomotion_planner::Contact> lknee = std::make_shared<wholebodycontact_locomotion_planner::Contact>();
          lknee->name = "LLEG_JOINT2";
          lknee->link1 = param->robot->link("LLEG_JOINT2");
          lknee->localPose1.translation() = cnoid::Vector3(0.045,0.073,-0.41);
          lknee->localPose2.translation() =  lknee->link1->p() + lknee->link1->R() * lknee->localPose1.translation();
          lknee->localPose1.linear() = lknee->link1->R().transpose() * lknee->localPose2.linear();
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
          lknee->C = C;
          cnoid::VectorX dl = Eigen::VectorXd::Zero(11);
          lknee->dl = dl;
          cnoid::VectorX du = 1e10 * Eigen::VectorXd::Ones(11);
          du[0] = 2000.0;
          lknee->du = du;
          param->currentContactPoints.push_back(lknee);
        }
        {
          std::shared_ptr<wholebodycontact_locomotion_planner::Contact> rknee = std::make_shared<wholebodycontact_locomotion_planner::Contact>();
          rknee->name = "RLEG_JOINT2";
          rknee->link1 = param->robot->link("RLEG_JOINT2");
          rknee->localPose1.translation() = cnoid::Vector3(0.045,-0.073,-0.41);
          rknee->localPose2.translation() =  rknee->link1->p() + rknee->link1->R() * rknee->localPose1.translation();
          rknee->localPose1.linear() = rknee->link1->R().transpose() * rknee->localPose2.linear();
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
          rknee->C = C;
          cnoid::VectorX dl = Eigen::VectorXd::Zero(11);
          rknee->dl = dl;
          cnoid::VectorX du = 1e10 * Eigen::VectorXd::Ones(11);
          du[0] = 2000.0;
          rknee->du = du;
          param->currentContactPoints.push_back(rknee);
        }
        // {
        //   std::shared_ptr<wholebodycontact_locomotion_planner::Contact> lelbow = std::make_shared<wholebodycontact_locomotion_planner::Contact>();
        //   lelbow->name = "LARM_JOINT4";
        //   lelbow->link1 = param->robot->link("LARM_JOINT4");
        //   lelbow->localPose1.translation() = cnoid::Vector3(-0.063,0.0,0.0);
        //   lelbow->localPose2.translation() =  lelbow->link1->p() + lelbow->link1->R() * lelbow->localPose1.translation();
        //   lelbow->localPose1.linear() = lelbow->link1->R().transpose() * lelbow->localPose2.linear();
        //   Eigen::SparseMatrix<double,Eigen::RowMajor> C(11,6);
        //   C.insert(0,2) = 1.0;
        //   C.insert(1,0) = 1.0; C.insert(1,2) = 0.2;
        //   C.insert(2,0) = -1.0; C.insert(2,2) = 0.2;
        //   C.insert(3,1) = 1.0; C.insert(3,2) = 0.2;
        //   C.insert(4,1) = -1.0; C.insert(4,2) = 0.2;
        //   C.insert(5,2) = 0.05; C.insert(5,3) = 1.0;
        //   C.insert(6,2) = 0.05; C.insert(6,3) = -1.0;
        //   C.insert(7,2) = 0.05; C.insert(7,4) = 1.0;
        //   C.insert(8,2) = 0.05; C.insert(8,4) = -1.0;
        //   C.insert(9,2) = 0.005; C.insert(9,5) = 1.0;
        //   C.insert(10,2) = 0.005; C.insert(10,5) = -1.0;
        //   lelbow->C = C;
        //   cnoid::VectorX dl = Eigen::VectorXd::Zero(11);
        //   lelbow->dl = dl;
        //   cnoid::VectorX du = 1e10 * Eigen::VectorXd::Ones(11);
        //   du[0] = 2000.0;
        //   lelbow->du = du;
        //   param->currentContactPoints.push_back(lelbow);
        // }
        // {
        //   std::shared_ptr<wholebodycontact_locomotion_planner::Contact> relbow = std::make_shared<wholebodycontact_locomotion_planner::Contact>();
        //   relbow->name = "RARM_JOINT4";
        //   relbow->link1 = param->robot->link("RARM_JOINT4");
        //   relbow->localPose1.translation() = cnoid::Vector3(-0.063,0.0,0.0);
        //   relbow->localPose2.translation() =  relbow->link1->p() + relbow->link1->R() * relbow->localPose1.translation();
        //   relbow->localPose1.linear() = relbow->link1->R().transpose() * relbow->localPose2.linear();
        //   Eigen::SparseMatrix<double,Eigen::RowMajor> C(11,6);
        //   C.insert(0,2) = 1.0;
        //   C.insert(1,0) = 1.0; C.insert(1,2) = 0.2;
        //   C.insert(2,0) = -1.0; C.insert(2,2) = 0.2;
        //   C.insert(3,1) = 1.0; C.insert(3,2) = 0.2;
        //   C.insert(4,1) = -1.0; C.insert(4,2) = 0.2;
        //   C.insert(5,2) = 0.05; C.insert(5,3) = 1.0;
        //   C.insert(6,2) = 0.05; C.insert(6,3) = -1.0;
        //   C.insert(7,2) = 0.05; C.insert(7,4) = 1.0;
        //   C.insert(8,2) = 0.05; C.insert(8,4) = -1.0;
        //   C.insert(9,2) = 0.005; C.insert(9,5) = 1.0;
        //   C.insert(10,2) = 0.005; C.insert(10,5) = -1.0;
        //   relbow->C = C;
        //   cnoid::VectorX dl = Eigen::VectorXd::Zero(11);
        //   relbow->dl = dl;
        //   cnoid::VectorX du = 1e10 * Eigen::VectorXd::Ones(11);
        //   du[0] = 2000.0;
        //   relbow->du = du;
        //   param->currentContactPoints.push_back(relbow);
        // }
        {
          std::shared_ptr<wholebodycontact_locomotion_planner::Contact> lhand = std::make_shared<wholebodycontact_locomotion_planner::Contact>();
          lhand->name = "HANDBASE_L";
          lhand->link1 = param->robot->link("HANDBASE_L");
          lhand->localPose1.translation() = cnoid::Vector3(0.13,0.0,-0.069);
          lhand->localPose2.translation() =  lhand->link1->p() + lhand->link1->R() * lhand->localPose1.translation();
          lhand->localPose1.linear() = lhand->link1->R().transpose() * lhand->localPose2.linear();
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
          lhand->C = C;
          cnoid::VectorX dl = Eigen::VectorXd::Zero(11);
          lhand->dl = dl;
          cnoid::VectorX du = 1e10 * Eigen::VectorXd::Ones(11);
          du[0] = 2000.0;
          lhand->du = du;
          param->currentContactPoints.push_back(lhand);
        }
        {
          std::shared_ptr<wholebodycontact_locomotion_planner::Contact> rhand = std::make_shared<wholebodycontact_locomotion_planner::Contact>();
          rhand->name = "HANDBASE_R";
          rhand->link1 = param->robot->link("HANDBASE_R");
          rhand->localPose1.translation() = cnoid::Vector3(0.13,0.0,-0.069);
          rhand->localPose2.translation() =  rhand->link1->p() + rhand->link1->R() * rhand->localPose1.translation();
          rhand->localPose1.linear() = rhand->link1->R().transpose() * rhand->localPose2.linear();
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
          rhand->C = C;
          cnoid::VectorX dl = Eigen::VectorXd::Zero(11);
          rhand->dl = dl;
          cnoid::VectorX du = 1e10 * Eigen::VectorXd::Ones(11);
          du[0] = 2000.0;
          rhand->du = du;
          param->currentContactPoints.push_back(rhand);
        }
      }
    }

    param->constraints.clear();
    // joint limit
    for(int i=0;i<param->robot->numJoints();i++){
      std::shared_ptr<ik_constraint2::JointLimitConstraint> constraint = std::make_shared<ik_constraint2::JointLimitConstraint>();
      constraint->joint() = param->robot->joint(i);
      param->constraints.push_back(constraint);
    }
    // joint displacement
    for(int i=0;i<param->robot->numJoints();i++){
      std::shared_ptr<ik_constraint2::JointDisplacementConstraint> constraint = std::make_shared<ik_constraint2::JointDisplacementConstraint>();
      constraint->joint() = param->robot->joint(i);
      param->constraints.push_back(constraint);
    }
    // environmental collision
    for (int i=0; i<param->robot->numLinks(); i++) {
      if (!quadruped) {
        if ((param->robot->link(i)->name() == "LLEG_JOINT3") || // JOINT3は膝下で手先が触れるなら本来は触れている
            (param->robot->link(i)->name() == "RLEG_JOINT3") ||
            (param->robot->link(i)->name() == "LLEG_JOINT4") || // JOINT4は足首内側に入り込んだリンクなので考慮しなくて良い
            (param->robot->link(i)->name() == "RLEG_JOINT4") ||
            (param->robot->link(i)->name() == "LARM_JOINT3") || // JOINT3は肘上で前腕が触れるなら本来は触れている
            (param->robot->link(i)->name() == "RARM_JOINT3") ||
            (param->robot->link(i)->name() == "LARM_JOINT4") || // JOINT4は肘下で前腕が触れるなら本来は触れている
            (param->robot->link(i)->name() == "RARM_JOINT4") ||
            (param->robot->link(i)->name() == "LARM_JOINT6") || // JOINT6は手首内側に入り込んだリンクなので考慮しなくて良い
            (param->robot->link(i)->name() == "RARM_JOINT6") ||
            (param->robot->link(i)->name() == "LARM_JOINT7") || // JOINT7とHANDBASEの間はfixed jointなので回避しようがない
            (param->robot->link(i)->name() == "RARM_JOINT7") ||
            (param->robot->link(i)->name() == "L_THUMB_JOINT0") ||
            (param->robot->link(i)->name() == "L_THUMB_JOINT1") ||
            (param->robot->link(i)->name() == "L_INDEX_JOINT0") ||
            (param->robot->link(i)->name() == "L_INDEX_JOINT1") ||
            (param->robot->link(i)->name() == "L_MIDDLE_JOINT0") ||
            (param->robot->link(i)->name() == "L_LOCK_JOINT0") ||
            (param->robot->link(i)->name() == "R_THUMB_JOINT0") ||
            (param->robot->link(i)->name() == "R_THUMB_JOINT1") ||
            (param->robot->link(i)->name() == "R_INDEX_JOINT0") ||
            (param->robot->link(i)->name() == "R_INDEX_JOINT1") ||
            (param->robot->link(i)->name() == "R_MIDDLE_JOINT0") ||
            (param->robot->link(i)->name() == "R_LOCK_JOINT0")) continue;
      } else {
        if ((param->robot->link(i)->name() == "LLEG_JOINT3") || // JOINT3は膝下で膝上が触れるなら本来は触れている
            (param->robot->link(i)->name() == "RLEG_JOINT3") ||
            (param->robot->link(i)->name() == "LLEG_JOINT4") || // JOINT4は足首内側に入り込んだリンクなので考慮しなくて良い
            (param->robot->link(i)->name() == "RLEG_JOINT4") ||
            // (param->robot->link(i)->name() == "LARM_JOINT3") || // JOINT3は肘上で肘下が触れるなら本来は触れている
            // (param->robot->link(i)->name() == "RARM_JOINT3") ||
            (param->robot->link(i)->name() == "LARM_JOINT4") || // JOINT3は肘上で肘下が触れるなら本来は触れている
            (param->robot->link(i)->name() == "RARM_JOINT4") ||
            (param->robot->link(i)->name() == "LARM_JOINT5") || // JOINT5は前腕で肘下が触れるなら本来は触れている
            (param->robot->link(i)->name() == "RARM_JOINT5") ||
            (param->robot->link(i)->name() == "LARM_JOINT6") || // JOINT6は手首内側に入り込んだリンクなので考慮しなくて良い
            (param->robot->link(i)->name() == "RARM_JOINT6") ||
            (param->robot->link(i)->name() == "LARM_JOINT7") || // JOINT7とHANDBASEの間はfixed jointなので回避しようがない
            (param->robot->link(i)->name() == "RARM_JOINT7") ||
            (param->robot->link(i)->name() == "L_THUMB_JOINT0") ||
            (param->robot->link(i)->name() == "L_THUMB_JOINT1") ||
            (param->robot->link(i)->name() == "L_INDEX_JOINT0") ||
            (param->robot->link(i)->name() == "L_INDEX_JOINT1") ||
            (param->robot->link(i)->name() == "L_MIDDLE_JOINT0") ||
            (param->robot->link(i)->name() == "L_LOCK_JOINT0") ||
            (param->robot->link(i)->name() == "R_THUMB_JOINT0") ||
            (param->robot->link(i)->name() == "R_THUMB_JOINT1") ||
            (param->robot->link(i)->name() == "R_INDEX_JOINT0") ||
            (param->robot->link(i)->name() == "R_INDEX_JOINT1") ||
            (param->robot->link(i)->name() == "R_MIDDLE_JOINT0") ||
            (param->robot->link(i)->name() == "R_LOCK_JOINT0")) continue;
      }
      std::shared_ptr<ik_constraint2_distance_field::DistanceFieldCollisionConstraint> constraint = std::make_shared<ik_constraint2_distance_field::DistanceFieldCollisionConstraint>();
      constraint->A_link() = param->robot->link(i);
      constraint->field() = field;
      constraint->tolerance() = 0.04; // ちょうど干渉すると法線ベクトルが変になることがあるので, 1回のiterationで動きうる距離よりも大きくせよ.
      constraint->precision() = 0.03; // 角で不正確になりがちなので, toleranceを大きくしてprecisionも大きくして、best effort的にする. precisionはdistanceFieldのサイズの倍数より大きくする. 大きく動くのでつま先が近かったときにつま先は近くならないがかかとが地面にめり込む、ということは起こりうる.
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
        constraint->tolerance() = 0.002;
        constraint->updateBounds(); // キャッシュを内部に作る. キャッシュを作ったあと、10スレッドぶんコピーする方が速い
        param->constraints.push_back(constraint);
      }
    }

    param->modes.clear();
    {
      std::shared_ptr<wholebodycontact_locomotion_planner::Mode> mode = std::make_shared<wholebodycontact_locomotion_planner::Mode>();
      param->modes["wholebody"] = mode;
      mode->score = 2.0;
      mode->minimumContactCount = 6;
      {
        // reachability
        for (int i=0; i<param->robot->numLinks(); i++) {
          if(std::find(contactableLinkNames.begin(),contactableLinkNames.end(),param->robot->link(i)->name()) == contactableLinkNames.end()) continue;
          std::shared_ptr<ik_constraint2_distance_field::DistanceFieldCollisionConstraint> constraint = std::make_shared<ik_constraint2_distance_field::DistanceFieldCollisionConstraint>();
          constraint->A_link() = param->robot->link(i);
          constraint->field() = field;
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
    std::unordered_map<std::string, std::vector<cnoid::Isometry3> > contactPoints = wholebodycontact_locomotion_planner::createContactPoints(param, contactFileName);
    for (int i=0; i<param->robot->numLinks(); i++) {
      if(std::find(contactableLinkNames.begin(),contactableLinkNames.end(),param->robot->link(i)->name()) == contactableLinkNames.end()) continue;
      cnoid::LinkPtr variable = new cnoid::Link();
      variable->setJointType(cnoid::Link::JointType::FreeJoint);
      std::shared_ptr<ik_constraint2_body_contact::BodyContactConstraint> constraint = std::make_shared<ik_constraint2_body_contact::BodyContactConstraint>();
      constraint->A_link() = param->robot->link(i);
      constraint->B_link() = nullptr;
      constraint->contact_pos_link() = variable;
      constraint->contact_pos_link()->T() = constraint->A_localpos();
      for(std::unordered_map<std::string, std::vector<cnoid::Isometry3> >::const_iterator it=contactPoints.begin(); it!=contactPoints.end(); it++){
        if (it->first==param->robot->link(i)->name()) constraint->setContactPoints(it->second, 0.05, 32);
      }
      constraint->contactSearchLimit() = 0.01; // 大きすぎると振動してしまうので注意
      constraint->precision() = 0.03;
      constraint->contactWeight() = 1;
      constraint->normalGradientDistance() = 0.03;
      constraint->weight() << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0; // rollやpitchを正確にすると、足裏の端で接触点探索の結果足の甲にいったときに、ほぼ最短接触点であるために接触点は変化せず、無理につま先立ちしようとしてIKがとけない、ということになる.
      constraint->debugLevel() = 0;
      constraint->updateBounds();

      param->bodyContactConstraints.push_back(constraint);
    }

  }
}
