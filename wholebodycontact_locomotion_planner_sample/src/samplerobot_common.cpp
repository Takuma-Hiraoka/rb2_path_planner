#include "samplerobot_common.h"

#include <cnoid/BodyLoader>
#include <ros/package.h>
#include <iostream>
#include <choreonoid_bullet/choreonoid_bullet.h>
#include <choreonoid_cddlib/choreonoid_cddlib.h>
#include <cnoid/MeshExtractor>
#include <ik_constraint2_vclip/ik_constraint2_vclip.h>

namespace wholebodycontact_locomotion_planner_sample{
  void generateSampleRobot(const std::shared_ptr<moveit_extensions::InterpolatedPropagationDistanceField>& field,
                           std::shared_ptr<wholebodycontact_locomotion_planner::WBLPParam>& param,
                           cnoid::BodyPtr& abstractRobot, // for visual
                           bool quadruped
                           ) {
    std::vector<std::string> contactableLinkNames{
        "LLEG_ANKLE_R",
        "RLEG_ANKLE_R",
        // "LLEG_HIP_Y",
        // "RLEG_HIP_Y",
        // "LLEG_KNEE",
        // "RLEG_KNEE",
        "LARM_WRIST_R",
        "RARM_WRIST_R",
        // "LARM_SHOULDER_R",
        // "RARM_SHOULDER_R",
        // "LARM_ELBOW",
        // "RARM_ELBOW",
        // "WAIST",
        // "WAIST_R",
        // "CHEST"
        };

    cnoid::BodyLoader bodyLoader;
    param->robot = bodyLoader.load(ros::package::getPath("choreonoid") + "/share/model/SR1/SR1.body");

    param->robot->link("RLEG_HIP_P")->setJointRange(-150.0/180.0*M_PI, 30.0/180.0*M_PI);
    param->robot->link("LLEG_HIP_P")->setJointRange(-150.0/180.0*M_PI, 30.0/180.0*M_PI);
    param->robot->link("RLEG_KNEE")->setJointRange(param->robot->link("RLEG_KNEE")->q_lower(), 150.0/180.0*M_PI);
    param->robot->link("LLEG_KNEE")->setJointRange(param->robot->link("LLEG_KNEE")->q_lower(), 150.0/180.0*M_PI);

    param->robot->rootLink()->p() = cnoid::Vector3(0,0,0.65);
    param->robot->rootLink()->v().setZero();
    param->robot->rootLink()->R() = cnoid::Matrix3::Identity();
    param->robot->rootLink()->w().setZero();

    std::vector<double> reset_manip_pose{
      0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0,// rleg
        0.523599, 0.0, 0.0, -1.74533, 0.15708, -0.113446, 0.637045,// rarm
        0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0,// lleg
        0.523599, 0.0, 0.0, -1.74533, -0.15708, -0.113446, -0.637045,// larm
        0.1, 0.0, 0.0}; // torso. waist-pを少し前に傾けておくと、後ろにひっくり返りにくくなる
    if (quadruped) {
      param->robot->rootLink()->p() = cnoid::Vector3(-0.3,0,0.55);
      param->robot->rootLink()->v().setZero();
      param->robot->rootLink()->R() = cnoid::rotFromRpy(0.0,M_PI/2,0.0);
      param->robot->rootLink()->w().setZero();
      std::vector<double> quadruped_reset_manip_pose{
        0.0, -2.349066, 0.0, 1.420305, -0.671239, 0.0,// rleg
          -1.523599, 0.0, 0.0, -0.74533, 0.15708, -1.013446, 0.637045,// rarm
          0.0, -2.349066, 0.0, 1.420305, -0.671239, 0.0,// lleg
          -1.523599, 0.0, 0.0, -0.74533, -0.15708, -1.013446, -0.637045,// larm
          0.1, 0.0, 0.0};
      reset_manip_pose = quadruped_reset_manip_pose;
    }
    for(int j=0; j < param->robot->numJoints(); ++j){
      param->robot->joint(j)->q() = reset_manip_pose[j];
    }

    param->robot->calcForwardKinematics();
    param->robot->calcCenterOfMass();

    {
      // variables
      param->variables.push_back(param->robot->rootLink());
      for(int i=0;i<param->robot->numJoints();i++){
        param->variables.push_back(param->robot->joint(i));
      }
    }

    {
      // task: nominal constairnt
      for(int i=0;i<param->robot->numJoints();i++){
        std::shared_ptr<ik_constraint2::JointAngleConstraint> constraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
        constraint->joint() = param->robot->joint(i);
        constraint->targetq() = reset_manip_pose[i];
        constraint->precision() = 1e10; // always satisfied
        param->nominals.push_back(constraint);
      }
    }

    wholebodycontact_locomotion_planner::createAbstractRobot(param, contactableLinkNames, abstractRobot);
    // currentContactPoint
    param->currentContactPoints.clear();
    {
      {
        std::shared_ptr<wholebodycontact_locomotion_planner::Contact> lleg = std::make_shared<wholebodycontact_locomotion_planner::Contact>();
        lleg->name = "LLEG_ANKLE_R";
        lleg->link1 = param->robot->link("LLEG_ANKLE_R");
        lleg->localPose1.translation() = cnoid::Vector3(0.0,0.0,-0.055);
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
        du[0] = 20000.0;
        lleg->du = du;
        param->currentContactPoints.push_back(lleg);
      }
      {
        std::shared_ptr<wholebodycontact_locomotion_planner::Contact> rleg = std::make_shared<wholebodycontact_locomotion_planner::Contact>();
        rleg->name = "RLEG_ANKLE_R";
        rleg->link1 = param->robot->link("RLEG_ANKLE_R");
        rleg->localPose1.translation() = cnoid::Vector3(0.0,0.0,-0.055);
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
        du[0] = 20000.0;
        rleg->du = du;
        param->currentContactPoints.push_back(rleg);
      }
      if (quadruped) {
        {
          std::shared_ptr<wholebodycontact_locomotion_planner::Contact> larm = std::make_shared<wholebodycontact_locomotion_planner::Contact>();
          larm->name = "LARM_WRIST_R";
          larm->link1 = param->robot->link("LARM_WRIST_R");
          larm->localPose1.translation() = cnoid::Vector3(-0.025,0.0,-0.195);
          larm->localPose1.linear() = cnoid::rotFromRpy(0,M_PI /2,0);
          larm->localPose2 = larm->link1->T() * larm->localPose1;
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
          larm->C = C;
          cnoid::VectorX dl = Eigen::VectorXd::Zero(11);
          larm->dl = dl;
          cnoid::VectorX du = 1e10 * Eigen::VectorXd::Ones(11);
          du[0] = 20000.0;
          larm->du = du;
          param->currentContactPoints.push_back(larm);
        }
        {
          std::shared_ptr<wholebodycontact_locomotion_planner::Contact> rarm = std::make_shared<wholebodycontact_locomotion_planner::Contact>();
          rarm->name = "RARM_WRIST_R";
          rarm->link1 = param->robot->link("RARM_WRIST_R");
          rarm->localPose1.translation() = cnoid::Vector3(-0.025,0.0,-0.195);
          rarm->localPose1.linear() = cnoid::rotFromRpy(0,M_PI /2,0);
          rarm->localPose2 = rarm->link1->T() * rarm->localPose1;
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
          rarm->C = C;
          cnoid::VectorX dl = Eigen::VectorXd::Zero(11);
          rarm->dl = dl;
          cnoid::VectorX du = 1e10 * Eigen::VectorXd::Ones(11);
          du[0] = 20000.0;
          rarm->du = du;
          param->currentContactPoints.push_back(rarm);
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
      if(param->robot->link(i)->name() == "LLEG_ANKLE_P" ||
         param->robot->link(i)->name() == "RLEG_ANKLE_P" ||
         param->robot->link(i)->name() == "LLEG_HIP_P" ||
         param->robot->link(i)->name() == "RLEG_HIP_P" ||
         param->robot->link(i)->name() == "LLEG_HIP_R" ||
         param->robot->link(i)->name() == "RLEG_HIP_R" ||
         param->robot->link(i)->name() == "LARM_WRIST_P" ||
         param->robot->link(i)->name() == "RARM_WRIST_P" ||
         param->robot->link(i)->name() == "LARM_WRIST_Y" ||
         param->robot->link(i)->name() == "RARM_WRIST_Y" ||
         param->robot->link(i)->name() == "LARM_SHOULDER_Y" ||
         param->robot->link(i)->name() == "RARM_SHOULDER_Y" ||
         param->robot->link(i)->name() == "LARM_SHOULDER_P" ||
         param->robot->link(i)->name() == "RARM_SHOULDER_P" ||
         param->robot->link(i)->name() == "WAIST_P") continue;
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
      std::vector<std::string> rarm{"RARM_SHOULDER_R", "RARM_ELBOW", "RARM_WRIST_R"};
      std::vector<std::string> larm{"LARM_SHOULDER_R", "LARM_ELBOW", "LARM_WRIST_R"};
      std::vector<std::string> rleg{"RLEG_HIP_Y", "RLEG_KNEE"/*, "RLEG_ANKLE_R"*/};
      std::vector<std::string> lleg{"LLEG_HIP_Y", "LLEG_KNEE"/*, "LLEG_ANKLE_R"*/};
      std::vector<std::string> torso{"WAIST", "WAIST_R", "CHEST"};

      std::vector<std::vector<std::string> > pairs;
      for(int i=0;i<rarm.size();i++) for(int j=0;j<larm.size();j++) pairs.push_back(std::vector<std::string>{rarm[i],larm[j]});
      for(int i=0;i<rarm.size();i++) for(int j=0;j<rleg.size();j++) pairs.push_back(std::vector<std::string>{rarm[i],rleg[j]});
      for(int i=0;i<rarm.size();i++) for(int j=0;j<lleg.size();j++) pairs.push_back(std::vector<std::string>{rarm[i],lleg[j]});
      for(int i=0;i<rarm.size();i++) for(int j=0;j<torso.size();j++) pairs.push_back(std::vector<std::string>{rarm[i],torso[j]});
      for(int i=0;i<larm.size();i++) for(int j=0;j<rleg.size();j++) pairs.push_back(std::vector<std::string>{larm[i],rleg[j]});
      for(int i=0;i<larm.size();i++) for(int j=0;j<lleg.size();j++) pairs.push_back(std::vector<std::string>{larm[i],lleg[j]});
      for(int i=0;i<larm.size();i++) for(int j=0;j<torso.size();j++) pairs.push_back(std::vector<std::string>{larm[i],torso[j]});
      for(int i=0;i<rleg.size();i++) for(int j=0;j<lleg.size();j++) pairs.push_back(std::vector<std::string>{rleg[i],lleg[j]});

      for(int i=0;i<pairs.size();i++){
        std::shared_ptr<ik_constraint2_vclip::VclipCollisionConstraint> constraint = std::make_shared<ik_constraint2_vclip::VclipCollisionConstraint>();
        constraint->A_link() = param->robot->link(pairs[i][0]);
        constraint->B_link() = param->robot->link(pairs[i][1]);
        constraint->tolerance() = 0.01;
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
          constraint->field() = field;
          constraint->tolerance() = 0.3; // ちょうど干渉すると法線ベクトルが変になることがあるので, 1回のiterationで動きうる距離よりも大きくせよ.
          //          constraint->precision() = 0.01;
          constraint->ignoreDistance() = 0.5; // 大きく動くので、ignoreも大きくする必要がある
          //      constraint->maxError() = 0.1; // めり込んだら一刻も早く離れたい
          constraint->invert() = true; // 距離をtolerance以下にする
          constraint->updateBounds(); // キャッシュを内部に作る. キャッシュを作ったあと、10スレッドぶんコピーする方が速い
          mode->reachabilityConstraints.push_back(constraint);
        }
      }
    } // mode1

    param->prioritizedLinks.clear();
    std::vector<cnoid::LinkPtr> prioritizedLinks1{param->robot->link("LLEG_ANKLE_R"),param->robot->link("RLEG_ANKLE_R")};
    std::vector<cnoid::LinkPtr> prioritizedLinks2{param->robot->link("LARM_WRIST_R"),param->robot->link("LARM_WRIST_R"),param->robot->link("WAIST"),param->robot->link("CHEST")};
    std::vector<cnoid::LinkPtr> prioritizedLinks3{param->robot->link("LLEG_KNEE"),param->robot->link("RLEG_KNEE"),param->robot->link("LLEG_HIPY"),param->robot->link("RLEG_HIPY"),param->robot->link("LARM_ELBOW"),param->robot->link("RARM_ELBOW"),param->robot->link("LARM_SHOULDER_R"),param->robot->link("RARM_SHOULDER_R"),param->robot->link("WAIST_R")};
    param->prioritizedLinks.push_back(prioritizedLinks1);
    param->prioritizedLinks.push_back(prioritizedLinks2);
    param->prioritizedLinks.push_back(prioritizedLinks3);

    // bodyContactConstraint
    std::string contactFileName = ros::package::getPath("wholebodycontact_locomotion_planner_sample") + "/config/sample_config.yaml";
    std::unordered_map<std::string, std::vector<cnoid::Isometry3> > contactPoints = wholebodycontact_locomotion_planner::createContactPoints(param, contactFileName);
    for (int i=0; i<param->robot->numLinks(); i++) {
      if(std::find(contactableLinkNames.begin(),contactableLinkNames.end(),param->robot->link(i)->name()) == contactableLinkNames.end()) continue;
      cnoid::LinkPtr variable = new cnoid::Link();
      cnoid::BodyPtr body = new cnoid::Body();
      body->setRootLink(variable);
      variable->setJointType(cnoid::Link::JointType::FreeJoint);
      std::shared_ptr<ik_constraint2_body_contact::BodyContactConstraint> constraint = std::make_shared<ik_constraint2_body_contact::BodyContactConstraint>();
      constraint->A_link() = param->robot->link(i);
      constraint->B_link() = nullptr;
      constraint->contact_pos_link() = variable;
      constraint->contact_pos_link()->T() = constraint->A_localpos();
      constraint->contact_pos_body() = body;
      for(std::unordered_map<std::string, std::vector<cnoid::Isometry3> >::const_iterator it=contactPoints.begin(); it!=contactPoints.end(); it++){
        if (it->first==param->robot->link(i)->name()) constraint->setContactPoints(it->second, 0.05, 16);
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
