#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <ros/package.h>

#include "samplerobot_common.h"
#include "world_common.h"

namespace wholebodycontact_locomotion_planner_sample{
  void sample2_tunnel(){
    cnoid::BodyPtr obstacle;
    std::shared_ptr<wholebodycontact_locomotion_planner::Environment> environment;
    generateTunnelWorld(obstacle, environment);

    std::shared_ptr<wholebodycontact_locomotion_planner::WBLPParam> param = std::make_shared<wholebodycontact_locomotion_planner::WBLPParam>();
    cnoid::BodyPtr abstractRobot;
    generateSampleRobot(environment->obstacles, param, abstractRobot, true);
    std::vector<double> initialPose;
    global_inverse_kinematics_solver::link2Frame(param->variables, initialPose);

    // setup viewer
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    viewer->objects(param->robot);
    viewer->objects(abstractRobot);
    viewer->objects(obstacle);

    viewer->drawObjects();

    param->debugLevel=3;
    param->viewer = viewer;
    param->gikRootParam.range = 0.5;
    param->gikRootParam.timeout = 30;
    param->gikRootParam.viewer = viewer;
    param->gikRootParam.drawLoop = 1;
    param->gikRootParam.debugLevel = 0;
    param->gikRootParam.pikParam.debugLevel = 0;
    param->gikRootParam.timeout = 20;
    param->gikRootParam.threads = 20;
    param->OptimizeTrajectory = true; // 軌道最適化を行うと外れ値的な接触や接触の数自体を減らせる
    param->toParam.shortcut = true;

    cnoid::Isometry3 goal = param->robot->rootLink()->T();
    goal.translation()[0] += 3.3;
    goal.translation()[2] -= 0.2;

    std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<wholebodycontact_locomotion_planner::Contact> > > > path;

    wholebodycontact_locomotion_planner::solveCBPath(environment,
                                                     goal,
                                                     param,
                                                     path);
    // variables
    std::vector<cnoid::LinkPtr> abstractVariables;
    abstractVariables.push_back(abstractRobot->rootLink());
    for(int i=0;i<abstractRobot->numJoints();i++){
      abstractVariables.push_back(abstractRobot->joint(i));
    }

    for(int i=0;i<path.size();i++){
      global_inverse_kinematics_solver::frame2Link(path.at(i).first,param->variables);
      param->robot->calcForwardKinematics(false);
      param->robot->calcCenterOfMass();
      global_inverse_kinematics_solver::frame2Link(path.at(i).first,abstractVariables);
      abstractRobot->calcForwardKinematics(false);
      abstractRobot->calcCenterOfMass();
      viewer->drawObjects();
      std::cerr << "contacts :";
      for (int j=0; j<path.at(i).second.size(); j++) std::cerr << " " << path.at(i).second[j]->name;
      std::cerr << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<wholebodycontact_locomotion_planner::Contact> > > > contactPath;
    global_inverse_kinematics_solver::frame2Link(initialPose,param->variables);
    param->robot->calcForwardKinematics(false);
    param->robot->calcCenterOfMass();
    bool solved = wholebodycontact_locomotion_planner::solveWBLP(param,
                                                                 path,
                                                                 contactPath);
    if (!solved) return;

    while(true) {
      // main loop
      for(int i=0;i<path.size();i++){
        global_inverse_kinematics_solver::frame2Link(path.at(i).first,param->variables);
        param->robot->calcForwardKinematics(false);
        param->robot->calcCenterOfMass();
        global_inverse_kinematics_solver::frame2Link(path.at(i).first,abstractVariables);
        abstractRobot->calcForwardKinematics(false);
        abstractRobot->calcCenterOfMass();
        viewer->drawObjects();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      global_inverse_kinematics_solver::frame2Link(initialPose,param->variables);
      param->robot->calcForwardKinematics(false);
      param->robot->calcCenterOfMass();
      for(int i=0;i<contactPath.size();i++){
        global_inverse_kinematics_solver::frame2Link(contactPath.at(i).first,param->variables);
        param->robot->calcForwardKinematics(false);
        param->robot->calcCenterOfMass();
        global_inverse_kinematics_solver::frame2Link(contactPath.at(i).first,abstractVariables);
        abstractRobot->calcForwardKinematics(false);
        abstractRobot->calcCenterOfMass();
        viewer->drawObjects();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
    }
  }
}
