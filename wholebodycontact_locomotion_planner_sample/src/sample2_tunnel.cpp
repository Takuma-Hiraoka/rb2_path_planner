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
    generateSampleRobot(environment->obstacles, param, abstractRobot);

    // setup viewer
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    viewer->objects(param->robot);
    viewer->objects(abstractRobot);
    viewer->objects(obstacle);
    viewer->objects(environment->surfacesBody);

    viewer->drawObjects();

    param->gikRootParam.range = 0.5;
    param->gikRootParam.timeout = 30;
    param->gikRootParam.viewer = viewer;
    param->gikRootParam.drawLoop = 1;
    param->gikRootParam.debugLevel = 0;
    param->gikRootParam.pikParam.debugLevel = 0;
    param->gikRootParam.timeout = 20;
    param->gikRootParam.threads = 10;

    cnoid::Isometry3 goal = param->robot->rootLink()->T();
    goal.translation()[0] += 3.3;
    goal.translation()[2] -= 0.2;

    std::vector<std::pair<std::vector<double>, std::string> > path;

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
    }
  }
}
