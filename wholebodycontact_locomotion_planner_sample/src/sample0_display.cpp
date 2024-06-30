#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <ros/package.h>

#include "samplerobot_common.h"
#include "world_common.h"

namespace wholebodycontact_locomotion_planner_sample{
  void sample0_display(){
    cnoid::BodyPtr obstacle;
    std::shared_ptr<wholebodycontact_locomotion_planner::Environment> environment;
    generateStepWorld(obstacle, environment);

    std::shared_ptr<wholebodycontact_locomotion_planner::WBLPParam> param = std::make_shared<wholebodycontact_locomotion_planner::WBLPParam>();
    cnoid::BodyPtr abstractRobot;
    generateSampleRobot(environment->obstacles, param, abstractRobot, true);

    // setup viewer
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    viewer->objects(param->robot);
    viewer->objects(abstractRobot);
    viewer->objects(obstacle);
    viewer->objects(environment->surfacesBody);

    viewer->drawObjects();
  }
}
