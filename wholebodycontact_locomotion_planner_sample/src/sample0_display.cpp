#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <ros/package.h>

#include "samplerobot_common.h"
#include "world_common.h"

namespace wholebodycontact_locomotion_planner_sample{
  void sample0_display(){
    cnoid::BodyPtr obstacle;
    generateStepWorld(obstacle);
    cnoid::BodyPtr robot;
    cnoid::BodyPtr abstractRobot;
    generateSampleRobot(robot, abstractRobot);

    // setup viewer
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    viewer->objects(robot);
    viewer->objects(abstractRobot);
    viewer->objects(obstacle);

    viewer->drawObjects();
  }
}
