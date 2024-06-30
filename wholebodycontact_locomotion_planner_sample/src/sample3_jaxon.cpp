#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <ros/package.h>

#include "samplerobot_common.h"
#include "world_common.h"

namespace wholebodycontact_locomotion_planner_sample{
  void sample3_jaxon(){
    cnoid::BodyPtr obstacle;
    std::shared_ptr<wholebodycontact_locomotion_planner::Environment> environment;
    generateTunnelWorld(obstacle, environment);

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

    // setup viewer
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    viewer->objects(param->robot);
    viewer->objects(abstractRobot);
    viewer->objects(obstacle);
    viewer->objects(environment->surfacesBody);
    viewer->drawObjects();

  }
}
