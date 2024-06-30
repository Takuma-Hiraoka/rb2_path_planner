#ifndef SAMPLEROBOT_COMMON_H
#define SAMPLEROBOT_COMMON_H

#include <cnoid/Body>
#include <wholebodycontact_locomotion_planner/RobotState.h>
#include <wholebodycontact_locomotion_planner/wholebodycontact_locomotion_planner.h>

namespace wholebodycontact_locomotion_planner_sample{

  void generateSampleRobot(const std::shared_ptr<moveit_extensions::InterpolatedPropagationDistanceField>& field,
                           std::shared_ptr<wholebodycontact_locomotion_planner::WBLPParam>& param,
                           cnoid::BodyPtr& abstractRobot, // for visual. BulletKeepCollisionConstraint時、*_bulletModelの作成にはこのcollisionShapeを使いつつ、*_link_bulletModel()には*_linkを入れて、探索変数にもrobotの変数だけを使う.
                           bool quadruped=false
                           );

};

#endif
