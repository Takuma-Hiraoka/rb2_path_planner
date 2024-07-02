#ifndef WORLD_COMMON_H
#define WORLD_COMMON_H

#include <cnoid/Body>
#include <wholebodycontact_locomotion_planner/RobotState.h>

namespace wholebodycontact_locomotion_planner_sample{

  void generateStepWorld(cnoid::BodyPtr& obstacle, // for visual
                         std::shared_ptr<wholebodycontact_locomotion_planner::Environment>& environment,
                         double height=0.3);
  void generateTunnelWorld(cnoid::BodyPtr& obstacle, // for visual
                         std::shared_ptr<wholebodycontact_locomotion_planner::Environment>& environment);
  void generateWallWorld(cnoid::BodyPtr& obstacle, // for visual
                         std::shared_ptr<wholebodycontact_locomotion_planner::Environment>& environment);

};

#endif
