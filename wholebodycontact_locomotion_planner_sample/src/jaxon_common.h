#ifndef JAXON_COMMON_H
#define JAXON_COMMON_H

#include <cnoid/Body>
#include <wholebodycontact_locomotion_planner/RobotState.h>
#include <wholebodycontact_locomotion_planner/Util.h>
#include <wholebodycontact_locomotion_planner/wholebodycontact_locomotion_planner.h>

namespace wholebodycontact_locomotion_planner_sample{

  void generateJAXON(const std::shared_ptr<moveit_extensions::InterpolatedPropagationDistanceField>& field,
                           std::shared_ptr<wholebodycontact_locomotion_planner::WBLPParam>& param,
                           cnoid::BodyPtr& abstractRobot,
                           bool quadruped=false
                           );

};


#endif
