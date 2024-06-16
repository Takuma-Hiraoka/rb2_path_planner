#ifndef SAMPLEROBOT_COMMON_H
#define SAMPLEROBOT_COMMON_H

#include <cnoid/Body>

namespace wholebodycontact_locomotion_planner_sample{

  void generateSampleRobot(cnoid::BodyPtr& robot,
                           cnoid::BodyPtr& abstractRobot
                           );

};

#endif
