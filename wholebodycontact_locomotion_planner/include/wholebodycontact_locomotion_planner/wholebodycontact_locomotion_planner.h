#ifndef WHOLEBODYCONTACT_LOCOMOTION_PLANNER_H
#define WHOLEBODYCONTACT_LOCOMOTION_PLANNER_H

#include <choreonoid_viewer/choreonoid_viewer.h>
#include <wholebodycontact_locomotion_planner/RobotState.h>
#include <global_inverse_kinematics_solver/global_inverse_kinematics_solver.h>

namespace wholebodycontact_locomotion_planner{
  class WBLPParam {
  public:
    int debugLevel = 0;
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = nullptr;
    cnoid::BodyPtr robot;
    std::vector<cnoid::LinkPtr> variables;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals;
    std::unordered_map<std::string, std::shared_ptr<Mode> > modes;
    global_inverse_kinematics_solver::GIKParam gikRootParam;

    WBLPParam() {
      gikRootParam.range = 0.05; //大きすぎるとめり込む. 0.03は小さすぎて遅い
      gikRootParam.delta = 0.2;
      gikRootParam.timeout = 20;
      gikRootParam.maxTranslation = 3;
      gikRootParam.threads = 10;
      gikRootParam.goalBias = 0.2;
    };
  };
  bool solveCBPath(const std::shared_ptr<Environment>& environment,
                   const cnoid::Isometry3 goal, // rootLink
                   const std::shared_ptr<WBLPParam>& param,
                   std::vector<std::vector<double> >& outputPath
                   );
}

#endif
