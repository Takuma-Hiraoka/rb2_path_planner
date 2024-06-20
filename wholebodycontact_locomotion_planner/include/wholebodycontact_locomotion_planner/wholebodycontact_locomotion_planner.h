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
      gikRootParam.range = 0.05;
      gikRootParam.delta = 0.2;
      gikRootParam.timeout = 20;
      gikRootParam.maxTranslation = 3;
      gikRootParam.threads = 1;
      gikRootParam.goalBias = 0.2;
      gikRootParam.projectCellSize = 0.4; // 0.05よりも0.1の方が速い. 0.2より0.4のほうが速い? 2m * 2m * 2mの空間を動くとして、samplingを200個くらいまでにしたければ、cellの大きさもそれなりに大きくないとスカスカになってしまう.
      gikRootParam.pikParam.we = 5e2; // 逆運動学が振動しないこと優先. 1e0だと不安定. 1e3だと大きすぎる
      gikRootParam.pikParam.maxIteration = 100; // max iterationに達するか、convergeしたら終了する. isSatisfiedでは終了しない. ゼロ空間でreference angleに可能な限り近づけるタスクがあるので. 1 iterationで0.5msくらいかかるので、stateを1つ作るための時間の上限が見積もれる. 一見、この値を小さくすると早くなりそうだが、goalSampling時に本当はgoalに到達できるのにその前に返ってしまうことで遅くなることがあるため、少ないiterationでも収束するように他のパラメータを調整したほうがいい
      gikRootParam.pikParam.minIteration = 10;
      gikRootParam.pikParam.checkFinalState = true; // ゼロ空間でreference angleに可能な限り近づけるタスクのprecitionは大きくして、常にsatisfiedになることに注意
      gikRootParam.pikParam.calcVelocity = false; // 疎な軌道生成なので、velocityはチェックしない
      gikRootParam.pikParam.convergeThre = 5e-2; // 要パラチューン. IKConsraintのmaxErrorより小さくないと、収束誤判定する. maxErrorが5e-2の場合、5e-2だと大きすぎる. 5e-3だと小さすぎて時間がかかる. ikのwe, wn, wmax, maxErrorといったパラメータと連動してパラチューンせよ.
      gikRootParam.pikParam.pathOutputLoop = 5;

    };
  };
  bool solveCBPath(const std::shared_ptr<Environment>& environment,
                   const cnoid::Isometry3 goal, // rootLink
                   const std::shared_ptr<WBLPParam>& param,
                   std::vector<std::vector<double> >& outputPath
                   );
}

#endif
