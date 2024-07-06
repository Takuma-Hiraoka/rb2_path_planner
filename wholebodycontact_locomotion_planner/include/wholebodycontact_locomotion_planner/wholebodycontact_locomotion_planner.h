#ifndef WHOLEBODYCONTACT_LOCOMOTION_PLANNER_H
#define WHOLEBODYCONTACT_LOCOMOTION_PLANNER_H

#include <choreonoid_viewer/choreonoid_viewer.h>
#include <wholebodycontact_locomotion_planner/RobotState.h>
#include <ik_constraint2_body_contact/BodyContactConstraint.h>
#include <global_inverse_kinematics_solver/global_inverse_kinematics_solver.h>
#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>
#include <trajectory_optimizer/trajectory_optimizer.h>

namespace wholebodycontact_locomotion_planner{
  class WBLPParam {
  public:
    int debugLevel = 0;
    double expansionLength = 0.1; // contactableLinksをの凸包をこの長さだけ拡大したものをabstractRobotにする. このabstractRobotを使ってreachabilityConstraintを作ること.
    int maxSubGoalIdx = 3; // 接触が切り替わるか、maxSubGoalIdxぶん進んだ接触状態に完全に一致するように接触を定めていく. この数までは接触1つを進めるだけ進めてしまうので、大きすぎると(5,5)ならできたのに(10,5)ができずに計画失敗となってしまう.
    int maxContactIter = 1000; // 接触一つを動かしていく上限回数
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = nullptr;
    cnoid::BodyPtr robot;
    std::vector<cnoid::LinkPtr> variables;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals;
    std::unordered_map<std::string, std::shared_ptr<Mode> > modes;
    std::vector<std::shared_ptr<Contact> > currentContactPoints;
    std::vector<std::shared_ptr<ik_constraint2_body_contact::BodyContactConstraint> > bodyContactConstraints;
    std::vector<std::vector<cnoid::LinkPtr> > prioritizedLinks;
    global_inverse_kinematics_solver::GIKParam gikRootParam;
    global_inverse_kinematics_solver::GIKParam gikParam;
    bool OptimizeTrajectory = false; // 関節角度軌道を最適化、近いstate同士をショートカットするかどうか. もともと粗い軌道でありショートカットできる数は少なく、計算時間が増えるデメリットのほうが大きい.
    bool useSwingGIK = false; // TODO なぜか遅い
    trajectory_optimizer::TOParam toParam;

    WBLPParam() {
      gikRootParam.range = 0.05;
      gikRootParam.delta = 0.2;
      gikRootParam.timeout = 20;
      gikRootParam.maxTranslation = 3;
      gikRootParam.threads = 1;
      gikRootParam.goalBias = 0.2; // state生成後この確率で更にgoalへ近づくためにstateを作る。あまり大きいと局所最適解ばかり見つかって遅い
      gikRootParam.projectCellSize = 0.4; // 0.05よりも0.1の方が速い. 0.2より0.4のほうが速い? 2m * 2m * 2mの空間を動くとして、samplingを200個くらいまでにしたければ、cellの大きさもそれなりに大きくないとスカスカになってしまう.
      gikRootParam.pikParam.we = 3e1; // 逆運動学が振動しないこと優先. 1e0だと不安定. 1e3だと大きすぎる
      gikRootParam.pikParam.wmax = 3e0; // 1e2程度にすると関節がめり込まなくなるが、ほとんど動かない. TODO 体を小さくするタスクを明示的に与える？
      gikRootParam.pikParam.maxIteration = 100; // max iterationに達するか、convergeしたら終了する. isSatisfiedでは終了しない. ゼロ空間でreference angleに可能な限り近づけるタスクがあるので. 1 iterationで0.5msくらいかかるので、stateを1つ作るための時間の上限が見積もれる. 一見、この値を小さくすると早くなりそうだが、goalSampling時に本当はgoalに到達できるのにその前に返ってしまうことで遅くなることがあるため、少ないiterationでも収束するように他のパラメータを調整したほうがいい
      gikRootParam.pikParam.minIteration = 20;
      gikRootParam.pikParam.checkFinalState = true; // ゼロ空間でreference angleに可能な限り近づけるタスクのprecitionは大きくして、常にsatisfiedになることに注意
      gikRootParam.pikParam.calcVelocity = false; // 疎な軌道生成なので、velocityはチェックしない
      gikRootParam.pikParam.convergeThre = 5e-2; // 要パラチューン. IKConsraintのmaxErrorより小さくないと、収束誤判定する. maxErrorが5e-2の場合、5e-2だと大きすぎる. 5e-3だと小さすぎて時間がかかる. ikのwe, wn, wmax, maxErrorといったパラメータと連動してパラチューンせよ.
      gikRootParam.pikParam.pathOutputLoop = 5;

      gikParam = gikRootParam;
      gikParam.delta = 0.4;
      gikParam.projectCellSize = 0.05;
      gikParam.threads = 20;
      gikParam.timeout = 10; // 滑りがあるので遅いときは諦めて良い
      gikParam.goalBias = 0.05;
      /* gikParam.pikParam.we = 1e2; // 逆運動学が振動しないこと優先. 1e0だと不安定. 1e3だと大きすぎる */
      /* gikParam.pikParam.wmax = 1e1; // 1e2程度にすると関節がめり込まなくなるが、ほとんど動かない. */

      toParam.shortcutThre=4e-2;
    };
  };
  bool solveCBPath(const std::shared_ptr<Environment>& environment,
                   const cnoid::Isometry3 goal, // rootLink
                   const std::shared_ptr<WBLPParam>& param,
                   std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > >& outputPath
                   );
  bool solveWBLP(const std::shared_ptr<WBLPParam>& param,
                 const std::vector<std::pair<std::vector<double>,std::vector<std::shared_ptr<Contact> > > >& guidePath,
                 std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > >& outputPath // angle, contact
                 );
  std::unordered_map<std::string, std::vector<cnoid::Isometry3> >  createContactPoints(const std::shared_ptr<WBLPParam>& param,
                                                                                       std::string contactFileName);
}

#endif
