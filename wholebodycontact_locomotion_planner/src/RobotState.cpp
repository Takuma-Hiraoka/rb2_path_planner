#include <wholebodycontact_locomotion_planner/RobotState.h>
#include <choreonoid_cddlib/choreonoid_cddlib.h>
#include <ik_constraint2_scfr/KeepCollisionScfrConstraint.h>

namespace wholebodycontact_locomotion_planner {
  std::shared_ptr<ik_constraint2::IKConstraint> Mode::generateCondition(const std::shared_ptr<Environment>& environment, const cnoid::BodyPtr& robot){
    std::shared_ptr<ik_constraint2_keep_collision_scfr::KeepCollisionScfrConstraint> keepScfrConstraint = std::make_shared<ik_constraint2_keep_collision_scfr::KeepCollisionScfrConstraint>();
    std::shared_ptr<ik_constraint2_scfr::ScfrConstraint> scfrConstraint = std::make_shared<ik_constraint2_scfr::ScfrConstraint>();
    scfrConstraint->A_robot() = robot;
    keepScfrConstraint->breakableSCFRParam().maxIter = 0; // vertexを増やす回数. 5だと6接触で6ms程度. そもそも重心の実行可能領域があるかどうかを知りたいので. はじめの4点が取れればそれで十分
    keepScfrConstraint->minimumContactCount() = 3;
    keepScfrConstraint->breakableSCFRParam().lpTolerance = 1e-7;
    scfrConstraint->SCFRParam().eps = 0.2; // default 0.05 15点程度で接していても2ms程度で解ける
    scfrConstraint->SCFRParam().lpTolerance = 1e-7;
    //    scfrConstraint->debugLevel() = 2;
    scfrConstraint->maxCError() = 0.1;
    keepScfrConstraint->scfrConstraint() = scfrConstraint;
    for(int i=0;i<this->reachabilityConstraints.size();i++){
      std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = this->reachabilityConstraints[i];
      constraint->B_link() = environment->surfacesBody->rootLink();
      constraint->B_link_bulletModel() = constraint->B_link();
      constraint->B_bulletModel() = environment->surfacesBulletModel;
      constraint->useSingleMeshB() = false; // support polygonを個別にチェック
      choreonoid_cddlib::convertToFACEExpressions(constraint->B_link()->collisionShape(),
                                                  constraint->B_FACE_C(),
                                                  constraint->B_FACE_dl(),
                                                  constraint->B_FACE_du());
      keepScfrConstraint->keepCollisionConstraints().push_back(constraint);
    }
    keepScfrConstraint->updateBounds();
    //    keepScfrConstraint->debugLevel() = 2;
    //    scfrConstraint->SCFRparam().debugLevel = 1;
    return keepScfrConstraint;
  }
}
