#include <wholebodycontact_locomotion_planner/RobotState.h>
#include <choreonoid_cddlib/choreonoid_cddlib.h>
#include <ik_constraint2_scfr/KeepCollisionScfrConstraint.h>

namespace wholebodycontact_locomotion_planner {
  std::shared_ptr<ik_constraint2::IKConstraint> Mode::generateCondition(const std::shared_ptr<Environment>& environment, const cnoid::BodyPtr& robot){
    std::shared_ptr<ik_constraint2::ANDConstraint> conditions = std::make_shared<ik_constraint2::ANDConstraint>();
    //conditions->debugLevel() = 2;
    for(int i=0;i<this->collisionConstraints.size();i++){
      conditions->children().push_back(this->collisionConstraints[i]);
    }
    std::shared_ptr<ik_constraint2::ORConstraint> reachabilityConditions = std::make_shared<ik_constraint2::ORConstraint>();
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
      constraint->updateBounds(); // キャッシュを内部に作る.
      reachabilityConditions->children().push_back(constraint);
    }
    conditions->children().push_back(reachabilityConditions);

    std::shared_ptr<ik_constraint2_keep_collision_scfr::KeepCollisionScfrConstraint> scfrConstraint = std::make_shared<ik_constraint2_keep_collision_scfr::KeepCollisionScfrConstraint>();
    for(int i=0;i<this->reachabilityConstraints.size();i++){
      scfrConstraint->A_robot() = robot;
      scfrConstraint->keepCollisionConstraints().push_back(this->reachabilityConstraints[i]);
    }
    conditions->children().push_back(scfrConstraint);
    return conditions;
  }
}
