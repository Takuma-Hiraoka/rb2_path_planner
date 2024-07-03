#include <wholebodycontact_locomotion_planner/RobotState.h>
#include <wholebodycontact_locomotion_planner/Util.h>
#include <choreonoid_cddlib/choreonoid_cddlib.h>

namespace wholebodycontact_locomotion_planner {
  std::shared_ptr<ik_constraint2::IKConstraint> Mode::generateCondition(const std::shared_ptr<Environment>& environment, const cnoid::BodyPtr& robot){
    std::shared_ptr<ik_constraint2_keep_collision_scfr::KeepCollisionScfrConstraint> keepScfrConstraint = std::make_shared<ik_constraint2_keep_collision_scfr::KeepCollisionScfrConstraint>();
    std::shared_ptr<ik_constraint2_scfr::ScfrConstraint> scfrConstraint = std::make_shared<ik_constraint2_scfr::ScfrConstraint>();
    scfrConstraint->A_robot() = robot;
    keepScfrConstraint->minimumContactCount() = 3;
    keepScfrConstraint->breakableSCFRParam().lpTolerance = 1e-7;
    scfrConstraint->SCFRParam().eps = 0.2; // default 0.05 15点程度で接していても2ms程度で解ける
    scfrConstraint->SCFRParam().lpTolerance = 1e-7;
    //    scfrConstraint->debugLevel() = 2;
    scfrConstraint->maxCError() = 0.1;
    keepScfrConstraint->scfrConstraint() = scfrConstraint;
    for(int i=0;i<this->reachabilityConstraints.size();i++){
      keepScfrConstraint->keepCollisionConstraints().push_back(this->reachabilityConstraints[i]);
    }
    keepScfrConstraint->updateBounds();
    //    keepScfrConstraint->debugLevel() = 2;
    //    scfrConstraint->SCFRparam().debugLevel = 1;
    return keepScfrConstraint;
  }
  void Contact::calcBoundingBox() {
    cnoid::SgMeshPtr mesh = convertToSgMesh(this->link1->collisionShape());
    if(mesh && (mesh->numTriangles() != 0)) {
      mesh->updateBoundingBox();
      this->bbx = mesh->boundingBox();
    }
  }
}
