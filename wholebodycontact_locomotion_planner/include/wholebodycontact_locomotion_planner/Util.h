#ifndef WHOLEBODYCONTACT_LOCOMOTION_PLANNER_UTIL_H
#define WHOLEBODYCONTACT_LOCOMOTION_PLANNER_UTIL_H

#include <wholebodycontact_locomotion_planner/wholebodycontact_locomotion_planner.h>
#include <global_inverse_kinematics_solver/global_inverse_kinematics_solver.h>
#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>

namespace wholebodycontact_locomotion_planner{
  bool solveContactIK(const std::shared_ptr<WBLPParam>& param,
                      const std::vector<std::shared_ptr<Contact> >& stopContact,
                      const std::vector<std::shared_ptr<Contact> >& nextContacts,
                      const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominals,
                      bool attach,
                      bool slide,
                      bool lift=false);
  void calcIgnoreBoundingBox(const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& constraints,
                             const std::shared_ptr<Contact>& contact,
                             int level=1
                             ); // constraint中のcollisionConstraintについて、contactのlink1のlevel等親のリンクの干渉回避である場合、contactのlink1のBoundingBoxを追加する.
  void calcIgnoreBoundingBox(const std::vector<std::shared_ptr<ik_constraint2_distance_field::DistanceFieldCollisionConstraint> >& constraints,
                             const std::shared_ptr<Contact>& contact,
                             int level=1
                             ); // constraint中のcollisionConstraintについて、contactのlink1のlevel等親のリンクの干渉回避である場合、contactのlink1のBoundingBoxを追加する.
  bool calcContactPoint(const std::shared_ptr<WBLPParam>& param,
                        const std::vector<std::shared_ptr<Contact> >& attachContacts
                        );
  cnoid::SgMeshPtr convertToSgMesh (const cnoid::SgNodePtr collisionshape);

  void createAbstractRobot(const std::shared_ptr<WBLPParam>& param,
                           const std::vector<std::string> contactableLinkNames,
                           cnoid::BodyPtr& abstractRobot
                           );
}

#endif
