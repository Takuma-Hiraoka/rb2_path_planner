#ifndef WHOLEBODYCONTACT_LOCOMOTION_PLANNER_UTIL_H
#define WHOLEBODYCONTACT_LOCOMOTION_PLANNER_UTIL_H

#include <wholebodycontact_locomotion_planner/wholebodycontact_locomotion_planner.h>
#include <global_inverse_kinematics_solver/global_inverse_kinematics_solver.h>
#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>

namespace wholebodycontact_locomotion_planner{
  enum class IKState
    {
      DETACH_FIXED, // ついている接触を離す, 接触ローカル位置は前回ついていた場所
      DETACH, // ついている接触を離す, 接触ローカル位置は探索された場所
      ATTACH_FIXED, // 離れている接触をつける, 接触ローカル位置は前回ついていた場所
      ATTACH, // 離れている接触をつける, 接触ローカル位置は探索された場所
      SWING, // 離れている接触を離れたまま移動する, 接触ローカル位置を探索する
      CONTACT_SEARCH, // 接触は離れているが、ついているものとして重心制約を作る. 接触ローカル位置を探索する
      DETACH_SEARCH, // 接触は離れているが、ついているものとして重心制約を作る. 接触ローカル位置は探索された場所. 大域探索を行うため
      SLIDE, // ついている接触をついたまま移動する, 接触ローカル位置は前回ついていた場所
    };
  bool solveContactIK(const std::shared_ptr<WBLPParam>& param,
                      const std::vector<std::shared_ptr<Contact> >& stopContact,
                      const std::vector<std::shared_ptr<Contact> >& nextContacts,
                      const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominals,
                      const IKState ikState);
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
