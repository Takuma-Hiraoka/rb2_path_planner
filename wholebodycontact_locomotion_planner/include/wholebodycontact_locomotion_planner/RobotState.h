#ifndef WHOLEBODYCONTACT_LOCOMOTION_PLANNER_ROBOTSTATE_H
#define WHOLEBODYCONTACT_LOCOMOTION_PLANNER_ROBOTSTATE_H

#include <cnoid/Body>
#include <ik_constraint2/ik_constraint2.h>
#include <ik_constraint2_distance_field/ik_constraint2_distance_field.h>
#include <ik_constraint2_scfr/KeepCollisionScfrConstraint.h>

namespace wholebodycontact_locomotion_planner {
  class Environment {
  public:
    std::shared_ptr<moveit_extensions::InterpolatedPropagationDistanceField> obstacles = std::make_shared<moveit_extensions::InterpolatedPropagationDistanceField>(5,//size_x
                                                                                                                                                                   5,//size_y
                                                                                                                                                                   5,//size_z
                                                                                                                                                                   0.04,//resolution
                                                                                                                                                                   -2.5,//origin_x
                                                                                                                                                                   -2.5,//origin_y
                                                                                                                                                                   -2.5,//origin_z
                                                                                                                                                                   0.5, // max_distance
                                                                                                                                                                   false// propagate_negative_distances
                                                                                                                                                                   );
  };

  class Mode {
  public:
    std::string name;
    double score = 1.0; // 大きい方を好む

    std::vector<std::shared_ptr<ik_constraint2_distance_field::DistanceFieldCollisionConstraint> > reachabilityConstraints;
    std::shared_ptr<ik_constraint2::IKConstraint> generateCondition(const std::shared_ptr<Environment>& environment, const cnoid::BodyPtr& robot);
  };
  class ContactPoint{
  public:
    // from config file
    cnoid::Vector3 translation = cnoid::Vector3::Zero(); // リンク座標系でどこに取り付けられているか
    cnoid::Matrix3 rotation = cnoid::Matrix3::Identity(); // リンク座標系でセンサの姿勢．zがリンク内側方向
  };
  class Contact{ // PositionConstraintに入れられるように
  public:
    std::string name; // 接触しているリンク
    cnoid::LinkPtr link1 = nullptr; // nullptrならworld
    cnoid::Isometry3 localPose1 = cnoid::Isometry3::Identity();
    cnoid::LinkPtr link2 = nullptr; // nullptrならworld
    cnoid::Isometry3 localPose2 = cnoid::Isometry3::Identity();
    cnoid::BoundingBox bbx;
    Eigen::SparseMatrix<double,Eigen::RowMajor> C{0,6}; // localPose1 frame/origin. link1がlink2から受ける力に関する接触力制約. 列は6. C, ld, udの行数は同じ.
    cnoid::VectorX dl;
    cnoid::VectorX du;
    void calcBoundingBox();
  };
}

#endif
