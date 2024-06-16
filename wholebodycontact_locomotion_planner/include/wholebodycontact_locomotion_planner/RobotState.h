#ifndef WHOLEBODYCONTACT_LOCOMOTION_PLANNER_ROBOTSTATE_H
#define WHOLEBODYCONTACT_LOCOMOTION_PLANNER_ROBOTSTATE_H

#include <cnoid/Body>
#include <ik_constraint2_distance_field/ik_constraint2_distance_field.h>
#include <ik_constraint2_bullet/ik_constraint2_bullet.h>

namespace wholebodycontact_locomotion_planner {
  class ContactableRegion {
  public:
    cnoid::Isometry3 pose = cnoid::Isometry3::Identity();
    Eigen::Matrix<double, 3, Eigen::Dynamic> shape; // pose frame. 3xX [v1, v2, v3 ...] の凸形状
    std::shared_ptr<btConvexShape> bulletModel;
    /*
      2D surface polygonの場合
          shapeのZ座標は0. Z座標+の方向が法線方向
     */
  };

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
    std::vector<ContactableRegion> surfaces;
    cnoid::BodyPtr surfacesBody = new cnoid::Body();
    std::vector<std::shared_ptr<btConvexShape> > surfacesBulletModel; // rootLinkに対応
  };
}

#endif
