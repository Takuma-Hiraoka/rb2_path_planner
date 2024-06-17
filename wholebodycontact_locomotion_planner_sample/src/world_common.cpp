#include "world_common.h"

#include <cnoid/MeshGenerator>
#include <choreonoid_bullet/choreonoid_bullet.h>

namespace wholebodycontact_locomotion_planner_sample{
  void generateStepWorld(cnoid::BodyPtr& obstacle, // for visual
                         std::shared_ptr<wholebodycontact_locomotion_planner::Environment>& environment
                         ){
    cnoid::MeshGenerator meshGenerator;
    obstacle = new cnoid::Body();
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      {
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(4,2,0.1)));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(0,0,-0.05);
          posTransform->addChild(shape);
          rootLink->addShapeNode(posTransform);
        }
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(1,1,0.1)));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(1.5,0,0.35);
          posTransform->addChild(shape);
          rootLink->addShapeNode(posTransform);
        }
      }
      obstacle->setRootLink(rootLink);
    }
    environment = std::make_shared<wholebodycontact_locomotion_planner::Environment>();
    {
      // collision world
      environment->obstacles = std::make_shared<moveit_extensions::InterpolatedPropagationDistanceField>(5,//size_x
                                                                                                         5,//size_y
                                                                                                         5,//size_z
                                                                                                         0.02,//resolution // constratintのtoleranceよりも小さい必要がある.
                                                                                                         -2.5,//origin_x
                                                                                                         -2.5,//origin_y
                                                                                                         -2.5,//origin_z
                                                                                                         0.5, // max_distance
                                                                                                         true// propagate_negative_distances
                                                                                                         );
      EigenSTL::vector_Vector3d vertices;
      for(int i=0;i<obstacle->numLinks();i++){
        std::vector<Eigen::Vector3f> vertices_ = ik_constraint2_distance_field::getSurfaceVertices(obstacle->link(i), 0.01);
        for(int j=0;j<vertices_.size();j++){
          vertices.push_back(obstacle->link(i)->T() * vertices_[j].cast<double>());
        }
      }
      environment->obstacles->addPointsToField(vertices);
    }

    {
      // Supoort Polygon
      environment->surfacesBody->setRootLink(environment->surfacesBody->rootLink());
      {
        wholebodycontact_locomotion_planner::ContactableRegion region;
        region.pose.translation() = cnoid::Vector3(0.0, 0.0, 0.0);
        region.shape.resize(3,4);
        region.shape.col(0) = cnoid::Vector3(+1.9,+0.9,0.0);
        region.shape.col(1) = cnoid::Vector3(-1.9,+0.9,0.0);
        region.shape.col(2) = cnoid::Vector3(-1.9,-0.9,0.0);
        region.shape.col(3) = cnoid::Vector3(+1.9,-0.9,0.0);

        cnoid::SgShapePtr shape = new cnoid::SgShape();
        cnoid::MeshGenerator::Extrusion extrusion;
        for(int i=0;i<region.shape.cols();i++){
          extrusion.crossSection.push_back(region.shape.col(i).head<2>());
        }
        extrusion.spine.push_back(cnoid::Vector3(0,0,-0.005));
        extrusion.spine.push_back(cnoid::Vector3(0,0,0.0));
        extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
        extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
        extrusion.scale.push_back(cnoid::Vector2(1,1));
        extrusion.scale.push_back(cnoid::Vector2(1,1));
        shape->setMesh(meshGenerator.generateExtrusion(extrusion));

        region.bulletModel = choreonoid_bullet::convertToBulletModel(shape);
        environment->surfaces.push_back(region);

        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0);
        material->setDiffuseColor(cnoid::Vector3f(1.0,0.0,0.0));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->T() = region.pose;
        posTransform->addChild(shape);
        environment->surfacesBody->rootLink()->addShapeNode(posTransform);
      }
      {
        wholebodycontact_locomotion_planner::ContactableRegion region;
        region.pose.translation() = cnoid::Vector3(1.5,0.0,0.4);
        region.shape.resize(3,4);
        region.shape.col(0) = cnoid::Vector3(+0.4,+0.4,0.0);
        region.shape.col(1) = cnoid::Vector3(-0.4,+0.4,0.0);
        region.shape.col(2) = cnoid::Vector3(-0.4,-0.4,0.0);
        region.shape.col(3) = cnoid::Vector3(+0.4,-0.4,0.0);

        cnoid::SgShapePtr shape = new cnoid::SgShape();
        cnoid::MeshGenerator::Extrusion extrusion;
        for(int i=0;i<region.shape.cols();i++){
          extrusion.crossSection.push_back(region.shape.col(i).head<2>());
        }
        extrusion.spine.push_back(cnoid::Vector3(0,0,-0.005));
        extrusion.spine.push_back(cnoid::Vector3(0,0,0.0));
        extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
        extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
        extrusion.scale.push_back(cnoid::Vector2(1,1));
        extrusion.scale.push_back(cnoid::Vector2(1,1));
        shape->setMesh(meshGenerator.generateExtrusion(extrusion));

        region.bulletModel = choreonoid_bullet::convertToBulletModel(shape);
        environment->surfaces.push_back(region);

        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0);
        material->setDiffuseColor(cnoid::Vector3f(1.0,0.0,0.0));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->T() = region.pose;
        posTransform->addChild(shape);
        environment->surfacesBody->rootLink()->addShapeNode(posTransform);
      }
    }
    // bulletModelの原点の位置が違うので, 上とはべつで再度変換する必要がある
    environment->surfacesBulletModel = choreonoid_bullet::convertToBulletModels(environment->surfacesBody->rootLink()->collisionShape());
  }

  void generateTunnelWorld(cnoid::BodyPtr& obstacle, // for visual
                         std::shared_ptr<wholebodycontact_locomotion_planner::Environment>& environment
                         ){
    cnoid::MeshGenerator meshGenerator;
    obstacle = new cnoid::Body();
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      {
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(4,1,0.1)));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(1.5,0,-0.05);
          posTransform->addChild(shape);
          rootLink->addShapeNode(posTransform);
        }
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(2,1,0.1)));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(1.5,0,0.85);
          posTransform->addChild(shape);
          rootLink->addShapeNode(posTransform);
        }
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(2,0.1,0.75)));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(1.5,0.45,0.4);
          posTransform->addChild(shape);
          rootLink->addShapeNode(posTransform);
        }
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(2,0.1,0.75)));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(1.5,-0.45,0.4);
          posTransform->addChild(shape);
          rootLink->addShapeNode(posTransform);
        }
      }
      obstacle->setRootLink(rootLink);
    }
    environment = std::make_shared<wholebodycontact_locomotion_planner::Environment>();
    {
      // collision world
      environment->obstacles = std::make_shared<moveit_extensions::InterpolatedPropagationDistanceField>(7,//size_x
                                                                                                         5,//size_y
                                                                                                         5,//size_z
                                                                                                         0.02,//resolution // constratintのtoleranceよりも小さい必要がある.
                                                                                                         -0.5,//origin_x
                                                                                                         -2.5,//origin_y
                                                                                                         -2.5,//origin_z
                                                                                                         0.5, // max_distance
                                                                                                         true// propagate_negative_distances
                                                                                                         );
      EigenSTL::vector_Vector3d vertices;
      for(int i=0;i<obstacle->numLinks();i++){
        std::vector<Eigen::Vector3f> vertices_ = ik_constraint2_distance_field::getSurfaceVertices(obstacle->link(i), 0.01);
        for(int j=0;j<vertices_.size();j++){
          vertices.push_back(obstacle->link(i)->T() * vertices_[j].cast<double>());
        }
      }
      environment->obstacles->addPointsToField(vertices);
    }

    {
      // Supoort Polygon
      environment->surfacesBody->setRootLink(environment->surfacesBody->rootLink());
      {
        wholebodycontact_locomotion_planner::ContactableRegion region;
        region.pose.translation() = cnoid::Vector3(1.5, 0.0, 0.0);
        region.shape.resize(3,4);
        region.shape.col(0) = cnoid::Vector3(+1.9,+0.35,0.0);
        region.shape.col(1) = cnoid::Vector3(-1.9,+0.35,0.0);
        region.shape.col(2) = cnoid::Vector3(-1.9,-0.35,0.0);
        region.shape.col(3) = cnoid::Vector3(+1.9,-0.35,0.0);

        cnoid::SgShapePtr shape = new cnoid::SgShape();
        cnoid::MeshGenerator::Extrusion extrusion;
        for(int i=0;i<region.shape.cols();i++){
          extrusion.crossSection.push_back(region.shape.col(i).head<2>());
        }
        extrusion.spine.push_back(cnoid::Vector3(0,0,-0.005));
        extrusion.spine.push_back(cnoid::Vector3(0,0,0.0));
        extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
        extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
        extrusion.scale.push_back(cnoid::Vector2(1,1));
        extrusion.scale.push_back(cnoid::Vector2(1,1));
        shape->setMesh(meshGenerator.generateExtrusion(extrusion));

        region.bulletModel = choreonoid_bullet::convertToBulletModel(shape);
        environment->surfaces.push_back(region);

        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0);
        material->setDiffuseColor(cnoid::Vector3f(1.0,0.0,0.0));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->T() = region.pose;
        posTransform->addChild(shape);
        environment->surfacesBody->rootLink()->addShapeNode(posTransform);
      }
    }
    // bulletModelの原点の位置が違うので, 上とはべつで再度変換する必要がある
    environment->surfacesBulletModel = choreonoid_bullet::convertToBulletModels(environment->surfacesBody->rootLink()->collisionShape());
  }
}
