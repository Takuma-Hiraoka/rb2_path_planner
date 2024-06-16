#include "samplerobot_common.h"

#include <cnoid/BodyLoader>
#include <ros/package.h>
#include <iostream>
#include <choreonoid_qhull/choreonoid_qhull.h>
#include <cnoid/MeshExtractor>

namespace wholebodycontact_locomotion_planner_sample{
  inline void addMesh(cnoid::SgMeshPtr model, std::shared_ptr<cnoid::MeshExtractor> meshExtractor){
    cnoid::SgMeshPtr mesh = meshExtractor->currentMesh();
    const cnoid::Affine3& T = meshExtractor->currentTransform();

    const int vertexIndexTop = model->vertices()->size();

    const cnoid::SgVertexArray& vertices = *mesh->vertices();
    const int numVertices = vertices.size();
    for(int i=0; i < numVertices; ++i){
      const cnoid::Vector3 v = T * vertices[i].cast<cnoid::Affine3::Scalar>();
      model->vertices()->push_back(v.cast<cnoid::Vector3f::Scalar>());
    }

    const int numTriangles = mesh->numTriangles();
    for(int i=0; i < numTriangles; ++i){
      cnoid::SgMesh::TriangleRef tri = mesh->triangle(i);
      const int v0 = vertexIndexTop + tri[0];
      const int v1 = vertexIndexTop + tri[1];
      const int v2 = vertexIndexTop + tri[2];
      model->addTriangle(v0, v1, v2);
    }
  }

  inline cnoid::SgMeshPtr convertToSgMesh (const cnoid::SgNodePtr collisionshape){

    if (!collisionshape) return nullptr;

    std::shared_ptr<cnoid::MeshExtractor> meshExtractor = std::make_shared<cnoid::MeshExtractor>();
    cnoid::SgMeshPtr model = new cnoid::SgMesh; model->getOrCreateVertices();
    if(meshExtractor->extract(collisionshape, [&]() { addMesh(model,meshExtractor); })){
    }else{
      // mesh not found
    }
    model->setName(collisionshape->name());

    return model;
  }

  void generateSampleRobot(cnoid::BodyPtr& robot,
                           cnoid::BodyPtr& abstractRobot
                           ) {
    cnoid::BodyLoader bodyLoader;
    robot = bodyLoader.load(ros::package::getPath("choreonoid") + "/share/model/SR1/SR1.body");

    robot->rootLink()->p() = cnoid::Vector3(0,0,0.65);
    robot->rootLink()->v().setZero();
    robot->rootLink()->R() = cnoid::Matrix3::Identity();
    robot->rootLink()->w().setZero();

    std::vector<double> reset_manip_pose{
      0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0,// rleg
        0.523599, 0.0, 0.0, -1.74533, 0.15708, -0.113446, 0.637045,// rarm
        0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0,// lleg
        0.523599, 0.0, 0.0, -1.74533, -0.15708, -0.113446, -0.637045,// larm
        0.1, 0.0, 0.0}; // torso. waist-pを少し前に傾けておくと、後ろにひっくり返りにくくなる
    for(int j=0; j < robot->numJoints(); ++j){
      robot->joint(j)->q() = reset_manip_pose[j];
    }

    robot->calcForwardKinematics();
    robot->calcCenterOfMass();

    abstractRobot = robot->clone();
    {
      cnoid::Affine3 transform = cnoid::Affine3::Identity();
      transform.linear() *= 1.2;
      for (int i=0; i<abstractRobot->numLinks(); i++) {
        // 拡大凸包meshを作る
        cnoid::SgNodePtr collisionshape = robot->link(i)->collisionShape();
        Eigen::Matrix<double,3,Eigen::Dynamic> vertices;
        if (collisionshape) {
          cnoid::SgMeshPtr model = convertToSgMesh(collisionshape); // まずmeshに変換
          // 拡大
          {
            for (int v=0; v<model->vertices()->size(); v++) {
              model->vertices()->at(v) += model->vertices()->at(v).cast<cnoid::Vector3f::Scalar>() / (model->vertices()->at(v).cast<cnoid::Vector3f::Scalar>()).norm() * 0.03;
            }
          }
          // 凸包
          if (model && model->vertices()->size() > 0) {
            Eigen::Matrix<double,3,Eigen::Dynamic> vs(3,model->vertices()->size());
            for(size_t i=0;i<model->vertices()->size();i++){
              vs.col(i) = model->vertices()->at(i).cast<Eigen::Vector3d::Scalar>();
            }
            vertices = vs;
          } else {
            vertices = Eigen::MatrixXd(3,0);
          }
        } else {
          vertices = Eigen::MatrixXd(3,0);
        }
        cnoid::SgShapePtr shape = choreonoid_qhull::generateMeshFromConvexHull(vertices);
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.6);
        shape->setMaterial(material);
        if(shape) shape->setName(collisionshape->name());
        if(shape){
          robot->link(i)->clearShapeNodes();
          robot->link(i)->addVisualShapeNode(shape);
          robot->link(i)->addCollisionShapeNode(shape);
        }else{
          std::cerr << __PRETTY_FUNCTION__ << " convex hull " << robot->link(i)->name() << " fail" << std::endl;
        }

      }
    }
  }
}
