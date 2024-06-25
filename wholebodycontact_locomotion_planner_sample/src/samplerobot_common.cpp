#include "samplerobot_common.h"

#include <cnoid/BodyLoader>
#include <ros/package.h>
#include <iostream>
#include <choreonoid_qhull/choreonoid_qhull.h>
#include <choreonoid_bullet/choreonoid_bullet.h>
#include <choreonoid_cddlib/choreonoid_cddlib.h>
#include <cnoid/MeshExtractor>
#include <cnoid/YAMLReader>
#include <ik_constraint2_vclip/ik_constraint2_vclip.h>

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

  void generateSampleRobot(const std::shared_ptr<moveit_extensions::InterpolatedPropagationDistanceField>& field,
                           std::shared_ptr<wholebodycontact_locomotion_planner::WBLPParam>& param,
                           cnoid::BodyPtr& abstractRobot // for visual
                           ) {
    cnoid::BodyLoader bodyLoader;
    param->robot = bodyLoader.load(ros::package::getPath("choreonoid") + "/share/model/SR1/SR1.body");

    param->robot->rootLink()->p() = cnoid::Vector3(0,0,0.65);
    param->robot->rootLink()->v().setZero();
    param->robot->rootLink()->R() = cnoid::Matrix3::Identity();
    param->robot->rootLink()->w().setZero();

    std::vector<double> reset_manip_pose{
      0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0,// rleg
        0.523599, 0.0, 0.0, -1.74533, 0.15708, -0.113446, 0.637045,// rarm
        0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0,// lleg
        0.523599, 0.0, 0.0, -1.74533, -0.15708, -0.113446, -0.637045,// larm
        0.1, 0.0, 0.0}; // torso. waist-pを少し前に傾けておくと、後ろにひっくり返りにくくなる
    for(int j=0; j < param->robot->numJoints(); ++j){
      param->robot->joint(j)->q() = reset_manip_pose[j];
    }

    param->robot->calcForwardKinematics();
    param->robot->calcCenterOfMass();

    {
      // variables
      param->variables.push_back(param->robot->rootLink());
      for(int i=0;i<param->robot->numJoints();i++){
        param->variables.push_back(param->robot->joint(i));
      }
    }

    {
      // task: nominal constairnt
      for(int i=0;i<param->robot->numJoints();i++){
        std::shared_ptr<ik_constraint2::JointAngleConstraint> constraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
        constraint->joint() = param->robot->joint(i);
        constraint->targetq() = reset_manip_pose[i];
        constraint->precision() = 1e10; // always satisfied
        param->nominals.push_back(constraint);
      }
    }

    abstractRobot = param->robot->clone();
    {
      cnoid::Affine3 transform = cnoid::Affine3::Identity();
      transform.linear() *= 1.2;
      for (int i=0; i<abstractRobot->numLinks(); i++) {
        if(param->robot->link(i)->name() == "LLEG_ANKLE_P" ||
           param->robot->link(i)->name() == "RLEG_ANKLE_P" ||
           param->robot->link(i)->name() == "LLEG_HIP_P" ||
           param->robot->link(i)->name() == "RLEG_HIP_P" ||
           param->robot->link(i)->name() == "LLEG_HIP_R" ||
           param->robot->link(i)->name() == "RLEG_HIP_R" ||
           param->robot->link(i)->name() == "LARM_WRIST_P" ||
           param->robot->link(i)->name() == "RARM_WRIST_P" ||
           param->robot->link(i)->name() == "LARM_WRIST_Y" ||
           param->robot->link(i)->name() == "RARM_WRIST_Y" ||
           param->robot->link(i)->name() == "LARM_SHOULDER_Y" ||
           param->robot->link(i)->name() == "RARM_SHOULDER_Y" ||
           param->robot->link(i)->name() == "LARM_SHOULDER_P" ||
           param->robot->link(i)->name() == "RARM_SHOULDER_P" ||
           param->robot->link(i)->name() == "WAIST_P") continue;
        // 拡大凸包meshを作る
        cnoid::SgNodePtr collisionshape = param->robot->link(i)->collisionShape();
        Eigen::Matrix<double,3,Eigen::Dynamic> vertices;
        if (collisionshape) {
          cnoid::SgMeshPtr model = convertToSgMesh(collisionshape); // まずmeshに変換
          // 拡大
          {
            for (int v=0; v<model->vertices()->size(); v++) {
              model->vertices()->at(v) += model->vertices()->at(v).cast<cnoid::Vector3f::Scalar>() / (model->vertices()->at(v).cast<cnoid::Vector3f::Scalar>()).norm() * 0.1;
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
          abstractRobot->link(i)->clearShapeNodes();
          abstractRobot->link(i)->addVisualShapeNode(shape);
          abstractRobot->link(i)->addCollisionShapeNode(shape);
        }else{
          std::cerr << __PRETTY_FUNCTION__ << " convex hull " << abstractRobot->link(i)->name() << " fail" << std::endl;
        }

      }
    } // abstractRobot

    // currentContactPoint
    param->currentContactPoints.clear();
    {
      {
        wholebodycontact_locomotion_planner::Contact lleg;
        lleg.name = "LLEG_ANKLE_R";
        lleg.link1 = param->robot->link("LLEG_ANKLE_R");
        lleg.localPose1.translation() = cnoid::Vector3(0.0,0.0,-0.045);
        lleg.localPose2.translation() = cnoid::Vector3(0.0,0.09,0.0);
        param->currentContactPoints.push_back(lleg);
      }
      {
        wholebodycontact_locomotion_planner::Contact rleg;
        rleg.name = "RLEG_ANKLE_R";
        rleg.link1 = param->robot->link("RLEG_ANKLE_R");
        rleg.localPose1.translation() = cnoid::Vector3(0.0,0.0,0.045);
        rleg.localPose2.translation() = cnoid::Vector3(0.0,-0.09,0.045);
        param->currentContactPoints.push_back(rleg);
      }
    }

    param->constraints.clear();
    // environmental collision
    for (int i=0; i<param->robot->numLinks(); i++) {
      if(param->robot->link(i)->name() == "LLEG_ANKLE_P" ||
         param->robot->link(i)->name() == "RLEG_ANKLE_P" ||
         param->robot->link(i)->name() == "LLEG_HIP_P" ||
         param->robot->link(i)->name() == "RLEG_HIP_P" ||
         param->robot->link(i)->name() == "LLEG_HIP_R" ||
         param->robot->link(i)->name() == "RLEG_HIP_R" ||
         param->robot->link(i)->name() == "LARM_WRIST_P" ||
         param->robot->link(i)->name() == "RARM_WRIST_P" ||
         param->robot->link(i)->name() == "LARM_WRIST_Y" ||
         param->robot->link(i)->name() == "RARM_WRIST_Y" ||
         param->robot->link(i)->name() == "LARM_SHOULDER_Y" ||
         param->robot->link(i)->name() == "RARM_SHOULDER_Y" ||
         param->robot->link(i)->name() == "LARM_SHOULDER_P" ||
         param->robot->link(i)->name() == "RARM_SHOULDER_P" ||
         param->robot->link(i)->name() == "WAIST_P") continue;
      std::shared_ptr<ik_constraint2_distance_field::DistanceFieldCollisionConstraint> constraint = std::make_shared<ik_constraint2_distance_field::DistanceFieldCollisionConstraint>();
      constraint->A_link() = param->robot->link(i);
      constraint->field() = field;
      constraint->tolerance() = 0.06; // ちょうど干渉すると法線ベクトルが変になることがあるので, 1回のiterationで動きうる距離よりも大きくせよ.
      constraint->precision() = 0.05; // 角で不正確になりがちなので, toleranceを大きくしてprecisionも大きくして、best effort的にする. precisionはdistanceFieldのサイズの倍数より大きくする. 大きく動くのでつま先が近かったときにつま先は近くならないがかかとが地面にめり込む、ということは起こりうる.
      constraint->ignoreDistance() = 0.5; // 大きく動くので、ignoreも大きくする必要がある
      //      constraint->maxError() = 0.1; // めり込んだら一刻も早く離れたい
      constraint->updateBounds(); // キャッシュを内部に作る. キャッシュを作ったあと、10スレッドぶんコピーする方が速い
      param->constraints.push_back(constraint);
    }
    // task: self collision
    {
      std::vector<std::string> rarm{"RARM_SHOULDER_R", "RARM_ELBOW", "RARM_WRIST_R"};
      std::vector<std::string> larm{"LARM_SHOULDER_R", "LARM_ELBOW", "LARM_WRIST_R"};
      std::vector<std::string> rleg{"RLEG_HIP_Y", "RLEG_KNEE"};
      std::vector<std::string> lleg{"LLEG_HIP_Y", "LLEG_KNEE"};
      std::vector<std::string> torso{"WAIST", "WAIST_R", "CHEST"};

      std::vector<std::vector<std::string> > pairs;
      for(int i=0;i<rarm.size();i++) for(int j=0;j<larm.size();j++) pairs.push_back(std::vector<std::string>{rarm[i],larm[j]});
      for(int i=0;i<rarm.size();i++) for(int j=0;j<rleg.size();j++) pairs.push_back(std::vector<std::string>{rarm[i],rleg[j]});
      for(int i=0;i<rarm.size();i++) for(int j=0;j<lleg.size();j++) pairs.push_back(std::vector<std::string>{rarm[i],lleg[j]});
      for(int i=0;i<rarm.size();i++) for(int j=0;j<torso.size();j++) pairs.push_back(std::vector<std::string>{rarm[i],torso[j]});
      for(int i=0;i<larm.size();i++) for(int j=0;j<rleg.size();j++) pairs.push_back(std::vector<std::string>{larm[i],rleg[j]});
      for(int i=0;i<larm.size();i++) for(int j=0;j<lleg.size();j++) pairs.push_back(std::vector<std::string>{larm[i],lleg[j]});
      for(int i=0;i<larm.size();i++) for(int j=0;j<torso.size();j++) pairs.push_back(std::vector<std::string>{larm[i],torso[j]});
      for(int i=0;i<rleg.size();i++) for(int j=0;j<lleg.size();j++) pairs.push_back(std::vector<std::string>{rleg[i],lleg[j]});

      for(int i=0;i<pairs.size();i++){
        std::shared_ptr<ik_constraint2_vclip::VclipCollisionConstraint> constraint = std::make_shared<ik_constraint2_vclip::VclipCollisionConstraint>();
        constraint->A_link() = param->robot->link(pairs[i][0]);
        constraint->B_link() = param->robot->link(pairs[i][1]);
        constraint->tolerance() = 0.01;
        param->constraints.push_back(constraint);
      }
    }

    param->modes.clear();
    {
      std::shared_ptr<wholebodycontact_locomotion_planner::Mode> mode = std::make_shared<wholebodycontact_locomotion_planner::Mode>();
      param->modes["wholebody"] = mode;
      mode->score = 2.0;
      {
        // reachability
        for (int i=0; i<param->robot->numLinks(); i++) {
          if(param->robot->link(i)->name() == "LLEG_ANKLE_P" ||
             param->robot->link(i)->name() == "RLEG_ANKLE_P" ||
             param->robot->link(i)->name() == "LLEG_HIP_P" ||
             param->robot->link(i)->name() == "RLEG_HIP_P" ||
             param->robot->link(i)->name() == "LLEG_HIP_R" ||
             param->robot->link(i)->name() == "RLEG_HIP_R" ||
             param->robot->link(i)->name() == "LARM_WRIST_P" ||
             param->robot->link(i)->name() == "RARM_WRIST_P" ||
             param->robot->link(i)->name() == "LARM_WRIST_Y" ||
             param->robot->link(i)->name() == "RARM_WRIST_Y" ||
             param->robot->link(i)->name() == "LARM_SHOULDER_Y" ||
             param->robot->link(i)->name() == "RARM_SHOULDER_Y" ||
             param->robot->link(i)->name() == "LARM_SHOULDER_P" ||
             param->robot->link(i)->name() == "RARM_SHOULDER_P" ||
             param->robot->link(i)->name() == "WAIST_P") continue;
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = param->robot->link(i);
          constraint->A_link_bulletModel() = constraint->A_link();
          constraint->A_bulletModel() = choreonoid_bullet::convertToBulletModels(abstractRobot->link(i)->collisionShape());
          constraint->B_link() = constraint->A_link(); // dummy. solve時にenvironmentから設定し直す
          constraint->precision() = 0.01;
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraints.push_back(constraint);
        }
      }
    } // mode1

    param->contactPoints.clear();
    std::string contactFileName = ros::package::getPath("wholebodycontact_locomotion_planner_sample") + "/config/sample_config.yaml";
    cnoid::YAMLReader reader;
    cnoid::MappingPtr node;
    std::string prevLinkName = "";
    std::vector<wholebodycontact_locomotion_planner::ContactPoint> contactPoints;
    try {
      node = reader.loadDocument(contactFileName)->toMapping();
    } catch(const cnoid::ValueNode::Exception& ex) {
      std::cerr << ex.message()  << std::endl;
    }
    if(node){
      cnoid::Listing* tactileSensorList = node->findListing("tactile_sensor");
      if (!tactileSensorList->isValid()) {
        std::cerr << "tactile_sensor list is not valid" << std::endl;
      }else{
        for (int i=0; i< tactileSensorList->size(); i++) {
          cnoid::Mapping* info = tactileSensorList->at(i)->toMapping();
          std::string linkName;
          // linkname
          info->extract("link", linkName);
          if (linkName != prevLinkName) {
            if (prevLinkName != "") {
              param->contactPoints[prevLinkName] = contactPoints;
            }
            prevLinkName = linkName;
            contactPoints.clear();
          }
          wholebodycontact_locomotion_planner::ContactPoint sensor;
          // translation
          cnoid::ValueNodePtr translation_ = info->extract("translation");
          if(translation_){
            cnoid::ListingPtr translationTmp = translation_->toListing();
            if(translationTmp->size()==3){
              sensor.translation = cnoid::Vector3(translationTmp->at(0)->toDouble(), translationTmp->at(1)->toDouble(), translationTmp->at(2)->toDouble());
            }
          }
          // rotation
          cnoid::ValueNodePtr rotation_ = info->extract("rotation");
          if(rotation_){
            cnoid::ListingPtr rotationTmp = rotation_->toListing();
            if(rotationTmp->size() == 4){
              sensor.rotation = cnoid::AngleAxisd(rotationTmp->at(3)->toDouble(),
                                                  cnoid::Vector3{rotationTmp->at(0)->toDouble(), rotationTmp->at(1)->toDouble(), rotationTmp->at(2)->toDouble()}).toRotationMatrix();
            }
          }
          contactPoints.push_back(sensor);
        }
        if (prevLinkName != "") {
          param->contactPoints[prevLinkName] = contactPoints;
        }
      }
    }
  }
}
