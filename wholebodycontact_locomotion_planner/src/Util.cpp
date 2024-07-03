#include <wholebodycontact_locomotion_planner/Util.h>
#include <cnoid/MeshExtractor>
#include <choreonoid_qhull/choreonoid_qhull.h>

namespace wholebodycontact_locomotion_planner{
  bool solveContactIK(const std::shared_ptr<WBLPParam>& param,
                      const std::vector<std::shared_ptr<Contact> >& stopContacts,
                      const std::vector<std::shared_ptr<Contact> >& nextContacts,
                      const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominals,
                      bool attach,
                      bool slide) {
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints0;
    double defaultTolerance = 0.06;
    double defaultPrecision = 0.05;
    for (int i=0; i<param->constraints.size(); i++) {
      if (typeid(*(param->constraints[i]))==typeid(ik_constraint2_distance_field::DistanceFieldCollisionConstraint)) {
        bool skip=false;
        for (int j=0; j<stopContacts.size() && !skip;j++) {
          if (stopContacts[j]->name == std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param->constraints[i])->A_link()->name()) skip = true;
        }
        for (int j=0; j<nextContacts.size() && !skip;j++) {
          if (nextContacts[j]->name == std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param->constraints[i])->A_link()->name()) {
            if (attach) { // 実際に触れされるときだけ、触れるリンクの干渉は無視する. slideならはじめに着いたとき、detach-attachならdetachのときに干渉を考慮した姿勢が出ているので、そこから先は干渉しないと仮定.
              skip = true;
            } else {
              defaultTolerance = std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param->constraints[i])->tolerance();
              defaultPrecision = std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param->constraints[i])->precision();
              std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param->constraints[i])->tolerance() = 0.02;
              std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param->constraints[i])->precision() = 0.01;
            }
          }
        }
        if (skip) continue;
      }
      constraints0.push_back(param->constraints[i]);
    }
    std::shared_ptr<ik_constraint2_scfr::ScfrConstraint> scfrConstraint = std::make_shared<ik_constraint2_scfr::ScfrConstraint>();
    scfrConstraint->A_robot() = param->robot;
    std::vector<cnoid::Isometry3> poses;
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > As;
    std::vector<cnoid::VectorX> bs;
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > Cs;
    std::vector<cnoid::VectorX> dls;
    std::vector<cnoid::VectorX> dus;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints1;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > goals;
    {
      for (int i=0; i<stopContacts.size(); i++) {
        bool move=false;
        for (int j=0; j<nextContacts.size(); j++) {
          if (stopContacts[i]->name == nextContacts[j]->name) move=true;;
        }
        if (move) continue; // このcontactを動かす予定.
        std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
        constraint->A_link() = stopContacts[i]->link1;
        constraint->A_localpos() = stopContacts[i]->localPose1;
        constraint->B_link() = stopContacts[i]->link2;
        constraint->B_localpos() = stopContacts[i]->localPose2;
        constraint->eval_link() = nullptr;
        constraint->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        constraints1.push_back(constraint);
        poses.push_back(stopContacts[i]->localPose2);
        As.emplace_back(0,6);
        bs.emplace_back(0);
        Cs.push_back(stopContacts[i]->C);
        dls.push_back(stopContacts[i]->dl);
        dus.push_back(stopContacts[i]->du);
      }
      for (int i=0; i<nextContacts.size(); i++) {
        std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
        constraint->A_link() = nextContacts[i]->link1;
        constraint->A_localpos() = nextContacts[i]->localPose1;
        if (!attach || slide) { // 接触ローカル位置は変えない
          for (int j=0; j<stopContacts.size(); j++) {
            if (nextContacts[i]->name == stopContacts[j]->name) {
              constraint->A_localpos() = stopContacts[j]->localPose1;
            }
          }
        }
        constraint->B_link() = nextContacts[i]->link2;
        constraint->B_localpos() = nextContacts[i]->localPose2;
        if (!attach) constraint->B_localpos().translation() += nextContacts[i]->localPose2.rotation() * cnoid::Vector3(0,0,0.05); // 0.05だけ離す
        constraint->eval_link() = nullptr;
        constraint->eval_localR() = nextContacts[i]->localPose2.rotation();
        constraint->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 0.01;
        if (!attach && param->useSwingGIK) {goals.push_back(constraint); // 浮いている時はGIKをつかう
        }else{constraints1.push_back(constraint);}
        if (slide) {
          for (int j=0; j<stopContacts.size(); j++) {
            if (nextContacts[i]->name == stopContacts[j]->name) {
              constraint->weight()[5] = 1.0; // 摩擦制約の関係上一致させる必要がある
              poses.push_back(nextContacts[i]->localPose2);
              cnoid::Vector3 diff = (stopContacts[j]->link1->T() * stopContacts[j]->localPose1).rotation().transpose() * (nextContacts[i]->localPose2.translation() - stopContacts[j]->localPose2.translation());
              cnoid::Matrix3 diffR = ((stopContacts[j]->link1->T() * stopContacts[j]->localPose1).rotation().transpose() * (nextContacts[i]->link1->T() * nextContacts[i]->localPose1).rotation());
              Eigen::SparseMatrix<double,Eigen::RowMajor> A(3,6);
              A.insert(0,0) = diff[0] > 0 ? -1.0 : 1.0; A.insert(0,2) = 0.2;
              A.insert(1,1) = diff[1] > 0 ? -1.0 : 1.0; A.insert(1,2) = 0.2;
              A.insert(2,5) = cnoid::rpyFromRot(diffR)[2] > 0 ? -1.0 : 1.0; A.insert(2,2) = 0.005;
              As.push_back(A);
              cnoid::VectorX b = Eigen::VectorXd::Zero(3);
              bs.push_back(b);
              Eigen::SparseMatrix<double,Eigen::RowMajor> C(5,6); // TODO 干渉形状から出す？
              C.insert(0,2) = 1.0;
              C.insert(1,2) = 0.05; C.insert(1,3) = 1.0;
              C.insert(2,2) = 0.05; C.insert(2,3) = -1.0;
              C.insert(3,2) = 0.05; C.insert(3,4) = 1.0;
              C.insert(4,2) = 0.05; C.insert(4,4) = -1.0;
              Cs.push_back(C);
              cnoid::VectorX dl = Eigen::VectorXd::Zero(5);
              dls.push_back(dl);
              cnoid::VectorX du = 1e10 * Eigen::VectorXd::Ones(5);
              du[0] = 2000.0;
              dus.push_back(du);
            }
          }
        }
      }
    }
    scfrConstraint->poses() = poses;
    scfrConstraint->As() = As;
    scfrConstraint->bs() = bs;
    scfrConstraint->Cs() = Cs;
    scfrConstraint->dls() = dls;
    scfrConstraint->dus() = dus;
    constraints1.push_back(scfrConstraint);

    bool solved;
    if (attach || !param->useSwingGIK) {
      std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints{constraints0, constraints1, nominals};
      prioritized_inverse_kinematics_solver2::IKParam pikParam;
      pikParam.checkFinalState=true;
      pikParam.calcVelocity = false;
      pikParam.debugLevel = 0;
      pikParam.we = 1e2;
      pikParam.we = 1e1;
      pikParam.maxIteration = 100;
      std::vector<std::shared_ptr<prioritized_qp_base::Task> > prevTasks;
      solved  =  prioritized_inverse_kinematics_solver2::solveIKLoop(param->variables,
                                                                     constraints,
                                                                     prevTasks,
                                                                     pikParam
                                                                     );
    } else {
      std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints{constraints0, constraints1};
      param->gikParam.projectLink.resize(1);
      param->gikParam.projectLink[0] = nextContacts[0]->link1;
      param->gikParam.projectLocalPose = nextContacts[0]->localPose1;
      std::shared_ptr<std::vector<std::vector<double> > > path;
      // 関節角度上下限を厳密に満たしていないと、omplのstart stateがエラーになるので
      for(int i=0;i<param->variables.size();i++){
        if(param->variables[i]->isRevoluteJoint() || param->variables[i]->isPrismaticJoint()) {
          param->variables[i]->q() = std::max(std::min(param->variables[i]->q(),param->variables[i]->q_upper()),param->variables[i]->q_lower());
        }
      }
      solved = global_inverse_kinematics_solver::solveGIK(param->variables,
                                                          constraints,
                                                          goals,
                                                          nominals,
                                                          param->gikParam,
                                                          path);
    }
    // for ( int i=0; i<constraints0.size(); i++ ) {
    //   std::cerr << "constraints0: "<< constraints0[i]->isSatisfied() << std::endl;
    // }
    // for ( int i=0; i<constraints1.size(); i++ ) {
    //   constraints1[i]->debugLevel() = 2;
    //   constraints1[i]->updateBounds();
    //   std::cerr << "constraints1: "<< constraints1[i]->isSatisfied() << std::endl;
    // }
    // for ( int i=0; i<nominals.size(); i++ ) {
    //   std::cerr << "nominals: "<< nominals[i]->isSatisfied() << std::endl;
    // }
    for (int i=0; i<param->constraints.size(); i++) {
      if (typeid(*(param->constraints[i]))==typeid(ik_constraint2_distance_field::DistanceFieldCollisionConstraint)) {
        for (int j=0; j<nextContacts.size();j++) {
          if (nextContacts[j]->name == std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param->constraints[i])->A_link()->name()) {
            std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param->constraints[i])->tolerance() = defaultTolerance;
            std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param->constraints[i])->precision() = defaultPrecision;
          }
        }
      }
    }

    return solved;
  }

  bool calcContactPoint(const std::shared_ptr<WBLPParam>& param,
                        const std::vector<std::shared_ptr<Contact> >& attachContacts
                        ) { // 本来はIKの中で探索したい
    for (int i=0;i<attachContacts.size();i++) {
      double maxValue = -1000;
      for(int j=0;j<param->contactPoints[attachContacts[i]->name].size();j++){
        double weight = 0.1;
        double value=weight*(attachContacts[i]->localPose2.linear()*cnoid::Vector3::UnitZ()).dot(attachContacts[i]->link1->R() * param->contactPoints[attachContacts[i]->name][j].rotation * cnoid::Vector3::UnitZ());
        value -=(attachContacts[i]->link1->p() + attachContacts[i]->link1->R() * param->contactPoints[attachContacts[i]->name][j].translation - attachContacts[i]->localPose2.translation()).norm();
        if (value > maxValue) {
          maxValue = value;
          attachContacts[i]->localPose1.linear() = param->contactPoints[attachContacts[i]->name][j].rotation;
          attachContacts[i]->localPose1.translation() = param->contactPoints[attachContacts[i]->name][j].translation;
        }
      }
    }
  }
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
      std::cerr << "[convertToSgMesh] meshExtractor->extract failed " << collisionshape->name() << std::endl;
      return nullptr;
    }
    model->setName(collisionshape->name());

    return model;
  }

  void createAbstractRobot(const std::shared_ptr<WBLPParam>& param,
                           const std::vector<std::string> contactableLinkNames,
                           cnoid::BodyPtr& abstractRobot
                           ) {
    abstractRobot = param->robot->clone();
    for (int i=0; i<abstractRobot->numLinks(); i++) {
      double expansionLength = param->expansionLength;
      if(std::find(contactableLinkNames.begin(),contactableLinkNames.end(),param->robot->link(i)->name()) == contactableLinkNames.end()) expansionLength = 0.0;
      // 拡大凸包meshを作る
      cnoid::SgNodePtr collisionshape = param->robot->link(i)->collisionShape();
      Eigen::Matrix<double,3,Eigen::Dynamic> vertices;
      if (collisionshape) {
        cnoid::SgMeshPtr model = convertToSgMesh(collisionshape); // まずmeshに変換
        // 拡大
        if (model) {
          for (int v=0; v<model->vertices()->size(); v++) {
            model->vertices()->at(v) += model->vertices()->at(v).cast<cnoid::Vector3f::Scalar>() / (model->vertices()->at(v).cast<cnoid::Vector3f::Scalar>()).norm() * expansionLength;
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
      if(shape){
        shape->setMaterial(material);
        shape->setName(collisionshape->name());
        abstractRobot->link(i)->clearShapeNodes();
        abstractRobot->link(i)->addVisualShapeNode(shape);
        abstractRobot->link(i)->addCollisionShapeNode(shape);
      }else{
        std::cerr << __PRETTY_FUNCTION__ << " convex hull " << abstractRobot->link(i)->name() << " fail" << std::endl;
      }
    }
  }
}
