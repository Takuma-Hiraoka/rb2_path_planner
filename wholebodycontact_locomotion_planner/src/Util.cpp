#include <wholebodycontact_locomotion_planner/Util.h>
#include <cnoid/MeshExtractor>
#include <choreonoid_qhull/choreonoid_qhull.h>
#include <ik_constraint2_scfr/SlideScfrConstraint.h>

namespace wholebodycontact_locomotion_planner{
  bool solveContactIK(const std::shared_ptr<WBLPParam>& param,
                      const std::vector<std::shared_ptr<Contact> >& stopContacts,
                      const std::vector<std::shared_ptr<Contact> >& nextContacts,
                      const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominals,
                      const IKState ikState,
                      const std::shared_ptr<std::vector<std::vector<double> > > path
                      ) {
    std::shared_ptr<std::vector<std::vector<double> > > tmpPath = std::make_shared<std::vector<std::vector<double> > >();
    std::vector<cnoid::LinkPtr> variables;
    for (int i=0;i<param->variables.size(); i++) {
      variables.push_back(param->variables[i]);
    }
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints0;
    for (int i=0; i<param->constraints.size(); i++) {
      if (typeid(*(param->constraints[i]))==typeid(ik_constraint2_distance_field::DistanceFieldCollisionConstraint)) {
        bool skip=false;
        for (int j=0; j<stopContacts.size() && !skip;j++) {
          bool move=false;
          for (int k=0; k<nextContacts.size(); k++) {
            if (stopContacts[j]->name == nextContacts[k]->name) move=true;;
          }
          if (move) continue; // 動かす予定ならnextContactsの判定を使う
          if (stopContacts[j]->name == std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param->constraints[i])->A_link()->name()) skip = true;
        }
        for (int j=0; j<nextContacts.size() && !skip;j++) {
          if (nextContacts[j]->name == std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param->constraints[i])->A_link()->name()) {
            if ((ikState == IKState::ATTACH) ||
                (ikState == IKState::ATTACH_FIXED) ||
                (ikState == IKState::ATTACH_SEARCH) ||
                (ikState == IKState::SLIDE) ||
                (ikState == IKState::DETACH) ||
                (ikState == IKState::DETACH_FIXED)) { // 実際に触れされるときだけ、触れるリンクの干渉は無視する. slideならはじめに着いたとき、detach-attachならdetachのときに干渉を考慮した姿勢が出ているので、そこから先は干渉しないと仮定.
              // DETACH時もすでに触れているときの挙動を回避するため
              skip = true;
            } else {
              std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param->constraints[i])->tolerance() = 0.02;
              std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param->constraints[i])->precision() = 0.015;
            }
          }
        }
        if (skip) continue;
      }
      constraints0.push_back(param->constraints[i]);
    }
    std::shared_ptr<ik_constraint2_slide_scfr::SlideScfrConstraint> scfrConstraint = std::make_shared<ik_constraint2_slide_scfr::SlideScfrConstraint>();
    scfrConstraint->A_robot() = param->robot;
    std::vector<cnoid::Isometry3> poses;
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > As;
    std::vector<cnoid::VectorX> bs;
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > Cs;
    std::vector<cnoid::VectorX> dls;
    std::vector<cnoid::VectorX> dus;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints1;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints2;
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
        if (param->useSlide) constraint->weight()[5] = 0.0;
        constraints1.push_back(constraint);
        poses.push_back(stopContacts[i]->localPose2);
        scfrConstraint->positionConstraints().push_back(constraint);
        As.emplace_back(0,6);
        bs.emplace_back(0);
        Cs.push_back(stopContacts[i]->C);
        dls.push_back(stopContacts[i]->dl);
        dus.push_back(stopContacts[i]->du);
        calcIgnoreBoundingBox(param->constraints, stopContacts[i], 3);
      }
      for (int i=0; i<nextContacts.size(); i++) {
        if ((ikState == IKState::ATTACH_SEARCH) || (ikState == IKState::SWING) || (ikState == IKState::CONTACT_SEARCH)) { // 現在接触している状態から外すときに接触点も探索に含めてしまうと、リンクの裏側に接触点が移動してしまう
          for (int j=0; j<param->bodyContactConstraints.size(); j++) {
            if (nextContacts[i]->name == param->bodyContactConstraints[j]->A_link()->name()) {
              std::shared_ptr<ik_constraint2_body_contact::BodyContactConstraint> constraint = param->bodyContactConstraints[j];
              constraint->A_link() = nextContacts[i]->link1;
              constraint->A_localpos() = nextContacts[i]->localPose1;
              for (int stop=0; stop<stopContacts.size(); stop++) {
                if (stopContacts[stop]->name == nextContacts[i]->name) constraint->A_localpos() = stopContacts[stop]->localPose1;
              }
              constraint->B_link() = nextContacts[i]->link2;
              constraint->B_localpos() = nextContacts[i]->localPose2;
              if ((ikState == IKState::SWING) || (ikState == IKState::CONTACT_SEARCH)) constraint->B_localpos().translation() += nextContacts[i]->localPose2.rotation() * cnoid::Vector3(0,0,0.02); // 0.03だけ離す
              constraint->eval_localR() = constraint->B_localpos().linear();
              if (constraint->contact_pos_link()){
                variables.push_back(constraint->contact_pos_link());
                constraint->contact_pos_link()->T() = constraint->A_localpos();
                std::vector<double> dqWeight = std::vector<double>(6,1);
                std::copy(dqWeight.begin(), dqWeight.end(), std::back_inserter(param->pikParam.dqWeight));
                std::copy(dqWeight.begin(), dqWeight.end(), std::back_inserter(param->gikParam.pikParam.dqWeight));
              }
              constraints2.push_back(constraint);
              if (ikState == IKState::CONTACT_SEARCH) {
                poses.push_back(nextContacts[i]->localPose2);
                As.emplace_back(0,6);
                bs.emplace_back(0);
                Cs.push_back(nextContacts[i]->C);
                dls.push_back(nextContacts[i]->dl);
                dus.push_back(nextContacts[i]->du);
              }
            }
          }
        } else {
          std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
          constraint->A_link() = nextContacts[i]->link1;
          if ((ikState==IKState::DETACH_FIXED) ||
              (ikState==IKState::ATTACH_FIXED)) {
            for (int j=0; j<stopContacts.size(); j++) {
              if (nextContacts[i]->name == stopContacts[j]->name) {
                constraint->A_localpos() = stopContacts[j]->localPose1;
              }
            }
          } else if ((ikState==IKState::DETACH) ||
                     (ikState==IKState::ATTACH) ||
                     (ikState==IKState::DETACH_SEARCH)  ||
                     (ikState==IKState::SLIDE)) {
            constraint->A_localpos() = nextContacts[i]->localPose1;
          } else {
            std::cerr << "Undefined IKState !!" << std::endl;
          }
          constraint->B_link() = nextContacts[i]->link2;
          if ((ikState==IKState::DETACH_FIXED) || (ikState==IKState::ATTACH_FIXED)) { // 離れられるかのチェックなのでいま接触している場所を目標にする.
            for (int j=0; j<stopContacts.size(); j++) {
              if (nextContacts[i]->name == stopContacts[j]->name) {
                constraint->B_localpos() = stopContacts[j]->localPose2;
              }
            }
          } else {
            constraint->B_localpos() = nextContacts[i]->localPose2;
          }
          if ((ikState==IKState::DETACH) ||
              (ikState==IKState::DETACH_FIXED) ||
              (ikState==IKState::DETACH_SEARCH)) constraint->B_localpos().translation() += nextContacts[i]->localPose2.rotation() * cnoid::Vector3(0,0,0.02);
          if ((ikState==IKState::ATTACH) ||
              (ikState==IKState::ATTACH_FIXED) ||
              (ikState==IKState::SLIDE)) calcIgnoreBoundingBox(param->constraints, nextContacts[i], 3);
          constraint->eval_link() = nullptr;
          constraint->eval_localR() = constraint->B_localpos().rotation();
          constraint->weight() = param->positionConstraintWeight;
          constraint->precision() = param->positionConstraintPrecision;
          constraints2.push_back(constraint);
          if (ikState == IKState::DETACH_SEARCH) { // stance探索のため、接触しているものとしてSCFRを作る
            constraint->weight()[3] = 0.0;
            constraint->weight()[4] = 0.0;
            constraint->weight()[5] = 0.0;
            poses.push_back(nextContacts[i]->localPose2);
            As.emplace_back(0,6);
            bs.emplace_back(0);
            Cs.push_back(nextContacts[i]->C);
            dls.push_back(nextContacts[i]->dl);
            dus.push_back(nextContacts[i]->du);
          }
          if (ikState==IKState::SLIDE) {
            for (int j=0; j<stopContacts.size(); j++) {
              if (nextContacts[i]->name == stopContacts[j]->name) {
                constraint->weight()[5] = constraint->precision() / 1e-3; // 摩擦制約の関係上一致させる必要がある
                scfrConstraint->positionConstraints().push_back(constraint);
                poses.push_back(stopContacts[j]->link1->T() * constraint->A_localpos());
                As.emplace_back(0,6);
                bs.emplace_back(0);
                Cs.push_back(stopContacts[i]->C);
                dls.push_back(stopContacts[i]->dl);
                dus.push_back(stopContacts[i]->du);
                calcIgnoreBoundingBox(param->constraints, stopContacts[i], 3);
              }
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
    constraints0.push_back(scfrConstraint);

    bool solved = false;
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints{constraints0, constraints1, constraints2, nominals};
    std::vector<std::shared_ptr<prioritized_qp_base::Task> > prevTasks;
    solved  =  prioritized_inverse_kinematics_solver2::solveIKLoop(variables,
                                                                   constraints,
                                                                   prevTasks,
                                                                   param->pikParam,
                                                                   tmpPath
                                                                   );
    if (!solved && // 単に勾配を降りるだけで解けるなら大域探索は行わない
        ((!param->useSwing && (ikState==IKState::DETACH)) || // この後swingで接触点探索を行うので大域探索はswingでやる
         (!param->useSwing && (ikState==IKState::DETACH_FIXED)) || // この後swingで接触点探索を行うので大域探索はswingでやる
         (ikState==IKState::ATTACH) ||
         (ikState==IKState::ATTACH_FIXED) ||
         (ikState==IKState::ATTACH_SEARCH) ||
         (ikState==IKState::DETACH_SEARCH) ||
         (ikState==IKState::SLIDE) ||
         (ikState==IKState::CONTACT_SEARCH) ||
         (ikState==IKState::SWING)) // 干渉判定ができないので触れるときはgikのpathが実行不可能なものが出てくる可能性があるが、単に姿勢だけを出す目的でのみ使う. 接触ローカル座標系の位置姿勢が変わらないのでめり込むような姿勢は出てきにくいはず.
        && param->useSwingGIK){
      std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > gikConstraints{constraints0, constraints1};
      param->gikParam.projectLink.resize(1);
      param->gikParam.projectLink[0] = nextContacts[0]->link1;
      param->gikParam.projectLocalPose = nextContacts[0]->localPose1;
      // 関節角度上下限を厳密に満たしていないと、omplのstart stateがエラーになるので
      for(int i=0;i<param->variables.size();i++){
        if(param->variables[i]->isRevoluteJoint() || param->variables[i]->isPrismaticJoint()) {
          param->variables[i]->q() = std::max(std::min(param->variables[i]->q(),param->variables[i]->q_upper()),param->variables[i]->q_lower());
        }
      }
      solved = global_inverse_kinematics_solver::solveGIK(variables,
                                                          gikConstraints,
                                                          constraints2,
                                                          nominals,
                                                          param->gikParam,
                                                          tmpPath);
    }

    if (path != nullptr) {
      path->clear();
      if (param->useInterpolatePath) *path = *tmpPath;
      else path->push_back(tmpPath->back());
    }

    // for ( int i=0; i<constraints0.size(); i++ ) {
    //   std::cerr << "constraints0: "<< constraints0[i]->isSatisfied() << std::endl;
    // }
    // for ( int i=0; i<constraints1.size(); i++ ) {
    //   constraints1[i]->debugLevel() = 2;
    //   constraints1[i]->updateBounds();
    //   std::cerr << "constraints1: "<< constraints1[i]->isSatisfied() << std::endl;
    // }
    // for ( int i=0; i<constraints2.size(); i++ ) {
    //   constraints2[i]->debugLevel() = 2;
    //   constraints2[i]->updateBounds();
    //   std::cerr << "constraints2: "<< constraints2[i]->isSatisfied() << std::endl;
    //   constraints2[i]->debugLevel() = 0;
    // }
    // for ( int i=0; i<nominals.size(); i++ ) {
    //   std::cerr << "nominals: "<< nominals[i]->isSatisfied() << std::endl;
    // }
    for (int i=0; i<param->constraints.size(); i++) {
      if (typeid(*(param->constraints[i]))==typeid(ik_constraint2_distance_field::DistanceFieldCollisionConstraint)) {
        std::static_pointer_cast<ik_constraint2_distance_field::DistanceFieldCollisionConstraint>(param->constraints[i])->ignoreBoundingBox().clear();
        for (int j=0; j<nextContacts.size();j++) {
          if (nextContacts[j]->name == std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param->constraints[i])->A_link()->name()) {
            std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param->constraints[i])->tolerance() = param->envCollisionDefaultTolerance;
            std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param->constraints[i])->precision() = param->envCollisionDefaultPrecision;
          }
        }
      }
    }
    if ((ikState==IKState::ATTACH_SEARCH) || (ikState==IKState::SWING) || (ikState == IKState::CONTACT_SEARCH)) {
      param->pikParam.dqWeight.resize(6+param->robot->numJoints());
      param->gikParam.pikParam.dqWeight.resize(6+param->robot->numJoints());
      for (int i=0; i<nextContacts.size(); i++) {
        for (int j=0; j<param->bodyContactConstraints.size(); j++) {
          if (nextContacts[i]->name == param->bodyContactConstraints[j]->A_link()->name()) {
            // nextContactは次のcurrentContactに使われる‥このため接触局所位置を変えたならnextContact側に反映する必要がある.
            // そのままparam->bodyContactConstraints[j]->A_localpos();を代入すると、接触点探索の結果角を超えて姿勢が大きく変わったときもその姿勢がnextContactとなり、currentContactに代入されて次の不動接触目標とされてしまう.
            // 次の不動接触目標とされても良いように、現在の環境の姿勢と合わせておく
            nextContacts[i]->localPose1.translation() = param->bodyContactConstraints[j]->A_localpos().translation();
            nextContacts[i]->localPose1.linear() = param->bodyContactConstraints[j]->A_link()->R().transpose() * param->bodyContactConstraints[j]->B_localpos().linear();
          }
        }
      }
    }

    return solved;
  }

  void calcLevelLinks(const cnoid::LinkPtr inputLink,
                      int level, // input
                      std::vector<cnoid::LinkPtr>& targetLinks // output
                      ){ // inputLinkのlevel等親のリンクをtargetLinksとして返す.
    targetLinks.clear();
    targetLinks.push_back(inputLink);
    for (int iter=0; iter<level; iter++) {
      int prevLevelSize = targetLinks.size();
      for (int i=0; i<prevLevelSize; i++) {
        if ((targetLinks[i]->parent() != nullptr) && (std::find(targetLinks.begin(), targetLinks.end(), targetLinks[i]->parent()) == targetLinks.end())) targetLinks.push_back(targetLinks[i]->parent());
        cnoid::LinkPtr child = targetLinks[i]->child();
        while (child != nullptr) {
          if (std::find(targetLinks.begin(), targetLinks.end(), child) == targetLinks.end()) targetLinks.push_back(child);
          child = child->sibling();
        }
      }
    }
  }

  void calcIgnoreBoundingBox(const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& constraints,
                             const std::shared_ptr<Contact>& contact,
                             int level
                             ) { // constraint中のcollisionConstraintについて、contactのlink1のlevel等親のリンクの干渉回避である場合、contactのlink1のBoundingBoxを追加する.
    std::vector<cnoid::LinkPtr> targetLinks;
    calcLevelLinks(contact->link1, level, targetLinks);

    for (int i=0; i<constraints.size(); i++) {
      if (typeid(*(constraints[i]))==typeid(ik_constraint2_distance_field::DistanceFieldCollisionConstraint)) {
        if (std::find(targetLinks.begin(), targetLinks.end(), std::static_pointer_cast<ik_constraint2::CollisionConstraint>(constraints[i])->A_link()) != targetLinks.end()) {
          if (std::static_pointer_cast<ik_constraint2::CollisionConstraint>(constraints[i])->A_link() == contact->link1) continue; // この関数が呼ばれるのはsolveContactIK中で、接触リンクそのもののcollisionConstraintはそもそもconstraintに入っていない. よってcontactと一致するものはどのみちconstraintに入らないが、下と仕様をそろえるため.
          ik_constraint2_distance_field::DistanceFieldCollisionConstraint::BoundingBox ignoreBoundingBox;
          ignoreBoundingBox.parentLink = contact->link1;
          ignoreBoundingBox.localPose.translation() = contact->bbx.center();
          ignoreBoundingBox.dimensions = contact->bbx.max() - contact->bbx.min();
          std::static_pointer_cast<ik_constraint2_distance_field::DistanceFieldCollisionConstraint>(constraints[i])->ignoreBoundingBox().push_back(ignoreBoundingBox);
        }
      }
    }

  }

  void calcIgnoreBoundingBox(const std::vector<std::shared_ptr<ik_constraint2_distance_field::DistanceFieldCollisionConstraint> >& constraints,
                             const std::shared_ptr<Contact>& contact,
                             int level
                             ) { // constraint中のcollisionConstraintについて、contactのlink1のlevel等親のリンクの干渉回避である場合、contactのlink1のBoundingBoxを追加する.
    std::vector<cnoid::LinkPtr> targetLinks;
    calcLevelLinks(contact->link1, level, targetLinks);

    for (int i=0; i<constraints.size(); i++) {
      if (std::find(targetLinks.begin(), targetLinks.end(), constraints[i]->A_link()) != targetLinks.end()) {
        if (constraints[i]->A_link() == contact->link1) continue; // この関数が呼ばれるのはgenerateCondition中で、reachabilityConstraintで足裏が地面に触れるときに脛も触れるとされては困るため. よってcontactと一致するものは入れなくて良い
        ik_constraint2_distance_field::DistanceFieldCollisionConstraint::BoundingBox ignoreBoundingBox;
        ignoreBoundingBox.parentLink = contact->link1;
        ignoreBoundingBox.localPose.translation() = contact->bbx.center();
        ignoreBoundingBox.dimensions = contact->bbx.max() - contact->bbx.min();
        constraints[i]->ignoreBoundingBox().push_back(ignoreBoundingBox);
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

  cnoid::SgMeshPtr convertToSgMesh (const cnoid::SgNodePtr collisionshape){

    if (!collisionshape) return nullptr;

    std::shared_ptr<cnoid::MeshExtractor> meshExtractor = std::make_shared<cnoid::MeshExtractor>();
    cnoid::SgMeshPtr model = new cnoid::SgMesh; model->getOrCreateVertices();
    if(meshExtractor->extract(collisionshape, [&]() { addMesh(model,meshExtractor); })){
    }else{
      //      std::cerr << "[convertToSgMesh] meshExtractor->extract failed " << collisionshape->name() << std::endl;
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
