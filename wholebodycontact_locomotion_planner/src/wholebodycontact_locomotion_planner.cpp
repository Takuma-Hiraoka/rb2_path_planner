#include <wholebodycontact_locomotion_planner/wholebodycontact_locomotion_planner.h>

namespace wholebodycontact_locomotion_planner{
  bool solveCBPath(const std::shared_ptr<Environment>& environment,
                   const cnoid::Isometry3 goal, // rootLink
                   const std::shared_ptr<WBLPParam>& param,
                   std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > >& outputPath // angle. environment contact candidate
                   ){
    std::vector<double> initialPose;
    global_inverse_kinematics_solver::link2Frame(param->variables, initialPose);


    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints0;
    {
      // pitch > 0
      std::shared_ptr<ik_constraint2::RegionConstraint> constraint = std::make_shared<ik_constraint2::RegionConstraint>();
      constraint->A_link() = param->robot->rootLink();
      constraint->A_localpos().translation() = cnoid::Vector3(0.1,0.0,0.0);
      constraint->B_link() = param->robot->rootLink();
      constraint->eval_link() = nullptr;
      constraint->weightR().setZero();
      constraint->C().resize(1,3);
      constraint->C().insert(0,2) = 1.0;
      constraint->dl().resize(1);
      constraint->dl()[0] = -1e10;
      constraint->du().resize(1);
      constraint->du()[0] = 0.0;
      //constraint->debugLevel() = 2;
      constraints0.push_back(constraint);
    }
    {
      // pitch < 90
      std::shared_ptr<ik_constraint2::RegionConstraint> constraint = std::make_shared<ik_constraint2::RegionConstraint>();
      constraint->A_link() = param->robot->rootLink();
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.1);
      constraint->B_link() = param->robot->rootLink();
      constraint->eval_link() = nullptr;
      constraint->weightR().setZero();
      constraint->C().resize(1,3);
      constraint->C().insert(0,2) = 1.0;
      constraint->dl().resize(1);
      constraint->dl()[0] = -1e10;
      constraint->du().resize(1);
      constraint->du()[0] = 0.0;
      //constraint->debugLevel() = 2;
      constraints0.push_back(constraint);
    }
    {
      // roll = 0
      std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
      constraint->A_link() = param->robot->rootLink();
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.1,0.0);
      constraint->B_link() = param->robot->rootLink();
      constraint->B_localpos().translation() = cnoid::Vector3(0.0,-0.1,0.0);
      constraint->eval_link() = nullptr;
      constraint->weight() << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
      //constraint->debugLevel() = 2;
      constraints0.push_back(constraint);
    }
    constraints.push_back(constraints0);

    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints1;
    for (int i=0; i<param->constraints.size(); i++) {
      constraints1.push_back(param->constraints[i]);
    }

    // まず今触れている接触を僅かに離す. 初期状態をsatisfiedにするため.
    {
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > breakConstraints;
      for (int i=0; i<param->currentContactPoints.size(); i++) {
        // link1のlocalPoseはZ正方向が接触から離れる方向である前提
        std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
        constraint->A_link() = param->currentContactPoints[i]->link1;
        constraint->A_localpos() = param->currentContactPoints[i]->localPose1;
        constraint->A_localpos().translation() += param->currentContactPoints[i]->localPose1.linear() * cnoid::Vector3(0,0,-0.02); // 0.02だけ離す
        constraint->B_link() = param->currentContactPoints[i]->link2;
        constraint->B_localpos() = param->currentContactPoints[i]->localPose2;
        constraint->eval_link() = nullptr;
        breakConstraints.push_back(constraint);
      }
      std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > preConstraints{constraints0, constraints1, breakConstraints};
      std::vector<std::shared_ptr<prioritized_qp_base::Task> > prevTasks;
      prioritized_inverse_kinematics_solver2::solveIKLoop(param->variables,
                                                          preConstraints,
                                                          prevTasks,
                                                          param->gikRootParam.pikParam
                                                          );
    }

    std::shared_ptr<ik_constraint2::ORConstraint> conditions = std::make_shared<ik_constraint2::ORConstraint>();
    for(std::unordered_map<std::string, std::shared_ptr<Mode> >::const_iterator it=param->modes.begin(); it!=param->modes.end(); it++){
      conditions->children().push_back(it->second->generateCondition(environment, param->robot));
    }
    constraints1.push_back(conditions);
    constraints.push_back(constraints1);

    for (int i=0; i<constraints.size(); i++) {
      for (int j=0; j<constraints[i].size(); j++) {
        constraints[i][j]->debugLevel() = 0;
        constraints[i][j]->updateBounds();
      }
    }

    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > goals;
    {
      std::shared_ptr<ik_constraint2::PositionConstraint> goal_ = std::make_shared<ik_constraint2::PositionConstraint>();
      goal_->A_link() = param->robot->rootLink();
      goal_->B_link() = nullptr;
      goal_->B_localpos() = goal;
      goal_->precision() = 1e-2;
      goals.push_back(goal_);
    }

    param->gikRootParam.projectLink.push_back(param->robot->rootLink());
    param->gikRootParam.projectCellSize = 0.2;
    std::shared_ptr<std::vector<std::vector<double> > > path = std::make_shared<std::vector<std::vector<double> > >();
    if(!global_inverse_kinematics_solver::solveGIK(param->variables,
                                                   constraints,
                                                   goals,
                                                   param->nominals,
                                                   param->gikRootParam,
                                                   path
                                                   )){
      std::cerr << "solveCBPath failed" << std::endl;
      return false;
    }

    // 関節角軌道が暴れているので軌道最適化
    if (param->OptimizeTrajectory) {
      param->toParam.pikParam.convergeThre=param->gikRootParam.pikParam.convergeThre * path->size();
      trajectory_optimizer::solveTO(param->variables,
                                    constraints,
                                    param->toParam,
                                    path);
    }

    outputPath.resize(path->size());
    for (int i=0; i<path->size(); i++) {
      outputPath[i].first = path->at(i);
      global_inverse_kinematics_solver::frame2Link(path->at(i),param->variables);
      param->robot->calcForwardKinematics(false);
      param->robot->calcCenterOfMass();
      // isSatisfiedであるリンクを全て接触させればSCFRが存在するmodeがすくなくとも一つあることをGIKで保証済み
      // TODO mode選択. どのみち全リンク接触でも優先度をつける等の工夫をするならmode自体が不要？
      std::vector<std::shared_ptr<Contact> >contacts;
      for (std::unordered_map<std::string, std::shared_ptr<Mode> >::const_iterator it=param->modes.begin(); it!=param->modes.end(); it++){
        for (int j=0; j<it->second->reachabilityConstraints.size(); j++) {
          it->second->reachabilityConstraints[j]->updateBounds();
          if (it->second->reachabilityConstraints[j]->isSatisfied()) {
            std::shared_ptr<Contact> contact = std::make_shared<Contact>();
            contact->name = it->second->reachabilityConstraints[j]->A_link()->name();
            contact->link1 = it->second->reachabilityConstraints[j]->A_link();
            contact->localPose1 = cnoid::Isometry3::Identity(); // WBLP時に決定する
            contact->link2 = nullptr;
            contact->localPose2.translation() = it->second->reachabilityConstraints[j]->currentp(); // TODO 環境から姿勢を出す.
            Eigen::SparseMatrix<double,Eigen::RowMajor> C(11,6); // TODO 干渉形状から出す？
            C.insert(0,2) = 1.0;
            C.insert(1,0) = 1.0; C.insert(1,2) = 0.2;
            C.insert(2,0) = -1.0; C.insert(2,2) = 0.2;
            C.insert(3,1) = 1.0; C.insert(3,2) = 0.2;
            C.insert(4,1) = -1.0; C.insert(4,2) = 0.2;
            C.insert(5,2) = 0.05; C.insert(5,3) = 1.0;
            C.insert(6,2) = 0.05; C.insert(6,3) = -1.0;
            C.insert(7,2) = 0.05; C.insert(7,4) = 1.0;
            C.insert(8,2) = 0.05; C.insert(8,4) = -1.0;
            C.insert(9,2) = 0.005; C.insert(9,5) = 1.0;
            C.insert(10,2) = 0.005; C.insert(10,5) = -1.0;
            contact->C = C;
            cnoid::VectorX dl = Eigen::VectorXd::Zero(11);
            contact->dl = dl;
            cnoid::VectorX du = 1e10 * Eigen::VectorXd::Ones(11);
            du[0] = 2000.0;
            contact->du = du;
            contacts.push_back(contact);
          }
        }
      }
      outputPath[i].second = contacts;
    }
    global_inverse_kinematics_solver::frame2Link(initialPose,param->variables);
    return true;

  }

  bool solveWBLP(const std::shared_ptr<Environment>& environment,
                 const std::shared_ptr<WBLPParam>& param,
                 const std::vector<std::pair<std::vector<double>,std::vector<std::shared_ptr<Contact> > > >& guidePath,
                 std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > >& outputPath // angle, contact
                 ) {
    int pathId=0;
    while(pathId < guidePath.size()) {
      std::vector<std::shared_ptr<Contact> > currentContact = (pathId == 0) ? param->currentContactPoints : guidePath[pathId].second;
      // 接触が切り替わる直前のIDを探す
      int nextId;
      for (nextId=pathId+1;nextId<guidePath.size();nextId++) {
        if(currentContact.size() != guidePath[nextId].second.size()) break;
        for (int i=0; i<currentContact.size(); i++) {
          if (currentContact[i]->name != guidePath[nextId].second[i]->name) break;
        }
      }

      std::vector<std::shared_ptr<Contact> > moveCandidate = currentContact;
      {
        // moveCandidateのうち、guidePath[nextId]と比較して最も離れているcontactを一つ選ぶ
        std::shared_ptr<Contact> moveContact;
        double maxDist = 0;
        for (int i=0; i<moveCandidate.size(); i++) {
          for (int j=0; j<guidePath[nextId].second.size(); j++) {
            if (moveCandidate[i]->name == guidePath[nextId].second[j]->name) {
              double dist = (moveCandidate[i]->localPose2.translation() - guidePath[nextId].second[j]->localPose2.translation()).sum();
              cnoid::AngleAxis angleAxis = cnoid::AngleAxis(moveCandidate[i]->localPose2.linear().transpose() * guidePath[nextId].second[j]->localPose2.linear());
              dist += (angleAxis.angle()*angleAxis.axis()).sum();
              if (dist > maxDist) {
                maxDist = dist;
                moveContact = moveCandidate[i];
              }
            }
          }
        }
        // 選ばれたcontactだけ、IKが解けなくなるまでguidePathを進める
        
      }
    }
    return true;
  }

  bool solveContactIK(const std::shared_ptr<WBLPParam>& param,
                      const std::vector<std::shared_ptr<Contact> >& stopContacts,
                      const std::vector<std::shared_ptr<Contact> >& nextContacts,
                      bool attach,
                      bool slide) {
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints0;
    for (int i=0; i<param->constraints.size(); i++) {
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
    {
      for (int i=0; i<stopContacts.size(); i++) {
        bool move=false;
        for (int j=0; j<nextContacts.size(); j++) {
          if (stopContacts[i]->name == nextContacts[j]->name) move=true;;
        }
        if (move) continue; // このcontactを動かす予定
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
        if (attach) constraint->A_localpos().translation() = nextContacts[i]->localPose1.linear() * cnoid::Vector3(0,0,-0.02); // 0.02だけ離す
        constraint->B_link() = nextContacts[i]->link2;
        constraint->B_localpos() = nextContacts[i]->localPose2;
        constraint->eval_link() = nullptr;
        constraint->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        constraints1.push_back(constraint);
        if (slide) {
          for (int j=0; j<stopContacts.size(); j++) {
            if (nextContacts[i]->name == stopContacts[j]->name) {
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
              cnoid::VectorX dl = Eigen::VectorXd::Zero(11);
              dls.push_back(dl);
              cnoid::VectorX du = 1e10 * Eigen::VectorXd::Ones(11);
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

    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints{constraints0,constraints1};
    prioritized_inverse_kinematics_solver2::IKParam pikParam;
    std::vector<std::shared_ptr<prioritized_qp_base::Task> > prevTasks;
    return prioritized_inverse_kinematics_solver2::solveIKLoop(param->variables,
                                                                      constraints,
                                                                      prevTasks,
                                                                      pikParam
                                                                      );
  }
}
