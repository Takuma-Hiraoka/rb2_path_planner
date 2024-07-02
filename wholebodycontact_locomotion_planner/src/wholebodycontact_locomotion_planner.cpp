#include <wholebodycontact_locomotion_planner/wholebodycontact_locomotion_planner.h>
#include <cnoid/MeshExtractor>
#include <cnoid/YAMLReader>
#include <choreonoid_qhull/choreonoid_qhull.h>

namespace wholebodycontact_locomotion_planner{
  void frame2Nominals(const std::vector<double>& frame, const std::vector<cnoid::LinkPtr>& links, std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominals) {
    nominals.clear();
    unsigned int i=0;
    for(int l=0;l<links.size();l++){
      if(links[l]->isRevoluteJoint() || links[l]->isPrismaticJoint()) {
        std::shared_ptr<ik_constraint2::JointAngleConstraint> constraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
        constraint->joint() = links[l];
        constraint->targetq() = frame[i];
        constraint->precision() = 1e10; // always satisfied
        nominals.push_back(constraint);
        i+=1;
      }else if(links[l]->isFreeJoint()) {
        i+=7;
      }
    }
  }
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
        constraint->B_link() = param->currentContactPoints[i]->link2;
        constraint->B_localpos() = param->currentContactPoints[i]->localPose2;
        constraint->B_localpos().translation() += param->currentContactPoints[i]->localPose1.linear() * cnoid::Vector3(0,0,0.02); // 0.02だけ離す
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
            contact->localPose1 = cnoid::Isometry3::Identity(); // 前後のstateの接触位置は基本的に同じであるためWBLP時に決定する
            contact->link2 = nullptr;
            contact->localPose2.translation() = it->second->reachabilityConstraints[j]->B_currentLocalp();
            cnoid::Vector3d z_axis = it->second->reachabilityConstraints[j]->currentDirection();
            cnoid::Vector3d x_axis = (z_axis==cnoid::Vector3d::UnitY()) ? cnoid::Vector3d::UnitZ() : cnoid::Vector3d::UnitY().cross(z_axis);
            cnoid::Vector3d y_axis = z_axis.cross(x_axis);
            contact->localPose2.linear().col(0) = x_axis.normalized(); contact->localPose2.linear().col(1) = y_axis.normalized(); contact->localPose2.linear().col(2) = z_axis.normalized();
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

  bool solveWBLP(const std::shared_ptr<WBLPParam>& param,
                 const std::vector<std::pair<std::vector<double>,std::vector<std::shared_ptr<Contact> > > >& guidePath,
                 std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > >& outputPath // angle, contact
                 ) {
    if (param->debugLevel >= 2) {
      std::cerr << "[solveWBLP] start. guide path size : " << guidePath.size() << std::endl;
    }
    outputPath.clear();
    int pathId=0;
      std::vector<std::shared_ptr<Contact> > currentContact = param->currentContactPoints;
    while(pathId < guidePath.size()) {
      // 接触が切り替わる直前,または進むindexの最大値だけ進んだIDを探す
      // 接触リンクが同じで接触面が異なる場合も、はじめのdetach-attachで遷移可能.
      int nextId;
      bool change = false;
      for (nextId=pathId+1;nextId<guidePath.size() && !change && nextId<pathId + param->maxSubGoalIdx;nextId++) {
        if(currentContact.size() != guidePath[nextId].second.size()) break;
        for (int i=0; i<currentContact.size(); i++) {
          if (currentContact[i]->name != guidePath[nextId].second[i]->name) {
            change = true;
            break;
          }
        }
      }
      if (param->debugLevel >= 2) {
        std::cerr << "[solveWBLP] current pathId : " << pathId << ". nextId :  " << nextId << std::endl;
      }

      // nextId-1になるまで、離れているものから順に近づけていく
      // 離れているものをできる限り近づける(guide path無視)と、スタックする恐れがある
      std::vector<int> subgoalIdQueue = std::vector<int>{nextId-1};
      std::vector<std::shared_ptr<Contact> > moveCandidate = currentContact;
      std::vector<std::vector<std::shared_ptr<Contact> > > moveCandidateQueue; // 複数接触が同じidまで進む場合、複数Contactを同時にCandidateに追加するため.
      for (int iter = 0; iter<param->maxContactIter;iter++) {
        if (param->debugLevel >= 2) {
          std::cerr << "[solveWBLP] current pathId : " << pathId << ". contact iter :  " << iter << ". moveCandidate : [";
          for (int i=0;i<moveCandidate.size();i++) std::cerr << moveCandidate[i]->name << " ";
          std::cerr << "]"<< std::endl;
          std::cerr << "subgoalIdQueue : [";
          for (int i=0;i<subgoalIdQueue.size();i++) std::cerr << subgoalIdQueue[i] << " ";
          std::cerr << "]"<< std::endl;
        }

        // moveCandidateのうち、guidePath[subgoalIdQueue.back()]と比較して最も離れているcontactを一つ選ぶ
        int moveContactId; // in moveCandidate
        int moveContactPathId; // in guidePath
        double maxDist = 0;
        for (int i=0; i<moveCandidate.size(); i++) {
          for (int j=0; j<guidePath[subgoalIdQueue.back()].second.size(); j++) {
            if (moveCandidate[i]->name == guidePath[subgoalIdQueue.back()].second[j]->name) {
              double dist = (moveCandidate[i]->localPose2.translation() - guidePath[subgoalIdQueue.back()].second[j]->localPose2.translation()).array().abs().sum();
              cnoid::AngleAxis angleAxis = cnoid::AngleAxis(moveCandidate[i]->localPose2.linear().transpose() * guidePath[subgoalIdQueue.back()].second[j]->localPose2.linear());
              dist += (angleAxis.angle()*angleAxis.axis()).array().abs().sum();
              if (dist > maxDist) {
                maxDist = dist;
                moveContactId = i;
                moveContactPathId = j;
              }
            }
          }
        }
        if (param->debugLevel >= 2) {
          std::cerr << "[solveWBLP] target move Contact : " << moveCandidate[moveContactId]->name << std::endl;
        }
        // 選ばれたcontactだけ、detach-attachでIKが解けなくなるまでguidePathを進める
        std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > > path;
        std::vector<double> pathInitialFrame;
        global_inverse_kinematics_solver::link2Frame(param->variables, pathInitialFrame);
        int idx;
        std::vector<double> lastLandingFrame;
        for (idx=pathId; idx<=subgoalIdQueue.back(); idx++) {
          std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals;
          frame2Nominals(guidePath[idx].first, param->variables, nominals);
          std::vector<double> frame;
          if (!solveContactIK(param, currentContact, std::vector<std::shared_ptr<Contact> >{guidePath[idx].second[moveContactPathId]}, nominals, false, false)) break;
          global_inverse_kinematics_solver::link2Frame(param->variables, frame);
          calcContactPoint(param, std::vector<std::shared_ptr<Contact> >{guidePath[idx].second[moveContactPathId]});
          if (solveContactIK(param, currentContact, std::vector<std::shared_ptr<Contact> >{guidePath[idx].second[moveContactPathId]}, nominals, true, false)) { // 着地も可能
            path.push_back(std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > >(frame, currentContact)); // TODO currentContactからmoveContactを除くこと
            global_inverse_kinematics_solver::link2Frame(param->variables, lastLandingFrame);
            global_inverse_kinematics_solver::frame2Link(frame, param->variables);
            param->robot->calcForwardKinematics(false);
            param->robot->calcCenterOfMass();
            if(param->debugLevel >= 3){
              if(param->viewer){
                param->viewer->drawObjects();
              }
              std::cerr << "proceed detach-attach path. current index : " << idx << " Press ENTER:" << std::endl;
              getchar();
            }
          } else {
            break;
          }
        }
        idx--;
        if(idx >= pathId) { // detach-attachで一つでも進むことができる場合. idx==pathIdのときは動いていないが、スタックしているときにIKを解くことを繰り返すことにより先にすすめることがある. TODO 解けないケースの対処法

          global_inverse_kinematics_solver::frame2Link(lastLandingFrame, param->variables);
          param->robot->calcForwardKinematics(false);
          param->robot->calcCenterOfMass();
          if(param->debugLevel >= 3){
            if(param->viewer){
              param->viewer->drawObjects();
            }
            std::cerr << "landing. current index : " << idx << " Press ENTER:" << std::endl;
            getchar();
          }

          path.push_back(std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > (lastLandingFrame, currentContact)); // TODO currentContactからmoveContactを除くこと
        } else { // detach-attachでは1stepも進めない場合
          path.clear();
          global_inverse_kinematics_solver::frame2Link(pathInitialFrame, param->variables);
          // 選ばれたcontactだけ、slideでIKが解けなくなるまでguidePathを進める
          for (idx=pathId; idx<=subgoalIdQueue.back(); idx++) {
            std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals;
            frame2Nominals(guidePath[idx].first, param->variables, nominals);
            for (int i=0;i<currentContact.size();i++) { // detach-attachのときはattachでguidePathのcontactのlocalPose1を書き換える. slideではここでcurrentContactに置き換える
              if (currentContact[i]->name == guidePath[idx].second[moveContactPathId]->name) {
                guidePath[idx].second[moveContactPathId]->localPose1 = currentContact[i]->localPose1;
              }
            }
            if (!solveContactIK(param, currentContact, std::vector<std::shared_ptr<Contact> >{guidePath[idx].second[moveContactPathId]}, nominals, true, true)) break;

            if(param->debugLevel >= 3){
              if(param->viewer){
                param->viewer->drawObjects();
              }
              std::cerr << "proceed slide path. current index : " << idx << " Press ENTER:" << std::endl;
              getchar();
            }

            std::vector<double> frame;
            global_inverse_kinematics_solver::link2Frame(param->variables, frame);
            outputPath.push_back(std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > (frame, currentContact));
          }
          idx--;
          // TODO
          if(idx >= pathId) {
          } else { // detach-attachでもslideでも動けない
            std::cerr << "cannot move contact" << std::endl;
            return false;
          }
        }

        if(param->debugLevel >= 3){
          if(param->viewer){
            param->viewer->drawObjects();
          }
        }
        // currentContactを更新
        for (int i=0; i<currentContact.size(); i++) {
          if (currentContact[i]->name == guidePath[idx].second[moveContactPathId]->name) {
            currentContact[i]->localPose1 = guidePath[idx].second[moveContactPathId]->localPose1;
            currentContact[i]->localPose2 = currentContact[i]->link1->T() * currentContact[i]->localPose1;
          }
        }

        // subgoalIdQueueとmoveCandidateの更新
        if (idx == subgoalIdQueue.back()) { // 目標の接触状態に達した
          if(moveCandidate.size() > 1) { // まだsubgoalIdQueueに達していない接触がある
            if (moveCandidateQueue.size() != 0) {
              moveCandidateQueue.back().push_back(moveCandidate[moveContactId]);
            } else {
              moveCandidateQueue.push_back(std::vector<std::shared_ptr<Contact> >{moveCandidate[moveContactId]});
            }
            moveCandidate.erase(moveCandidate.begin() + moveContactId);
          } else { // 全ての接触がsubgoalIdQueueに達している
            subgoalIdQueue.pop_back();
            moveCandidate.insert(moveCandidate.end(), moveCandidateQueue.back().begin(), moveCandidateQueue.back().end());
            pathId = idx;
            if (subgoalIdQueue.size() == 0) break;
          }
        } else { // 目標の接触状態に達しなかった
          if (moveCandidate.size() > 1) {// 動かしていない他の接触を動かして、この接触状態を目指す.
            subgoalIdQueue.push_back(idx);
            moveCandidateQueue.push_back(std::vector<std::shared_ptr<Contact> >{moveCandidate[moveContactId]});
            moveCandidate.erase(moveCandidate.begin() + moveContactId);
          } else { // 全ての接触を動かしたのに、pathを一つも進めなかった
            std::cerr << "no movable contact exist" << std::endl;
            return false;
          }
        }

        outputPath.insert(outputPath.end(), path.begin(), path.end());
      }
      // 接触の増加・減少処理
      if (pathId==guidePath.size()-1) { // 計画完了
        break;
      } else {
        std::vector<std::shared_ptr<Contact> > detachContact;
        for (int i=0;i<currentContact.size();i++) {
          bool detach = true;
          for (int j=0;j<guidePath[nextId].second.size() && detach;j++) {
            if (guidePath[nextId].second[j]->name == currentContact[i]->name) detach = false;
          }
          if(detach) detachContact.push_back(currentContact[i]);
        }
        std::vector<std::shared_ptr<Contact> > attachContact;
        for (int i=0;i<guidePath[nextId].second.size();i++) {
          bool attach = true;
          for (int j=0;j<currentContact.size() && attach;j++) {
            if (guidePath[nextId].second[i]->name == currentContact[j]->name) attach = false;
          }
          if(attach) attachContact.push_back(guidePath[nextId].second[i]);
        }
        if (detachContact.size() != 0 || attachContact.size() != 0) {
          // まず接触を離す
          {
            std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals;
            frame2Nominals(guidePath[nextId].first, param->variables, nominals);
            if(!solveContactIK(param, currentContact, detachContact, nominals, false, false)) {
              std::cerr << "cannot detach contact" << std::endl;
              break;
            }

            // currentContactを更新
            for (int i=0; i<currentContact.size(); i++) {
              for (int j=0;j<detachContact.size(); j++) {
                if (currentContact[i]->name == detachContact[j]->name) {
                  currentContact.erase(currentContact.begin() + i);
                }
              }
            }

            std::vector<double> frame;
            global_inverse_kinematics_solver::link2Frame(param->variables, frame);
            outputPath.push_back(std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > (frame, currentContact));
          }
          if(param->debugLevel >= 3){
            if(param->viewer){
              param->viewer->drawObjects();
            }
          }
          // 追加する接触のリンク座標を決める.
          {
            std::vector<double> preContactPose;
            global_inverse_kinematics_solver::link2Frame(param->variables, preContactPose);

            global_inverse_kinematics_solver::frame2Link(guidePath[nextId].first,param->variables);
            param->robot->calcForwardKinematics(false);
            param->robot->calcCenterOfMass();
            calcContactPoint(param, attachContact);

            // 接触を追加する
            global_inverse_kinematics_solver::frame2Link(preContactPose, param->variables);
            param->robot->calcForwardKinematics(false);
            param->robot->calcCenterOfMass();
            std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals;
            frame2Nominals(guidePath[nextId].first, param->variables, nominals);
            if(!solveContactIK(param, currentContact, attachContact, nominals, false, false)) {
              std::cerr << "cannot pre attach contact" << std::endl;
              break;
            }

            // currentContactを更新
            currentContact.insert(currentContact.end(), attachContact.begin(), attachContact.end());

            std::vector<double> frame;
            global_inverse_kinematics_solver::link2Frame(param->variables, frame);
            outputPath.push_back(std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > (frame, currentContact)); // TODO currentContactからattachContactを除くこと
            if(param->debugLevel >= 3){
              if(param->viewer){
                param->viewer->drawObjects();
              }
            }
            if(!solveContactIK(param, currentContact, attachContact, nominals, true, false)) {
              std::cerr << "cannot attach contact" << std::endl;
              break;
            }
            global_inverse_kinematics_solver::link2Frame(param->variables, frame);
            outputPath.push_back(std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > (frame, currentContact)); // TODO currentContactからattachContactを除くこと
            if(param->debugLevel >= 3){
              if(param->viewer){
                param->viewer->drawObjects();
              }
            }
          }
        }
        pathId++; // 接触の増加・減少を行った
      }
    }
    return true;
  }

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
  void createContactPoints(const std::shared_ptr<WBLPParam>& param,
                           std::string contactFileName
                           ) {
    param->contactPoints.clear();
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
          {
            bool found=false;
            for (int j=0;j<param->robot->numLinks();j++) {
              if (param->robot->link(j)->name() == linkName) found = true; 
            }
            if (!found) {
              for (int j=0;j<param->robot->numLinks();j++) {
                if (param->robot->link(j)->collisionShape()->child(0)->name() == linkName) linkName = param->robot->link(j)->name();
              }
            }
          }
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
