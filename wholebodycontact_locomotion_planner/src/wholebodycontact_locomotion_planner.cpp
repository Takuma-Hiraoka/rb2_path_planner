#include <wholebodycontact_locomotion_planner/wholebodycontact_locomotion_planner.h>
#include <wholebodycontact_locomotion_planner/Util.h>
#include <cnoid/YAMLReader>

namespace wholebodycontact_locomotion_planner{
  inline void frame2Nominals(const std::vector<double>& frame, const std::vector<cnoid::LinkPtr>& links, std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominals) {
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
        constraint->B_localpos().translation() += param->currentContactPoints[i]->localPose2.linear() * cnoid::Vector3(0,0,0.05); // 0.03だけ離す
        constraint->eval_link() = nullptr;
        breakConstraints.push_back(constraint);
      }

      std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > preConstraints{constraints0, constraints1, breakConstraints};
      std::vector<std::shared_ptr<prioritized_qp_base::Task> > prevTasks;
      prioritized_inverse_kinematics_solver2::solveIKLoop(param->variables,
                                                          preConstraints,
                                                          prevTasks,
                                                          param->pikParam
                                                          );
    }
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > rejections{conditions};
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
                                                   rejections,
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
            contact->localPose1.translation() =  it->second->reachabilityConstraints[j]->A_currentLocalp(); // WBLP時に接触点を決定する際に初期解として使えるように、最近傍点をいれておく
            contact->link2 = nullptr;
            contact->localPose2.translation() = it->second->reachabilityConstraints[j]->B_currentLocalp();
            cnoid::Vector3d z_axis = it->second->reachabilityConstraints[j]->currentDirection();
            cnoid::Vector3d x_axis = (z_axis==cnoid::Vector3d::UnitY() || z_axis==-cnoid::Vector3d::UnitY()) ? cnoid::Vector3d::UnitZ() : cnoid::Vector3d::UnitY().cross(z_axis);
            cnoid::Vector3d y_axis = z_axis.cross(x_axis);
            contact->localPose2.linear().col(0) = x_axis.normalized(); contact->localPose2.linear().col(1) = y_axis.normalized(); contact->localPose2.linear().col(2) = z_axis.normalized();
            contact->calcBoundingBox();
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
            du[0] = 20000.0;
            contact->du = du;
            contacts.push_back(contact);
          }
        }
      }
      outputPath[i].second = contacts;
    }
    global_inverse_kinematics_solver::frame2Link(initialPose,param->variables);
    param->robot->calcForwardKinematics(false);
    param->robot->calcCenterOfMass();
    return true;

  }

  bool solveCBStance(const std::shared_ptr<WBLPParam>& param,
                     const std::vector<std::pair<std::vector<double>,std::vector<std::shared_ptr<Contact> > > >& guidePath,
                     std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > >& outputPath // angle, contact
                     ) { // guidePathから、実際に接触した姿勢を作る. 特に接触点探索において幾何干渉のない接触点を作っておくことが目的
    // bbxを計算
    outputPath.clear();
    for (int i=0; i<guidePath.size(); i++) {
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals;
      frame2Nominals(guidePath[i].first, param->variables, nominals);
      global_inverse_kinematics_solver::frame2Link(guidePath[i].first, param->variables);
      param->robot->calcForwardKinematics(false);
      param->robot->calcCenterOfMass();
      if(!solveContactIK(param, std::vector<std::shared_ptr<Contact>>{}, guidePath[i].second, nominals, IKState::CONTACT_SEARCH)) {
        if (!solveContactIK(param, std::vector<std::shared_ptr<Contact>>{}, guidePath[i].second, nominals, IKState::DETACH_SEARCH)) {
          std::cerr << "[solveCBStance] failed." << std::endl;
          return false;
        }
      }
      std::vector<double> frame;
      global_inverse_kinematics_solver::link2Frame(param->variables, frame);
      outputPath.push_back(std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > >(frame, guidePath[i].second));
    }

    if (param->debugLevel >= 1) std::cerr << "[solveCBStance] succeeded." << std::endl;
    global_inverse_kinematics_solver::frame2Link(guidePath[0].first,param->variables);
    param->robot->calcForwardKinematics(false);
    param->robot->calcCenterOfMass();
    return true;
  }

  bool solveWBLP(const std::shared_ptr<WBLPParam>& param,
                 const std::vector<std::pair<std::vector<double>,std::vector<std::shared_ptr<Contact> > > >& guidePath,
                 std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > >& outputPath // angle, contact
                 ) {
    if (param->debugLevel >= 2) {
      std::cerr << "[solveWBLP] start. guide path size : " << guidePath.size() << std::endl;
    }

    // bbxを計算
    for (int i=0; i<param->currentContactPoints.size(); i++) {
      param->currentContactPoints[i]->calcBoundingBox();
    }
    outputPath.clear();
    int pathId=0;
    std::vector<std::shared_ptr<Contact> > currentContact = param->currentContactPoints;
    while(pathId < guidePath.size()) {
      // 接触が切り替わる直前,または進むindexの最大値だけ進んだIDを探す
      // 接触リンクが同じで接触面が異なる場合も、はじめのdetach-attachで遷移可能.
      int nextId;
      bool change = false;
      for (nextId=pathId+1;nextId<guidePath.size() && !change && nextId<=pathId + param->maxSubGoalIdx;nextId++) {
        if(currentContact.size() != guidePath[nextId].second.size()) change = true;
        for (int i=0; i<currentContact.size() && !change; i++) {
          if (currentContact[i]->name != guidePath[nextId].second[i]->name) {
            change = true;
          }
        }
        if (change) break;
      }
      if (param->debugLevel >= 2) {
        std::cerr << "[solveWBLP] current pathId : " << pathId << ". nextId :  " << nextId << std::endl;
      }

      // nextId-1(nextId++するので)になるまで、離れているものから順に近づけていく
      // 離れているものをできる限り近づける(guide path無視)と、スタックする恐れがある
      std::vector<int> subgoalIdQueue = std::vector<int>{change ? nextId : nextId-1};
      std::vector<std::shared_ptr<Contact> > moveCandidate;
      for (int i=0; i<currentContact.size(); i++) {
        for (int j=0; j<guidePath[subgoalIdQueue.back()].second.size(); j++) {
          if (currentContact[i]->name == guidePath[subgoalIdQueue.back()].second[j]->name) moveCandidate.push_back(currentContact[i]); // moveCandidateに追加するのは最後でも触れているものだけ. 最後に離す接触は動かさない.
        }
      }
      std::vector<std::vector<std::shared_ptr<Contact> > > moveCandidateQueue; // 0 : subgoalIdQueueに対応. そこまで達している. 1 : contact. 複数接触が同じidまで進む場合、複数Contactを同時にCandidateに追加するため.
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
        std::vector<int> moveContactIds; // 残りどのcontactが残っているか
        std::vector<std::string> moveContactLinks; // IKを解く際の目標
        // 手を動かした後腰を動かす場合、肩が戻ることになる
        // 今回動かすmoveContactsを求める
        {
          std::vector<cnoid::LinkPtr> targetLinks;
          calcLevelLinks(moveCandidate[moveContactId]->link1, param->moveContactsLevel, targetLinks);
          for (int i=0; i<moveCandidate.size(); i++) {
            if (std::find(targetLinks.begin(), targetLinks.end(), moveCandidate[i]->link1) != targetLinks.end()) moveContactIds.push_back(i);
          }
          for (int i=0; i<guidePath[pathId].second.size(); i++) {
            if (std::find(targetLinks.begin(), targetLinks.end(), guidePath[pathId].second[i]->link1) != targetLinks.end()) moveContactLinks.push_back(guidePath[pathId].second[i]->name);
          } // 接触の増加・減少がある場合最後のguidePathの大きさが異なる. IKを解く際の目標はpathIdにすでにあるもののみ
        }

        if (param->debugLevel >= 2) {
          std::cerr << "[solveWBLP] target move Contact : " << moveCandidate[moveContactId]->name << std::endl;
          std::cerr << "[solveWBLP] move Contacts : ";
          for (int i=0; i<moveContactLinks.size(); i++) std::cerr << moveContactLinks[i] << " ";
          std::cerr << std::endl;
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
          std::vector<std::shared_ptr<Contact> > moveContact;
          for (int i=0; i<guidePath[idx].second.size(); i++) {
            if (std::find(moveContactLinks.begin(), moveContactLinks.end(), guidePath[idx].second[i]->name) != moveContactLinks.end()) moveContact.push_back(guidePath[idx].second[i]);
          }
          if (!solveContactIK(param, currentContact, moveContact, nominals, idx==pathId ? IKState::DETACH_FIXED : IKState::DETACH)) break;
          global_inverse_kinematics_solver::link2Frame(param->variables, frame);
          if (solveContactIK(param, currentContact, moveContact, nominals, idx==pathId ? IKState::ATTACH_FIXED : IKState::ATTACH)) { // 着地も可能
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
        if(idx > pathId) { // detach-attachで一つでも進むことができる場合. TODO 解けないケースの対処法

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
          for (idx=pathId+1; idx<=subgoalIdQueue.back(); idx++) {
            std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals;
            frame2Nominals(guidePath[idx].first, param->variables, nominals);
            std::vector<std::shared_ptr<Contact> > moveContact;
            for (int i=0; i<guidePath[idx].second.size(); i++) {
              if (std::find(moveContactLinks.begin(), moveContactLinks.end(), guidePath[idx].second[i]->name) != moveContactLinks.end()) moveContact.push_back(guidePath[idx].second[i]);
            }

            if (!solveContactIK(param, currentContact, moveContact, nominals, IKState::SLIDE)) break;

            if(param->debugLevel >= 3){
              if(param->viewer){
                param->viewer->drawObjects();
              }
              std::cerr << "proceed slide path. current index : " << idx << " Press ENTER:" << std::endl;
              getchar();
            }

            std::vector<double> frame;
            global_inverse_kinematics_solver::link2Frame(param->variables, frame);
            path.push_back(std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > (frame, currentContact));
          }
          idx--;
          // TODO
          if(idx > pathId) {
          } else { // detach-attachでもslideでも動けない
            std::cerr << "cannot move contact" << std::endl;
            return false;
          }
        }

        outputPath.insert(outputPath.end(), path.begin(), path.end());
        if(param->debugLevel >= 3){
          if(param->viewer){
            param->viewer->drawObjects();
          }
        }
        // currentContactを更新
        for (int i=0; i<currentContact.size(); i++) {
          if (std::find(moveContactLinks.begin(), moveContactLinks.end(), currentContact[i]->name) == moveContactLinks.end()) continue; // このiterでは動かさなかった
          for (int j=0; j<guidePath[idx].second.size(); j++) {
            if (currentContact[i]->name == guidePath[idx].second[j]->name) {
              currentContact[i]->localPose1.translation() = guidePath[idx].second[j]->localPose1.translation();
              currentContact[i]->localPose1.linear() = currentContact[i]->link1->R().transpose() * guidePath[idx].second[j]->localPose2.linear();
              currentContact[i]->localPose2.translation() = currentContact[i]->link1->p() + currentContact[i]->link1->R() * currentContact[i]->localPose1.translation();
              currentContact[i]->localPose2.linear() = guidePath[idx].second[j]->localPose2.linear(); // localPose2はSCFRの計算にも使われ、特にZ方向が傾いてはいけない. localPose1は接触点探索時に傾きを考慮しない結果傾いているため、localPose1をlocalPose2に合わせる
            }
          }
        }

        if(param->debugLevel >= 2){
          std::cerr << "[solveWBLP] proceed : " << idx << std::endl;
        }
        // subgoalIdQueueとmoveCandidateの更新
        if (idx == subgoalIdQueue.back()) { // 目標の接触状態に達した
          if(moveCandidate.size() > moveContactIds.size()) { // まだsubgoalIdQueueに達していない接触がある
            std::vector<std::shared_ptr<Contact> > moveContacts;
            for(int i=0; i<moveContactIds.size(); i++) {
              moveContacts.push_back(moveCandidate[moveContactIds[i]]);
            }
            if (moveCandidateQueue.size() != 0) {
              for (int i=0; i<moveContacts.size(); i++) {
                if (std::find(moveCandidateQueue.back().begin(), moveCandidateQueue.back().end(), moveContacts[i]) == moveCandidateQueue.back().end()) moveCandidateQueue.back().push_back(moveCandidate[moveContactIds[i]]);
              }
            } else {
              moveCandidateQueue.push_back(moveContacts);
            }
            for (int i=0;i<moveContacts.size(); i++) {
              moveCandidate.erase(std::find(moveCandidate.begin(), moveCandidate.end(), moveContacts[i]));
            }
          } else { // 全ての接触がsubgoalIdQueueに達している
            subgoalIdQueue.pop_back();
            moveCandidate.insert(moveCandidate.end(), moveCandidateQueue.back().begin(), moveCandidateQueue.back().end());
            pathId = idx;
            if (subgoalIdQueue.size() == 0) break;
          }
        } else { // 目標の接触状態に達しなかった
          if (moveCandidate.size() > 1) {// 動かしていない他の接触を動かして、この接触状態を目指す.
            subgoalIdQueue.push_back(idx);
            std::vector<std::shared_ptr<Contact> > moveContacts;
            for(int i=0; i<moveContactIds.size(); i++) {
              moveContacts.push_back(moveCandidate[moveContactIds[i]]);
            }
            moveCandidateQueue.push_back(moveContacts);
            for (int i=0;i<moveContacts.size(); i++) {
              moveCandidate.erase(std::find(moveCandidate.begin(), moveCandidate.end(), moveContacts[i]));
            }
          } else { // 全ての接触を動かしたのに、pathを一つも進めなかった
            std::cerr << "no movable contact exist" << std::endl;
            return false;
          }
        }

      } // maxContactIter
      // 接触の増加・減少処理
      if (pathId==guidePath.size()-1) { // 計画完了
        if(param->debugLevel >= 0){
          std::cerr << "[solveWBLP] succeeded." << std::endl;
        }
        break;
      } else {
        // TODO 同時接触は必要？
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
        if (detachContact.size() != 0) {
          // まず接触を離す
          std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals;
          frame2Nominals(guidePath[nextId].first, param->variables, nominals);
          if(!solveContactIK(param, currentContact, detachContact, nominals, IKState::DETACH_FIXED)) {
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
          if(param->debugLevel >= 3){
            if(param->viewer){
              param->viewer->drawObjects();
            }
          }
        }

        if (attachContact.size() != 0) {
          // 接触を追加する
          std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals;
          frame2Nominals(guidePath[nextId].first, param->variables, nominals);
          if(!solveContactIK(param, currentContact, attachContact, nominals, IKState::SWING)) {
            std::cerr << "cannot pre attach contact" << std::endl;
            break;
          }

          std::vector<double> frame;
          global_inverse_kinematics_solver::link2Frame(param->variables, frame);
          outputPath.push_back(std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > (frame, currentContact)); // TODO currentContactからattachContactを除くこと
          if(param->debugLevel >= 3){
            if(param->viewer){
              param->viewer->drawObjects();
            }
          }
          if(!solveContactIK(param, currentContact, attachContact, nominals, IKState::ATTACH)) {
            std::cerr << "cannot attach contact" << std::endl;
            break;
          }
          global_inverse_kinematics_solver::link2Frame(param->variables, frame);
          // currentContactを更新
          for (int i=0; i<attachContact.size(); i++) {
            attachContact[i]->localPose1.linear() = attachContact[i]->link1->R().transpose() * attachContact[i]->localPose2.linear();
            attachContact[i]->localPose2.translation() = attachContact[i]->link1->p() + attachContact[i]->link1->R() * attachContact[i]->localPose1.translation(); // localPose2はSCFRの計算にも使われ、特にZ方向が傾いてはいけない. localPose1は接触点探索時に傾きを考慮しない結果傾いているため、localPose1をlocalPose2に合わせる
          }
          currentContact.insert(currentContact.end(), attachContact.begin(), attachContact.end());

          outputPath.push_back(std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > (frame, currentContact)); // TODO currentContactからattachContactを除くこと
          if(param->debugLevel >= 3){
            if(param->viewer){
              param->viewer->drawObjects();
            }
          }
        }
      }
    }
    return true;
  }
  std::unordered_map<std::string, std::vector<cnoid::Isometry3> >  createContactPoints(const std::shared_ptr<WBLPParam>& param,
                                                                                       std::string contactFileName
                                                                                       ) {
    cnoid::YAMLReader reader;
    cnoid::MappingPtr node;
    std::string prevLinkName = "";
    std::unordered_map<std::string, std::vector<cnoid::Isometry3> > contactPoints;
    std::vector<cnoid::Isometry3> contactPointsBuf;
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
              contactPoints[prevLinkName] = contactPointsBuf;
            }
            prevLinkName = linkName;
            contactPointsBuf.clear();
          }
          cnoid::Isometry3 sensor;
          // translation
          cnoid::ValueNodePtr translation_ = info->extract("translation");
          if(translation_){
            cnoid::ListingPtr translationTmp = translation_->toListing();
            if(translationTmp->size()==3){
              sensor.translation() = cnoid::Vector3(translationTmp->at(0)->toDouble(), translationTmp->at(1)->toDouble(), translationTmp->at(2)->toDouble());
            }
          }
          // rotation
          cnoid::ValueNodePtr rotation_ = info->extract("rotation");
          if(rotation_){
            cnoid::ListingPtr rotationTmp = rotation_->toListing();
            if(rotationTmp->size() == 4){
              sensor.linear() = cnoid::AngleAxisd(rotationTmp->at(3)->toDouble(),
                                                  cnoid::Vector3{rotationTmp->at(0)->toDouble(), rotationTmp->at(1)->toDouble(), rotationTmp->at(2)->toDouble()}).toRotationMatrix();
            }
          }
          contactPointsBuf.push_back(sensor);
        }
        if (prevLinkName != "") {
          contactPoints[prevLinkName] = contactPointsBuf;
        }
      }
    }
    return contactPoints;
  }
}
