#include <wholebodycontact_locomotion_planner/wholebodycontact_locomotion_planner.h>

namespace wholebodycontact_locomotion_planner{
  bool solveCBPath(const std::shared_ptr<Environment>& environment,
                   const cnoid::Isometry3 goal, // rootLink
                   const std::shared_ptr<WBLPParam>& param,
                   std::vector<std::vector<double> >& outputPath
                   ){
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints;
    {
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
    }

    std::shared_ptr<ik_constraint2::ORConstraint> conditions = std::make_shared<ik_constraint2::ORConstraint>();
    for(std::unordered_map<std::string, std::shared_ptr<Mode> >::const_iterator it=param->modes.begin(); it!=param->modes.end(); it++){
      conditions->children().push_back(it->second->generateCondition(environment, param->robot));
    }
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints1{conditions};
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
    outputPath.resize(path->size());
    for (int i=0; i<path->size(); i++) {
      outputPath[i] = path->at(i);
    }
    return true;

  }
}
