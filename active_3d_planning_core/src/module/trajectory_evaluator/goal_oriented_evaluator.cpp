#include "active_3d_planning_core/module/trajectory_evaluator/goal_oriented_evaluator.h"

#include <algorithm>

namespace active_3d_planning
{
namespace trajectory_evaluator
{
// Factory Registration
ModuleFactoryRegistry::Registration<GoalOrientedEvaluator>
    GoalOrientedEvaluator::registration("GoalOrientedEvaluator");

GoalOrientedEvaluator::GoalOrientedEvaluator(PlannerI &planner)
    : SimulatedSensorEvaluator(planner) {}

void GoalOrientedEvaluator::setupFromParamMap(Module::ParamMap *param_map)
{
    setParam<double>(param_map, "goal_weight",
                             &p_goal_weight_, 100.0);
    // setup parent
    SimulatedSensorEvaluator::setupFromParamMap(param_map);
}

bool GoalOrientedEvaluator::computeGainFromVisibleVoxels(TrajectorySegment *traj_in)
{
    Eigen::Vector3d far_goal{0,40,1.5};
    float distance_to_goal = (far_goal - traj_in->trajectory.back().position_W).norm();

    if (!traj_in->info)
    {
        traj_in->gain = 0.0;
        return false;
    }
    // remove all already observed voxels, count number of new voxels
    SimulatedSensorInfo *info =
        reinterpret_cast<SimulatedSensorInfo *>(traj_in->info.get());
    info->visible_voxels.erase(
        std::remove_if(info->visible_voxels.begin(), info->visible_voxels.end(),
                       [this](const Eigen::Vector3d &voxel) {
                           return planner_.getMap().isObserved(voxel);
                       }),
        info->visible_voxels.end());
    traj_in->gain = (double)info->visible_voxels.size() + p_goal_weight_/distance_to_goal;
    return true;
}

} // namespace trajectory_evaluator
} // namespace active_3d_planning
