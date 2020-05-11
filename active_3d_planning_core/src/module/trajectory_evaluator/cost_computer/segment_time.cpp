#include "active_3d_planning_core/module/trajectory_evaluator/cost_computer/segment_time.h"

namespace active_3d_planning
{
namespace cost_computer
{

// SegmentTime
ModuleFactoryRegistry::Registration<SegmentTime>
    SegmentTime::registration("SegmentTime");

SegmentTime::SegmentTime(PlannerI &planner) : CostComputer(planner){};

void SegmentTime::setupFromParamMap(Module::ParamMap *param_map)
{
    setParam<bool>(param_map, "accumulate", &p_accumulate_, false);
    setParam<bool>(param_map, "penalize_altitude_change", &p_altitude_change_, false);
    setParam<bool>(param_map, "penalize_yaw_change", &p_yaw_change_, false);
}

bool SegmentTime::computeCost(TrajectorySegment *traj_in)
{
    if (traj_in->trajectory.size() < 2)
    {
        traj_in->cost = 0.0;
        return false;
    }
    traj_in->cost =
        static_cast<double>(traj_in->trajectory.back().time_from_start_ns -
                            traj_in->trajectory.front().time_from_start_ns) * 1.0e-9;
    if (p_accumulate_ && traj_in->parent)
    {
        traj_in->cost += traj_in->parent->cost;
    }

    double acc_altitude_change = 0;
    if (p_altitude_change_)
    {
        for (int i = 0; i < traj_in->trajectory.size(); ++i)
        {
            double altitude_change = abs(traj_in->trajectory[i].position_W.z() - traj_in->trajectory[i + 1].position_W.z());
            acc_altitude_change += altitude_change;
        }
        traj_in->cost += std::max(acc_altitude_change, 0.0);
    }
    return true;
}

} // namespace cost_computer
} // namespace active_3d_planning
