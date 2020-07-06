#include "active_3d_planning_core/module/trajectory_evaluator/cost_computer/segment_time.h"
#include <ros/ros.h>
#include <math.h>
namespace active_3d_planning
{
    namespace cost_computer
    {

        // SegmentTime
        ModuleFactoryRegistry::Registration<SegmentTime>
            SegmentTime::registration("SegmentTime");

        SegmentTime::SegmentTime(PlannerI &planner) : CostComputer(planner)
        {

            last_global_goal_ = Eigen::Vector3d{0.0, 0.0, 0.0};
        };

        void SegmentTime::setupFromParamMap(Module::ParamMap *param_map)
        {
            setParam<bool>(param_map, "accumulate", &p_accumulate_, false);
            setParam<bool>(param_map, "penalize_altitude_change", &p_altitude_change_, false);
            setParam<bool>(param_map, "penalize_yaw_change", &p_yaw_change_, false);
            setParam<bool>(param_map, "penalize_distance_to_goal", &p_distance_to_goal_, false);
            setParam<double>(param_map, "start_time", &start_time_, ros::Time::now().toSec());
            setParam<double>(param_map, "time_weight", &p_time_weight_, 0.5);
            setParam<double>(param_map, "time_limit", &p_time_limit_, 10);
            setParam<double>(param_map, "exp_mult", &p_exp_mult_, 1000);
            setParam<double>(param_map, "exp_div", &p_exp_div_, 5);
        }

        bool SegmentTime::computeCost(TrajectorySegment *traj_in, Eigen::Vector3d *current_position)
        {

            if (traj_in->tg_visited)
            {
                return true;
            }
            if (traj_in->trajectory.size() < 2)
            {
                traj_in->cost = 0.0;
                return false;
            }
            traj_in->cost =
                static_cast<double>(traj_in->trajectory.back().time_from_start_ns -
                                    traj_in->trajectory.front().time_from_start_ns) *
                1.0e-9 * p_exp_mult_;
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

            if (p_distance_to_goal_)
            {
                double time_spent = ros::Time::now().toSec() - start_time_;
                double time_limit_sec = p_time_limit_ * 60;
                double time_remaining = std::max(time_limit_sec - time_spent, 0.0);

                Eigen::Vector2d distance_vector{global_goal_[0] - traj_in->trajectory.back().position_W.x(), global_goal_[1] - traj_in->trajectory.back().position_W.y()};
                double goal_distance = distance_vector.norm();

                Eigen::Vector2d current_distance_vector{global_goal_[0] - (*current_position)[0], global_goal_[1] - (*current_position)[1]};
                double current_goal_distance = current_distance_vector.norm();

                double diff_distance = goal_distance - current_goal_distance;
                double scaling = (time_limit_sec/5) * 1/(time_remaining/time_limit_sec);
                if (goal_distance > (current_goal_distance + (time_remaining/scaling)))
                {
                    traj_in->cost += 1000;
                }
        
             
            }
            return true;
        }

        bool SegmentTime::setGlobalGoal(Eigen::Vector3d *goal)
        {
            global_goal_[0] = (*goal)[0];
            global_goal_[1] = (*goal)[1];
            global_goal_[2] = (*goal)[2];

            if (!((last_global_goal_ - global_goal_).norm() == 0))
            {

                last_global_goal_ = global_goal_;
                new_global_goal_ = true;
                start_time_ = ros::Time::now().toSec();
            }
            return true;
        }

    } // namespace cost_computer
} // namespace active_3d_planning
