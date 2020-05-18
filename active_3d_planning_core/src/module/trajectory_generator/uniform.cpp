#define _USE_MATH_DEFINES

#include "active_3d_planning_core/module/trajectory_generator/uniform.h"
#include "active_3d_planning_core/data/system_constraints.h"

#include "active_3d_planning_core/data/trajectory.h"
#include "active_3d_planning_core/tools/defaults.h"

#include <cmath>
#include <random>

namespace active_3d_planning
{
namespace trajectory_generator
{

ModuleFactoryRegistry::Registration<Uniform> Uniform::registration("Uniform");

Uniform::Uniform(PlannerI &planner) : TrajectoryGenerator(planner) {}

void Uniform::setupFromParamMap(Module::ParamMap *param_map)
{
    TrajectoryGenerator::setupFromParamMap(param_map);
    setParam<double>(param_map, "distance", &p_distance_, 1.0);
    setParam<double>(param_map, "yaw_angle", &p_yaw_angle_, 1.571);
    setParam<double>(param_map, "ascent_angle", &p_ascent_angle_, 0.523);
    setParam<double>(param_map, "sampling_rate", &p_sampling_rate_, 20.0);
    setParam<int>(param_map, "n_segments", &p_n_segments_, 5);
    setParam<bool>(param_map, "goal_oriented", &p_goal_oriented_, false);
    setParam<double>(param_map, "goal_position_x", &p_goal_position_x_, 0.0);
    setParam<double>(param_map, "goal_position_y", &p_goal_position_y_, 50.0);
    setParam<double>(param_map, "goal_position_z", &p_goal_position_z_, 1.0);
    c_yaw_rate_ =
        std::min(p_yaw_angle_ * planner_.getSystemConstraints().v_max / p_distance_ / 2.0,
                 planner_.getSystemConstraints().yaw_rate_max);
    goal_pos_ = {p_goal_position_x_, p_goal_position_y_, p_goal_position_z_};
    home_pos_ = {0.0, 10.0, 1.5};
    goal_reached_ = false;
    //goal_traj_.push_back(goal_pose);
    //goal_traj_.push_back(home_pose);
}

bool Uniform::expandSegment(TrajectorySegment *target,
                            std::vector<TrajectorySegment *> *new_segments)
{
    // Create and add new adjacent trajectories to target segment
    target->tg_visited = true;
    int valid_segments = 0;
    TrajectorySegment *new_segment = target->spawnChild();
    int n_heights = 0;
    if (p_ascent_angle_ != 0.0)
    {
        n_heights = 2;
    }

    double current_ascent = 0.0;
    for (int j = -1; j < n_heights; ++j)
    {
        if (p_ascent_angle_ != 0.0)
        {
            current_ascent = (double)j * p_ascent_angle_;
        }
        for (int i = 0; i < p_n_segments_; ++i)
        {
            // Initialization
            new_segment->trajectory.clear();
            Eigen::Vector3d current_pos = target->trajectory.back().position_W;
            double yaw_rate =
                ((double)i - (double)p_n_segments_ / 2.0 + 0.5) * c_yaw_rate_;
            double current_yaw = 0;
            if (p_goal_oriented_)
            {
                Eigen::Vector2d dist{goal_pos_(0) - current_pos(0), goal_pos_(1) - current_pos(1)};
                double dist_goal = dist.norm(); 

                if(dist_goal <= 1.0 && !goal_reached_){
                    goal_reached_ = true;
                    goal_pos_ = home_pos_;
                    std::cout << "GOAL REACHED , new goal is " << goal_pos_ << "\n";
                }
                Eigen::Vector2d desired_direction{goal_pos_(0) - current_pos(0), goal_pos_(1) - current_pos(1)};
                Eigen::Vector2d zero_yaw{1,0};
                float dot_product = desired_direction.dot(zero_yaw);
                float norm = desired_direction.norm();
                current_yaw = acos(dot_product/norm);
                if (desired_direction(1) < 1e-4){
                    current_yaw = 2*3.1415 - current_yaw;
                }
                // if(goal_reached_){
                //     std::cout << "desired_direction  " << desired_direction << " current_yaw " << current_yaw << "\n";
                // }
            }
            else
            {
                current_yaw = target->trajectory.back().getYaw();
            }
          
            double current_distance = 0.0;
            double current_time = 0.0;
            bool collided = false;

            while (current_distance < p_distance_)
            {
                // Advance trajectory for every timestep
                current_yaw += yaw_rate / p_sampling_rate_;
                current_distance += planner_.getSystemConstraints().v_max / p_sampling_rate_;
                current_time += 1.0 / p_sampling_rate_;
                current_pos += planner_.getSystemConstraints().v_max / p_sampling_rate_ *
                               Eigen::Vector3d(cos(current_yaw) * cos(current_ascent),
                                               sin(current_yaw) * cos(current_ascent),
                                               sin(current_ascent));
                if (!checkTraversable(current_pos))
                {
                    collided = true;
                    break;
                }
                EigenTrajectoryPoint trajectory_point;
                trajectory_point.position_W = current_pos;
                trajectory_point.setFromYaw(defaults::angleScaled(current_yaw));
                trajectory_point.time_from_start_ns =
                    static_cast<int64_t>(current_time * 1.0e9);
                new_segment->trajectory.push_back(trajectory_point);
            }
            if (!collided)
            {
                valid_segments++;
                new_segments->push_back(new_segment);
                new_segment = target->spawnChild();
            }
        }
    }
    target->children.pop_back();

    // Feasible solution found?
    return (valid_segments > 0);
}

// bool Uniform::getGoalTrajectory(std::vector<Eigen::Vector3d> *goal_trajectory){

// }

} // namespace trajectory_generator
} // namespace active_3d_planning