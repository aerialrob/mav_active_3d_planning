#ifndef ACTIVE_3D_PLANNING_COST_COMPUTERS_SEGMENT_TIME_H
#define ACTIVE_3D_PLANNING_COST_COMPUTERS_SEGMENT_TIME_H

#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
    namespace cost_computer {

        // Execution time of a single segment
        class SegmentTime : public CostComputer {
        public:
            explicit SegmentTime(PlannerI &planner);

            bool computeCost(TrajectorySegment *traj_in, Eigen::Vector3d *current_position ) override;

            bool setGlobalGoal(Eigen::Vector3d *goal) override;

            void setupFromParamMap(Module::ParamMap *param_map) override;

        protected:
            static ModuleFactoryRegistry::Registration<SegmentTime> registration;

            // params
            bool p_accumulate_; // True: Use total time
            bool p_altitude_change_; // True: penalize the altitude change
            bool p_yaw_change_; // True: penalize the yaw change
            bool p_distance_to_goal_; //True penalize distance to goal
            double p_time_weight_; 
            double start_time_;
            double p_time_limit_; // simulation limit time
            double p_exp_mult_; // term which multiplies the decaying exp function
            double p_exp_div_;  // term which divedes the remaining time inside the exp function

            // var
            Eigen::Vector3d global_goal_;
            Eigen::Vector3d last_global_goal_;
            bool new_global_goal_;
        };

    } // namespace cost_computer
} // namespace active_3d_planning

#endif // ACTIVE_3D_PLANNING_COST_COMPUTERS_SEGMENT_TIME_H
