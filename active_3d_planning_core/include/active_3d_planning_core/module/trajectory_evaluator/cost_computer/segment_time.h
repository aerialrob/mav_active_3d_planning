#ifndef ACTIVE_3D_PLANNING_COST_COMPUTERS_SEGMENT_TIME_H
#define ACTIVE_3D_PLANNING_COST_COMPUTERS_SEGMENT_TIME_H

#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
    namespace cost_computer {

        // Execution time of a single segment
        class SegmentTime : public CostComputer {
        public:
            explicit SegmentTime(PlannerI &planner);

            bool computeCost(TrajectorySegment *traj_in) override;

            void setupFromParamMap(Module::ParamMap *param_map) override;

        protected:
            static ModuleFactoryRegistry::Registration<SegmentTime> registration;

            // params
            bool p_accumulate_; // True: Use total time
            bool p_altitude_change_; // True: penalize the altitude change
            bool p_yaw_change_; // True: penalize the yaw change
        };

    } // namespace cost_computer
} // namespace active_3d_planning

#endif // ACTIVE_3D_PLANNING_COST_COMPUTERS_SEGMENT_TIME_H
