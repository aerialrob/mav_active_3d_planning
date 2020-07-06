#ifndef ACTIVE_3D_PLANNING_MAV_TRAJECTORY_GENERATOR_FEASIBLE_INFORMED_RRT_STAR_H
#define ACTIVE_3D_PLANNING_MAV_TRAJECTORY_GENERATOR_FEASIBLE_INFORMED_RRT_STAR_H

#include "active_3d_planning_core/module/trajectory_generator/informed_rrt_star.h"

#include "active_3d_planning_mav/tools/linear_mav_trajectory_generator.h"

namespace active_3d_planning {
    namespace trajectory_generator {

        class FeasibleInformedRRTStar : public InformedRRTStar {
        public:
            FeasibleInformedRRTStar(PlannerI &planner);

            virtual void setupFromParamMap(Module::ParamMap *param_map) override;

        protected:
            static ModuleFactoryRegistry::Registration<FeasibleInformedRRTStar> registration;

            // Segment creator
            LinearMavTrajectoryGenerator segment_generator_;

            // params
            bool p_all_segments_feasible_;

            // Overwrite virtual method to create constrained trajectories
            bool connectPoses(const EigenTrajectoryPoint &start,
                              const EigenTrajectoryPoint &goal,
                              EigenTrajectoryPointVector *result,
                              bool check_collision = true) override;

            bool extractTrajectoryToPublish(EigenTrajectoryPointVector *trajectory,
                                            const TrajectorySegment &segment) override;
        };

    } // namespace trajectory_generator
} // namespace active_3d_planning
#endif // ACTIVE_3D_PLANNING_MAV_TRAJECTORY_GENERATOR_FEASIBLE_INFORMED_RRT_STAR_H
