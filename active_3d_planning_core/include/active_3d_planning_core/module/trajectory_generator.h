#ifndef ACTIVE_3D_PLANNING_CORE_TRAJECTORY_GENERATOR_H_
#define ACTIVE_3D_PLANNING_CORE_TRAJECTORY_GENERATOR_H_

#include "active_3d_planning_core/data/trajectory_segment.h"
#include "active_3d_planning_core/data/bounding_volume.h"
#include "active_3d_planning_core/module/module_factory_registry.h"
#include "active_3d_planning_core/tools/defaults.h"
#include <active_3d_planning_core/planner/planner_I.h>

#include <Eigen/Core>

#include <string>
#include <vector>

namespace active_3d_planning
{

    // Forward declaration
    class SegmentSelector;

    class GeneratorUpdater;

    // Base class for trajectory generation to provide uniform interface with other
    // classes
    class TrajectoryGenerator : public Module
    {
    public:
        explicit TrajectoryGenerator(PlannerI &planner);

        virtual ~TrajectoryGenerator() = default;

        bool new_global_goal_;
        // Expansion policy where to expand (from full tree)
        virtual bool selectSegment(TrajectorySegment **result,
                                   TrajectorySegment *root);

        // Expand a selected trajectory segment. Return true for successful expansion.
        virtual bool
        expandSegment(TrajectorySegment *target,
                      std::vector<TrajectorySegment *> *new_segments) = 0;

        // Update an existing segment when a new trajectory is executed, return true if the segment is to be kept alive, false if it should be removed from the tree
        virtual bool updateSegment(TrajectorySegment *segment);

        // Utility function for collision checking. Returns true if the position is
        // reachable.
        bool checkTraversable(const Eigen::Vector3d &position);

        // in case a trajectory needs to be modified to be published
        virtual bool
        extractTrajectoryToPublish(EigenTrajectoryPointVector *trajectory,
                                   const TrajectorySegment &segment);

        bool setObservedBoundingBox(std::vector<Eigen::Vector2d> *bounding_box)
        {
            observed_bounding_volume_[0][0] = (*bounding_box)[0][0];
            observed_bounding_volume_[0][1] = (*bounding_box)[0][1];
            observed_bounding_volume_[1][0] = (*bounding_box)[1][0];
            observed_bounding_volume_[1][1] = (*bounding_box)[1][1];
            return true;
        }

        bool setGlobalGoal(Eigen::Vector3d *goal)
        {
            global_goal_[0] = (*goal)[0];
            global_goal_[1] = (*goal)[1];
            global_goal_[2] = (*goal)[2];
            
            if(!((last_global_goal_ - global_goal_).norm() == 0)){

                last_global_goal_ = global_goal_;
                new_global_goal_ = true;
            }

            return true;
        }

        virtual void setupFromParamMap(Module::ParamMap *param_map) override;

    protected:
        // bounding box
        std::unique_ptr<BoundingVolume> bounding_volume_;
        // Observed bounding box
        std::vector<Eigen::Vector2d> observed_bounding_volume_;

        // Global goal
        Eigen::Vector3d global_goal_;
        Eigen::Vector3d last_global_goal_;
        double goal_distance_;

        // default modules
        std::unique_ptr<SegmentSelector> segment_selector_;
        std::unique_ptr<GeneratorUpdater> generator_updater_;

        // Parameters
        bool p_collision_optimistic_;
        double p_clearing_radius_; // Unknown space within clearing radius is
        // considered traversable
        std::string p_selector_args_;
        std::string p_updater_args_;
    };

    // Abstract encapsulation for default/modular implementations of the
    // selectSegment method
    class SegmentSelector : public Module
    {
    public:
        explicit SegmentSelector(PlannerI &planner);

        virtual bool selectSegment(TrajectorySegment **result,
                                   TrajectorySegment *root) = 0;
    };

    // Abstract encapsulation for default/modular implementations of the
    // updateSegments method
    class GeneratorUpdater : public Module
    {
    public:
        explicit GeneratorUpdater(PlannerI &planner);

        virtual bool updateSegment(TrajectorySegment *segment) = 0;
    };

} // namespace active_3d_planning
#endif // ACTIVE_3D_PLANNING_CORE_TRAJECTORY_GENERATOR_H_
