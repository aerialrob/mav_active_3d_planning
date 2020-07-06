#ifndef ACTIVE_3D_PLANNING_SENSOR_MODEL_SENSOR_MODEL_H
#define ACTIVE_3D_PLANNING_SENSOR_MODEL_SENSOR_MODEL_H

#include "active_3d_planning_core/module/module_factory.h"
#include "active_3d_planning_core/data/visualization_markers.h"
#include "active_3d_planning_core/data/trajectory_segment.h"
#include "active_3d_planning_core/map/occupancy_map.h"

#include <Eigen/Core>

#include <memory>
#include <string>

namespace active_3d_planning {

    // Base class that interfaces with sensor based evaluators.
    class SensorModel : public Module {
    public:
        SensorModel(PlannerI &planner);

        virtual ~SensorModel() = default;

        // Return the voxel centers of all visible voxels
        virtual bool
        getVisibleVoxelsFromTrajectory(std::vector<Eigen::Vector3d> *result,
                                       const TrajectorySegment &traj_in, std::vector<Eigen::Vector2d> *observed_bounding_box) = 0;

        // Implement this function to allow visualization of the sensing bounds
        virtual void visualizeSensorView(VisualizationMarkers *markers,
                                         const TrajectorySegment &trajectory) = 0;

        virtual void setupFromParamMap(Module::ParamMap *param_map) override;

    protected:
        // sensor model is configured to use voxel maps
        map::OccupancyMap *map_;
        
        // mounting transform from body (pose) to sensor , in body frame
        Eigen::Vector3d mounting_translation_; // x,y,z [m]
        Eigen::Quaterniond mounting_rotation_; // x,y,z,w quaternion
        std::vector<Eigen::Vector3d>  mounting_translation_list_;
        std::vector<Eigen::Quaterniond>  mounting_rotation_list_;
        int p_num_sensors_;
    };

} // namespace active_3d_planning
#endif // ACTIVE_3D_PLANNING_SENSOR_MODEL_SENSOR_MODEL_H
