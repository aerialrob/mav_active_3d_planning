#ifndef ACTIVE_3D_PLANNING_CORE_MAP_TSDF_MAP_H
#define ACTIVE_3D_PLANNING_CORE_MAP_TSDF_MAP_H

#include "active_3d_planning_core/map/occupancy_map.h"
#include "active_3d_planning_core/data/trajectory.h"

namespace active_3d_planning {
    namespace map {

        // base interface for Truncated Signed Distance Field voxelgrid maps
        class TSDFMap : public OccupancyMap {
        public:
            TSDFMap(PlannerI &planner) : OccupancyMap(planner) {}

            virtual ~TSDFMap() = default;

            // get the stored distance
            virtual double getVoxelDistance(const Eigen::Vector3d &point) = 0;

            // get the stored weight
            virtual double getVoxelWeight(const Eigen::Vector3d &point) = 0;

            // get the maximum allowed weight (return 0 if using uncapped weights)
            virtual double getMaximumWeight() = 0;

            // mark the voxel corresponding to this point as sensed in the esdf map
            virtual bool markAsSensed(const Eigen::Vector3d &point) = 0;

            // check whether point is sensed
            virtual bool isSensed(const Eigen::Vector3d &point) = 0;
            // get the percentage of the voxblox mesh which have been sensed
            virtual double getPercentageSensed() = 0;


        };

    } // namespace map
} // namespace active_3d_planning

#endif // ACTIVE_3D_PLANNING_CORE_MAP_TSDF_MAP_H