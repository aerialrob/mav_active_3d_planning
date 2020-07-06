#include "active_3d_planning_core/module/sensor_model/iterative_ray_caster.h"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <vector>

namespace active_3d_planning
{
    namespace sensor_model
    {

        ModuleFactoryRegistry::Registration<IterativeRayCaster>
            IterativeRayCaster::registration("IterativeRayCaster");

        IterativeRayCaster::IterativeRayCaster(PlannerI &planner)
            : CameraModel(planner) {}

        void IterativeRayCaster::setupFromParamMap(Module::ParamMap *param_map)
        {
            CameraModel::setupFromParamMap(param_map);
            setParam<double>(param_map, "ray_step", &p_ray_step_, map_->getVoxelSize());
            setParam<double>(param_map, "downsampling_factor", &p_downsampling_factor_,
                             1.0);

            for (int i = 0; i < p_num_sensors_; i++)
            {
                // Downsample to voxel size resolution at max range
                c_res_x_ =
                    std::min((int)ceil(p_ray_length_ * c_field_of_view_x_list[i] /
                                       (map_->getVoxelSize() * p_downsampling_factor_)),
                             p_resolution_x_list_[i]);
                c_res_y_ =
                    std::min((int)ceil(p_ray_length_ * c_field_of_view_y_list[i] /
                                       (map_->getVoxelSize() * p_downsampling_factor_)),
                             p_resolution_y_list_[i]);

                c_res_x_list_.push_back(c_res_x_);
                c_res_y_list_.push_back(c_res_y_);

                // Determine number of splits + split distances
                c_n_sections_ =
                    (int)std::floor(std::log2(std::min((double)c_res_x_, (double)c_res_y_)));
                c_split_widths_.push_back(0);
                for (int i = 0; i < c_n_sections_; ++i)
                {
                    c_split_widths_.push_back(std::pow(2, i));
                    c_split_distances_.push_back(p_ray_length_ / std::pow(2.0, (double)i));
                }
                c_split_distances_.push_back(0.0);
                std::reverse(c_split_distances_.begin(), c_split_distances_.end());
                std::reverse(c_split_widths_.begin(), c_split_widths_.end());

                c_n_sections_list_.push_back(c_n_sections_);
                c_split_widths_list_.push_back(c_split_widths_);
                c_split_distances_list_.push_back(c_split_distances_);
            }
        }

        bool IterativeRayCaster::getVisibleVoxels(
            std::vector<Eigen::Vector3d> *result, const Eigen::Vector3d &position,
            const Eigen::Quaterniond &orientation, std::vector<Eigen::Vector2d> *observed_bounding_box, int sensor_id)
        {
            // Setup ray table (contains at which segment to start, -1 if occluded
            c_res_x_ = c_res_x_list_[sensor_id];
            c_res_y_ = c_res_y_list_[sensor_id];
            c_split_distances_ = c_split_distances_list_[sensor_id];
            c_n_sections_ = c_n_sections_list_[sensor_id];
            c_split_widths_ = c_split_widths_list_[sensor_id];

            ray_table_ = Eigen::ArrayXXi::Zero(c_res_x_, c_res_y_);

            // Ray-casting
            Eigen::Vector3d camera_direction;
            Eigen::Vector3d direction;
            Eigen::Vector3d current_position;
            Eigen::Vector3d voxel_center;
            double distance;
            bool cast_ray;
            double map_distance;
            double x_min, x_max;
            for (int i = 0; i < c_res_x_; ++i)
            {
                for (int j = 0; j < c_res_y_; ++j)
                {
                    int current_segment = ray_table_(i, j); // get ray starting segment
                    if (current_segment < 0)
                    {
                        continue; // already occluded ray
                    }
                    CameraModel::getDirectionVector(&camera_direction,
                                                    (double)i / ((double)c_res_x_ - 1.0),
                                                    (double)j / ((double)c_res_y_ - 1.0));
                    direction = orientation * camera_direction;
                    distance = c_split_distances_[current_segment];
                    cast_ray = true;
                    while (cast_ray)
                    {
                        // iterate through all splits (segments)
                        while (distance < c_split_distances_[current_segment + 1])
                        {
                            current_position = position + distance * direction;
                            distance += p_ray_step_;

                            // Check voxel occupied
                            if (map_->getVoxelState(current_position) == map::OccupancyMap::OCCUPIED)
                            {
                                // Occlusion, mark neighboring rays as occluded
                                markNeighboringRays(i, j, current_segment, -1);
                                cast_ray = false;
                                map_->getVoxelCenter(&voxel_center, current_position);
                                result->push_back(voxel_center);

                                // Find X coordinate min and max from surface observed voxels 
                                if (current_position[0] < (*observed_bounding_box)[0][0])
                                {
                                    (*observed_bounding_box)[0][0] = current_position[0];
                                }
                                else if (current_position[0] > (*observed_bounding_box)[0][1])
                                {
                                    (*observed_bounding_box)[0][1] = current_position[0];
                                }
                                
                                // Find Y coordinate min and max from surface observed voxels 
                                if (current_position[1] < (*observed_bounding_box)[1][0])
                                {
                                    (*observed_bounding_box)[1][0] = current_position[1];
                                }
                                else if (current_position[1] > (*observed_bounding_box)[1][1])
                                {
                                    (*observed_bounding_box)[1][1] = current_position[1];
                                }

                                break;
                            }

                            // Add point (duplicates are handled in
                            // CameraModel::getVisibleVoxelsFromTrajectory)
                        }
                        if (cast_ray)
                        {
                            current_segment++;
                            if (current_segment >= c_n_sections_)
                            {
                                cast_ray = false; // done
                            }
                            else
                            {
                                // update ray starts of neighboring rays
                                markNeighboringRays(i, j, current_segment - 1, current_segment);
                            }
                        }
                    }
                }
            }

            //std::cout << "Found z bounds " << (*observed_bounding_box)[1][0] << " "<<  (*observed_bounding_box)[1][1] << "\n";
            return true;
        }

        void IterativeRayCaster::markNeighboringRays(int x, int y, int segment,
                                                     int value)
        {
            // Set all nearby (towards bottom right) ray starts, depending on the segment
            // depth, to a value.
            for (int i = x; i < std::min(c_res_x_, x + c_split_widths_[segment]); ++i)
            {
                for (int j = y; j < std::min(c_res_y_, y + c_split_widths_[segment]); ++j)
                {
                    ray_table_(i, j) = value;
                }
            }
        }

    } // namespace sensor_model
} // namespace active_3d_planning