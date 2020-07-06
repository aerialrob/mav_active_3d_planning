#include "active_3d_planning_core/module/sensor_model/camera_model.h"

#include "active_3d_planning_core/tools/defaults.h"

#include <algorithm>
#include <chrono>
#include <vector>
#include <glog/logging.h>

namespace active_3d_planning
{
    namespace sensor_model
    {

        CameraModel::CameraModel(PlannerI &planner) : SensorModel(planner) {}

        bool CameraModel::getVisibleVoxelsFromTrajectory(
            std::vector<Eigen::Vector3d> *result, const TrajectorySegment &traj_in, std::vector<Eigen::Vector2d> *observed_bounding_box )
        {
            // Sample camera poses through trajectory
            std::vector<int> indices;
            sampleViewpoints(&indices, traj_in);

            // Get all visible voxels
            for (int i = 0; i < indices.size(); ++i)
            {
                // Get camera pose / apply mounting transform
                for (int u = 0; u < p_num_sensors_; ++u)
                {
                    Eigen::Vector3d position = traj_in.trajectory[indices[i]].position_W;
                    Eigen::Quaterniond orientation =
                        traj_in.trajectory[indices[i]].orientation_W_B;
                    position = position + orientation * mounting_translation_list_[u];
                    orientation = orientation * mounting_rotation_list_[u];

                    // Get visible voxels
                    if (!p_test_)
                    {
                        getVisibleVoxels(result, position, orientation, observed_bounding_box, u);
                    }
                    else
                    {
                        // Time the performance of raycasters
                        auto start_time = std::chrono::high_resolution_clock::now();
                        getVisibleVoxels(result, position, orientation, observed_bounding_box, u);
                        auto end_time = std::chrono::high_resolution_clock::now();
                        time_count_.push_back(
                            (double)((end_time - start_time) / std::chrono::milliseconds(1)));
                        if (time_count_.size() % 20 == 0)
                        {
                            double mean = 0.0;
                            double stddev = 0.0;
                            for (int i = 0; i < time_count_.size(); ++i)
                            {
                                mean += time_count_[i];
                            }
                            mean /= time_count_.size();
                            for (int i = 0; i < time_count_.size(); ++i)
                            {
                                stddev += std::pow(time_count_[i] - mean, 2.0);
                            }
                            stddev = std::sqrt(stddev / time_count_.size());
                            LOG(INFO) << "Raycasting: Mean: " << mean << " ms, Stddev: " << stddev << " ms";
                        }
                    }
                }
            }

            // Remove non-unique voxels
            result->erase(std::unique(result->begin(), result->end()), result->end());
            return true;
        }

        void CameraModel::sampleViewpoints(std::vector<int> *result,
                                           const TrajectorySegment &traj_in)
        {
            if (p_sampling_time_ <= 0.0)
            {
                // Rate of 0 means only last point
                result->push_back(traj_in.trajectory.size() - 1);
            }
            else
            {
                int64_t sampling_time_ns = static_cast<int64_t>(p_sampling_time_ * 1.0e9);
                if (sampling_time_ns >= traj_in.trajectory.back().time_from_start_ns)
                {
                    // no points within one sampling interval: add last point
                    result->push_back(traj_in.trajectory.size() - 1);
                }
                else
                {
                    // sample the trajectory according to the sampling rate
                    int64_t current_time = sampling_time_ns;
                    for (int i = 0; i < traj_in.trajectory.size(); ++i)
                    {
                        if (traj_in.trajectory[i].time_from_start_ns >= current_time)
                        {
                            current_time += sampling_time_ns;
                            result->push_back(i);
                        }
                    }
                }
            }
        }

        void CameraModel::visualizeSensorView(VisualizationMarkers *markers,
                                              const TrajectorySegment &traj_in)
        {
            std::vector<int> indices;
            sampleViewpoints(&indices, traj_in);

            for (int i = 0; i < indices.size(); ++i)
            {
                Eigen::Vector3d position;
                Eigen::Quaterniond orientation;

                for (int u = 0; u < p_num_sensors_; u++)
                {
                    position = traj_in.trajectory[indices[i]].position_W;
                    orientation = traj_in.trajectory[indices[i]].orientation_W_B;
                    position = position + orientation * mounting_translation_list_[u];
                    orientation = orientation * mounting_rotation_list_[u];
                    visualizeSingleView(markers, position, orientation, u);
                }
            }
        }

        void CameraModel::visualizeSingleView(VisualizationMarkers *markers,
                                              const Eigen::Vector3d &position,
                                              const Eigen::Quaterniond &orientation,
                                              int id_sensor)
        {
            VisualizationMarker marker;
            marker.type = VisualizationMarker::LINE_LIST;
            marker.scale.x() = 0.02;
            marker.color.r = 1.0 - 0.5 * id_sensor;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;

            // Get boundary lines
            int res_x = ceil(p_ray_length_ * c_field_of_view_x_ / 0.25);
            int res_y = ceil(p_ray_length_ * c_field_of_view_y_ / 0.25);

            Eigen::Vector3d left[res_y + 1];
            Eigen::Vector3d right[res_y + 1];
            Eigen::Vector3d top[res_x + 1];
            Eigen::Vector3d bottom[res_x + 1];

            Eigen::Vector3d direction;

            for (int i = 0; i <= res_y; ++i)
            {
                getDirectionVector(&direction, 0.0, (double)i / (double)res_y);
                left[i] = position + p_ray_length_ * (orientation * direction);
                getDirectionVector(&direction, 1.0, (double)i / (double)res_y);
                right[i] = position + p_ray_length_ * (orientation * direction);
            }
            for (int i = 0; i <= res_x; ++i)
            {
                getDirectionVector(&direction, (double)i / (double)res_x, 0.0);
                bottom[i] = position + p_ray_length_ * (orientation * direction);
                getDirectionVector(&direction, (double)i / (double)res_x, 1.0);
                top[i] = position + p_ray_length_ * (orientation * direction);
            }

            marker.points.push_back(position);
            marker.points.push_back(left[0]);
            marker.points.push_back(position);
            marker.points.push_back(left[res_y]);
            marker.points.push_back(position);
            marker.points.push_back(right[0]);
            marker.points.push_back(position);
            marker.points.push_back(right[res_y]);

            for (int i = 1; i <= res_x; ++i)
            {
                marker.points.push_back(top[i - 1]);
                marker.points.push_back(top[i]);
                marker.points.push_back(bottom[i - 1]);
                marker.points.push_back(bottom[i]);
            }
            for (int i = 1; i <= res_y; ++i)
            {
                marker.points.push_back(left[i - 1]);
                marker.points.push_back(left[i]);
                marker.points.push_back(right[i - 1]);
                marker.points.push_back(right[i]);
            }
            markers->addMarker(marker);
        }

        void CameraModel::getDirectionVector(Eigen::Vector3d *result, double relative_x,
                                             double relative_y)
        {
            *result = Eigen::Vector3d(p_focal_length_,
                                      (0.5 - relative_x) * (double)p_resolution_x_,
                                      (0.5 - relative_y) * (double)p_resolution_y_)
                          .normalized();
        }

        void CameraModel::setupFromParamMap(Module::ParamMap *param_map)
        {
            SensorModel::setupFromParamMap(param_map);
            setParam<double>(param_map, "ray_length", &p_ray_length_, 5.0);
            setParam<double>(param_map, "sampling_time", &p_sampling_time_, 0.0);
            setParam<bool>(param_map, "test", &p_test_,
                           false); // set to measure performance of ray casters

            setParam<int>(param_map, "num_sensors", &p_num_sensors_, 1);

            for (int i = 0; i < p_num_sensors_; i++)
            {
                setParam<double>(param_map, "focal_length_" + std::to_string(i), &p_focal_length_, 320.0);
                p_focal_length_list_.push_back(p_focal_length_);

                setParam<int>(param_map, "resolution_x_" + std::to_string(i), &p_resolution_x_, 640);
                p_resolution_x_list_.push_back(p_resolution_x_);
                c_field_of_view_x_ = 2.0 * atan2(p_resolution_x_, p_focal_length_ * 2.0);
                c_field_of_view_x_list.push_back(c_field_of_view_x_);

                setParam<int>(param_map, "resolution_y_" + std::to_string(i), &p_resolution_y_, 480);
                p_resolution_y_list_.push_back(p_resolution_y_);
                c_field_of_view_y_ = 2.0 * atan2(p_resolution_y_, p_focal_length_ * 2.0);
                c_field_of_view_y_list.push_back(c_field_of_view_y_);
            }
        }

        bool CameraModel::checkParamsValid(std::string *error_message)
        {
            if (p_ray_length_ <= 0.0)
            {
                *error_message = "ray_length expected > 0.0";
                return false;
            }
            else if (p_focal_length_ <= 0.0)
            {
                *error_message = "focal_length expected > 0.0";
                return false;
            }
            else if (p_resolution_x_ <= 0)
            {
                *error_message = "resolution_x expected > 0";
                return false;
            }
            else if (p_resolution_y_ <= 0.0)
            {
                *error_message = "resolution_y expected > 0";
                return false;
            }
            return true;
        }

    } // namespace sensor_model
} // namespace active_3d_planning