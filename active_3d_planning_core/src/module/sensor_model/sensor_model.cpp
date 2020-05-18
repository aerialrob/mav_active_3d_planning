#include "active_3d_planning_core/module/sensor_model/sensor_model.h"
#include "active_3d_planning_core/planner/planner_I.h"

namespace active_3d_planning
{

    SensorModel::SensorModel(PlannerI &planner)
        : Module(planner) {}

    void SensorModel::setupFromParamMap(Module::ParamMap *param_map)
    {
        // setup map
        map_ = dynamic_cast<map::OccupancyMap *>(&(planner_.getMap()));
        if (!map_)
        {
            planner_.printError("'SensorModel' requires a map of type 'OccupancyMap'!");
        }

        setParam<int>(param_map, "num_sensors", &p_num_sensors_, 1);

        for (int i = 0; i < p_num_sensors_; i++)
        {
            double tx, ty, tz, rx, ry, rz, rw;
            setParam<double>(param_map, "mounting_translation_x_" + std::to_string(i), &tx, 0.0);
            setParam<double>(param_map, "mounting_translation_y_" + std::to_string(i), &ty, 0.0);
            setParam<double>(param_map, "mounting_translation_z_" + std::to_string(i), &tz, 0.0);
            setParam<double>(param_map, "mounting_rotation_x_" + std::to_string(i), &rx, 0.0);
            setParam<double>(param_map, "mounting_rotation_y_" + std::to_string(i), &ry, 0.0);
            setParam<double>(param_map, "mounting_rotation_z_" + std::to_string(i), &rz, 0.0);
            setParam<double>(param_map, "mounting_rotation_w_" + std::to_string(i), &rw, 1.0);
            mounting_translation_ = Eigen::Vector3d(tx, ty, tz);
            mounting_rotation_ = Eigen::Quaterniond(rw, rx, ry, rz).normalized();
            mounting_translation_list_.push_back(mounting_translation_);
            mounting_rotation_list_.push_back(mounting_rotation_);
        }
    }

} // namespace active_3d_planning