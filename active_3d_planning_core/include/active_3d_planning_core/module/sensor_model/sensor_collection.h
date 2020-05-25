#include "active_3d_planning_core/module/sensor_model/sensor_collection.h"
#include "active_3d_planning_core/planner/planner_I.h"

namespace active_3d_planning
{
    SensorCollection::SensorCollection(PlannerI &planner)
        : Module(planner) {}

    
    void SensorCollection::setupFromParamMap(Module::ParamMap *param_map){

        setParam<int>(param_map, "num_sensors", &p_num_sensors_, 1);
    }





} // namespace active_3d_planning
