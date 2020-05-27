#ifndef ACTIVE_3D_PLANNING_CORE_TRAJECTORY_GENERATOR_INFORMED_RRT_STAR_H
#define ACTIVE_3D_PLANNING_CORE_TRAJECTORY_GENERATOR_INFORMED_RRT_STAR_H

#include "active_3d_planning_core/libs/nanoflann.hpp"
#include "active_3d_planning_core/module/trajectory_generator.h"
#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning
{
    namespace trajectory_evaluator
    {
        class InformedRRTStarEvaluatorAdapter;
    }
    namespace trajectory_generator
    {

        class InformedRRTStar : public TrajectoryGenerator
        {

        public:
            struct TreeData
            {
                // data
                std::vector<Eigen::Vector3d> points;
                std::vector<TrajectorySegment *> data;

                // Clear up everything
                void clear();

                // push back a segment
                void addSegment(TrajectorySegment *to_add);

                // nanoflann functionality
                inline std::size_t kdtree_get_point_count() const { return points.size(); }

                inline double kdtree_get_pt(const size_t idx, const size_t dim) const
                {
                    if (dim == 0)
                        return points[idx].x();
                    else if (dim == 1)
                        return points[idx].y();
                    else
                        return points[idx].z();
                }

                template <class BBOX>
                bool kdtree_get_bbox(BBOX & /* bb */) const
                {
                    return false;
                }
            };
            typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
                nanoflann::L2_Simple_Adaptor<double, TreeData>, TreeData, 3>
                KDTree;

            InformedRRTStar(PlannerI &planner);

            void setupFromParamMap(Module::ParamMap *param_map) override;

            virtual bool
            expandSegment(TrajectorySegment *target,
                          std::vector<TrajectorySegment *> *new_segments) override;

            virtual bool selectSegment(TrajectorySegment **result,
                                       TrajectorySegment *root) override;

            std::vector<double> weights_x_;
            std::vector<double> weights_y_;
            std::discrete_distribution<int> distribution_prob_y_;
            std::discrete_distribution<int> distribution_prob_x_;
            bool initializeDistribution();
            bool updateDistribution(Eigen::Vector3d *sampled_point, int dim, bool higher);

        protected:
            static ModuleFactoryRegistry::Registration<InformedRRTStar> registration;

            // parameters
            int p_num_samples_;        // Num. of samples taken by the rrt
            double p_sampling_rate_;   // Hz
            double p_max_sample_dist_; // Maximum distance to connect a neighbour sample
            int p_n_neighbors_;        // Nr of neighbours to rewires
            bool p_update_subsequent_;
            double p_min_sample_dist_;
            int p_maximum_tries_;
            double p_goal_x_;
            double p_goal_y_;
            double p_goal_z_;
            double p_prob_divisions_;


            std::default_random_engine generator;
            Eigen::Vector3d local_volume_min_;
            Eigen::Vector3d local_volume_max_;

            Eigen::Vector3d current_goal_;
            Eigen::Vector3d home_pose_;
            Eigen::Vector3d connecting_sample_;
            // kdtree
            std::unique_ptr<KDTree> kdtree_;
            TreeData tree_data_;
            TrajectorySegment *previous_root_;

            bool getSample(Eigen::Vector3d *goal_pos, Eigen::Vector3d *target_position);
            bool connectPoses(const EigenTrajectoryPoint &start,
                              const EigenTrajectoryPoint &goal,
                              EigenTrajectoryPointVector *result,
                              bool check_collision = true);
            bool resetTree(TrajectorySegment *root);
            bool rewireSample(TrajectorySegment *new_segment, std::vector<Eigen::Vector3d> &candidates, std::vector<TrajectorySegment *> *new_segments);
            bool getCloseNeighbours(const Eigen::Vector3d &target_point, std::vector<Eigen::Vector3d> *close_neighbours);
            bool rewireToBestParent(TrajectorySegment *segment,
                                    const std::vector<TrajectorySegment *> &candidates, bool force_rewire = false);
            bool findIndexFromPoint(Eigen::Vector3d point, std::vector<TrajectorySegment *> &new_segments, TrajectorySegment *segment);
        };
    } // namespace trajectory_generator

    namespace trajectory_evaluator
    {

        // RRTStar Evaluator adapter, its following evaluator is used for all regular
        // evaluation duties.
        class InformedRRTStarEvaluatorAdapter : public TrajectoryEvaluator
        {
        public:
            explicit InformedRRTStarEvaluatorAdapter(PlannerI &planner);

            // Override virtual functions
            bool computeGain(TrajectorySegment *traj_in) override;

            bool computeCost(TrajectorySegment *traj_in) override;

            bool computeValue(TrajectorySegment *traj_in) override;

            //int selectNextBest(TrajectorySegment *traj_in) override;

            bool updateSegment(TrajectorySegment *segment) override;

            void visualizeTrajectoryValue(VisualizationMarkers *markers,
                                          const TrajectorySegment &trajectory) override;

            void setupFromParamMap(Module::ParamMap *param_map) override;

        protected:
            static ModuleFactoryRegistry::Registration<InformedRRTStarEvaluatorAdapter>
                registration;

            // members
            std::unique_ptr<TrajectoryEvaluator> following_evaluator_;
            trajectory_generator::InformedRRTStar *generator_;
        };

    } // namespace trajectory_evaluator
} // namespace active_3d_planning
#endif // ACTIVE_3D_PLANNING_CORE_TRAJECTORY_GENERATOR_INFORMED_RRT_STAR_H
