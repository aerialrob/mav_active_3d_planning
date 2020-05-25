#define _USE_MATH_DEFINES

#include "active_3d_planning_core/module/trajectory_generator/informed_rrt_star.h"
#include "active_3d_planning_core/data/system_constraints.h"

#include "active_3d_planning_core/data/trajectory.h"
#include "active_3d_planning_core/tools/defaults.h"

#include "active_3d_planning_core/module/module_factory.h"
#include "active_3d_planning_core/planner/planner_I.h"

#include <cmath>
#include <memory>
#include <random>
#include <vector>

namespace active_3d_planning
{
    namespace trajectory_generator
    {
        ModuleFactoryRegistry::Registration<InformedRRTStar> InformedRRTStar::registration("InformedRRTStar");

        InformedRRTStar::InformedRRTStar(PlannerI &planner) : TrajectoryGenerator(planner) {}

        void InformedRRTStar::setupFromParamMap(Module::ParamMap *param_map)
        {
            setParam<int>(param_map, "num_samples", &p_num_samples_, 100);
            setParam<double>(param_map, "sampling_rate", &p_sampling_rate_, 10);
            setParam<double>(param_map, "max_sample_dist", &p_max_sample_dist_, 2);
            setParam<double>(param_map, "min_sample_dist", &p_min_sample_dist_, 0.5);
            setParam<int>(param_map, "n_neighbors", &p_n_neighbors_, 5);
            setParam<bool>(param_map, "update_subsequent", &p_update_subsequent_, false);

            // setup parent
            TrajectoryGenerator::setupFromParamMap(param_map);
            //planner_.getFactory().registerLinkableModule("InformedRRTStarGenerator", this);
        }

        bool InformedRRTStar::selectSegment(TrajectorySegment **result,
                                            TrajectorySegment *root)
        {

            if (tree_data_.kdtree_get_point_count() <= 3)
            {
                std::cout << "reset tree!\n";
                resetTree(root);
            }

            Eigen::Vector3d global_goal{0, 50, 1.5};
            double min_dist = 1000;

            // Get all candidates
            std::vector<TrajectorySegment *> candidates;
            int maxdepth_ = 1;
            root->getTree(&candidates, maxdepth_ - 1);
            if (candidates.empty())
            {
                // exception catching
                *result = root;
                return false;
            }

            // uniform selection
            //*result = candidates[rand() % candidates.size()];

            // selection based on distance to goal - TODO: remove it because with the current method this is obsolete you do not extend a certain point
            int closer_candidate_index;
            for (int i = 0; i < candidates.size(); i++)
            {
                TrajectorySegment *candidate = candidates[i];
                Eigen::Vector3d candidate_pose = candidate->trajectory.back().position_W;
                double dist = global_goal(1) - candidate_pose(1);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    closer_candidate_index = i;
                }
            }
            //std::cout << "[select segment] " << candidates[closer_candidate_index]->trajectory.back().position_W << "\n";
            *result = candidates[closer_candidate_index];
            return true;
        }

        bool InformedRRTStar::expandSegment(TrajectorySegment *target,
                                            std::vector<TrajectorySegment *> *new_segments)
        {
            if (!target)
            {
                //std::cout << "No target!\n";
                // Segment selection failed
                return false;
            }
            //std::cout << "[Expand segment] " << target->trajectory.back().position_W << "\n";
            //std::cout << "Tree size " << tree_data_.kdtree_get_point_count() << "\n";

            bool sample_connected = false;
            for (int s = 0; s < p_num_samples_; s++)
            {
                Eigen::Vector3d sample;
                Eigen::Vector3d target_position = target->trajectory.back().position_W;
                getSample(&sample, &target_position);
                //std::cout << "got sample!\n";
                TrajectorySegment *new_segment;

                if (checkTraversable(sample))
                {
                    //std::cout << "checkTraversable!\n";
                    // find closest point in kdtree
                    double query_pt[3] = {sample.x(), sample.y(), sample.z()};
                    //std::cout << "First query point" << query_pt[0] << " " << query_pt[1] << " " << query_pt[2] << "\n";
                    double out_dist_sqr;
                    std::size_t ret_index;
                    nanoflann::KNNResultSet<double> resultSet(1);
                    resultSet.init(&ret_index, &out_dist_sqr);
                    if (!kdtree_->findNeighbors(resultSet, query_pt,
                                                nanoflann::SearchParams(10)))
                    {
                        std::cout << "No neighbour!\n";
                        continue;
                    }
                    if (out_dist_sqr <= p_max_sample_dist_ * p_max_sample_dist_ && out_dist_sqr >= p_min_sample_dist_ * p_min_sample_dist_)
                    {
                        //std::cout << "[EXPAND] Found closer neighbour " << tree_data_.kdtree_get_pt(ret_index, 0) << " " << tree_data_.kdtree_get_pt(ret_index, 1) << " " << tree_data_.kdtree_get_pt(ret_index, 2) << "\n";
                        //std::cout << "Close by ! " << ret_index << " data points " << tree_data_.data.size() << " " << tree_data_.data[ret_index]->trajectory.size() << " " << tree_data_.data[ret_index]->trajectory.size() << "\n";
                        EigenTrajectoryPointVector trajectory;
                        EigenTrajectoryPoint start_point = target->trajectory.back(); 
                        //Eigen::Vector3d close_point{tree_data_.kdtree_get_pt(ret_index, 0), tree_data_.kdtree_get_pt(ret_index, 1), tree_data_.kdtree_get_pt(ret_index, 2)};
                        //start_point.position_W = close_point; //tree_data_.data[ret_index]->trajectory.back(); //target->trajectory.back(); //
                        EigenTrajectoryPoint goal_point;
                        goal_point.position_W = sample;
                        goal_point.setFromYaw(M_PI/2);
                        if (!connectPoses(start_point, goal_point, &trajectory))
                        {
                            std::cout << "[EXPAND] Could not connect!\n";
                            continue;
                        }
                        else
                        {
                            //std::cout << "[EXPAND] Got sample!\n";

                            // Build result and add it to the kdtree
                            new_segment = target->spawnChild(); //new TrajectorySegment();//tree_data_.data[ret_index]->spawnChild(); ////
                            new_segment->trajectory = trajectory;
                          
                            std::cout << "Traj point" << trajectory.back().position_W << "\n";

                            new_segments->push_back(new_segment);
                            tree_data_.addSegment(new_segment);
                            kdtree_->addPoints(tree_data_.points.size() - 1,
                                               tree_data_.points.size() - 1);
                            sample_connected = true;
                            //std::cout << "[EXPAND] Added sample!\n";
                        }
                    }
                }
                //std::cout << "[EXPAND] Num tree data points " << tree_data_.kdtree_get_point_count() << "\n";
                if (tree_data_.data.size() > p_n_neighbors_ * 2 && sample_connected)
                {
                    //rewireSample(new_segment, new_segments);
                }
            }
            std::cout << "Finished exapnsion result " << sample_connected << "\n";
            if (sample_connected)
            {
                return true;
            }
            else           
            {
                return false;
            }
        }

        bool InformedRRTStar::getCloseNeighbours(TrajectorySegment *target, std::vector<TrajectorySegment *> *close_neighbours)
        {
            // Also tried radius search here but that stuff blows for some reason...
            double query_pt[3] = {target->trajectory.back().position_W.x(), target->trajectory.back().position_W.y(), target->trajectory.back().position_W.z()};
            std::size_t ret_index[p_n_neighbors_];
            double out_dist_sqrt[p_n_neighbors_];
            nanoflann::KNNResultSet<double> resultSet(p_n_neighbors_);
            resultSet.init(ret_index, out_dist_sqrt);
            bool found_neighbours = kdtree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));
            std::cout << "Found " << found_neighbours << "\n";

            // double radius = 2.0;
            // std::vector<std::pair<int, double>> IndicesDists;
            // std::size_t radius_search = kdtree_->radiusSearch(query_pt,radius, IndicesDists,nanoflann::SearchParams(10));
            // std::cout << " radius found " << radius_search << "\n";

            if (found_neighbours)
            {
                bool candidate_found = false;
                for (int i = 0; i < resultSet.size(); ++i)
                {
                    if (out_dist_sqrt[i] <= p_max_sample_dist_ * p_max_sample_dist_ && out_dist_sqrt[i] > 0.25)
                    {
                        candidate_found = true;
                        std::cout << "candidates " << tree_data_.data[ret_index[i]]->trajectory.back().position_W.x() << " " << tree_data_.data[ret_index[i]]->trajectory.back().position_W.y() << " " << tree_data_.data[ret_index[i]]->trajectory.back().position_W.z() << "\n";
                        close_neighbours->push_back(tree_data_.data[ret_index[i]]);
                    }
                }
                return candidate_found;
            }
        }

        bool InformedRRTStar::rewireSample(TrajectorySegment *sample, std::vector<TrajectorySegment *> *new_segments)
        {

            std::cout << "Rewireing"
                      << "\n";
            std::vector<TrajectorySegment *> close_neighbours;
            std::vector<TrajectorySegment *> safe_parents;
            if (!getCloseNeighbours(sample, &close_neighbours))
            {
                return false;
            }

            TrajectorySegment *current = sample;
            // while (current)
            // {
            //     // the connection of the new segment to the root cannot be rewired
            //     // (loops!)
            //     close_neighbours.erase(std::remove(close_neighbours.begin(),
            //                                        close_neighbours.end(), current),
            //                            close_neighbours.end());
            //     current = current->parent;
            // }
            std::cout << "Found candidates and removed some " << close_neighbours.size() << "\n";
            //std::vector<TrajectorySegment *> new_parent = {sample};
            for (int i = 0; i < close_neighbours.size(); ++i)
            {

                EigenTrajectoryPointVector trajectory;
                EigenTrajectoryPoint start = sample->trajectory.back();
                EigenTrajectoryPoint goal = close_neighbours[i]->trajectory.back();
                //     std::cout << "goal point selected "
                //               << "\n";
                //     // if (!connectPoses(start_point, goal_point, &trajectory))
                //     // {
                //     //     std::cout << "poses not connected "
                //     //               << "\n";
                //     //     continue;
                //     // }
                //     std::cout << "connectPoses"
                //               << "\n";
                Eigen::Vector3d start_pos = start.position_W;
                Eigen::Vector3d direction = goal.position_W - start_pos;
                std::cout << "start pos " << start_pos.x() << " " << start_pos.y() << " " << start_pos.z() << "\n";
                std::cout << "goal pos " << goal.position_W.x() << " " << goal.position_W.y() << " " << goal.position_W.z() << "\n";
                std::cout << " direction " << direction[0] << " " << direction[1] << " " << direction[2] << "\n";
                //std::cout << "compute n points "<< direction.norm() <<"\n";
                //int n_points = std::ceil(direction.norm() / planner_.getSystemConstraints().v_max * p_sampling_rate_);
                //std::cout << "compute "<< n_points <<" " << direction.norm() << "\n";
                //for (int i = 0; i < n_points; ++i)
                //{
                // if (!checkTraversable(start_pos + (double)i / (double)n_points * direction))
                // {
                //     return false;
                // }
                //}

                //     // Build trajectory
                //     n_points = std::ceil(direction.norm() / planner_.getSystemConstraints().v_max *
                //                          p_sampling_rate_);
                // for (int i = 0; i < n_points; ++i)
                // {
                //     EigenTrajectoryPoint trajectory_point;
                //     trajectory_point.position_W =
                //         start_pos + (double)i / (double)n_points * direction;
                //     trajectory_point.setFromYaw(goal.getYaw());
                //     trajectory_point.time_from_start_ns =
                //         static_cast<int64_t>((double)i / p_sampling_rate_ * 1.0e9);
                //     result->push_back(trajectory_point);
                // }

                //sample->parent->children.push_back(
                //    std::unique_ptr<TrajectorySegment>(close_neighbours[i]));

                //rewireToBestParent(close_neighbours[i], new_parent);
            }

            // Add to the kdtree
            // if (sample->parent)
            // {
            //     tree_data_.addSegment(sample);
            //     kdtree_->addPoints(tree_data_.points.size() - 1, tree_data_.points.size() - 1);
            //     return true;
            // }
            // else
            // {
            //     delete sample;
            //     return false;
            // }

            return true;
        }

        bool InformedRRTStar::rewireToBestParent(TrajectorySegment *segment,
                                                 const std::vector<TrajectorySegment *> &candidates, bool force_rewire)
        {
            // Evaluate all candidate parents and store the best one in the segment
            // Goal is the end point of the segment
            std::cout << "rewireToBestParent"
                      << "\n";
            EigenTrajectoryPoint goal_point = segment->trajectory.back();
            std::cout << "goal_point"
                      << "\n";

            // store the initial segment
            TrajectorySegment best_segment = segment->shallowCopy();
            std::cout << "best_segment"
                      << "\n";
            TrajectorySegment *initial_parent = segment->parent;
            std::cout << "initial_parent"
                      << "\n";

            // Find best segment
            for (int i = 0; i < candidates.size(); ++i)
            {
                segment->trajectory.clear();
                segment->parent = candidates[i];
                if (connectPoses(candidates[i]->trajectory.back(), goal_point,
                                 &(segment->trajectory)))
                {
                    std::cout << "Feasible connection"
                              << "\n";
                    // Feasible connection: evaluate the trajectory
                    planner_.getTrajectoryEvaluator().computeCost(segment);
                    planner_.getTrajectoryEvaluator().computeValue(segment);
                    std::cout << "got cost and value"
                              << "\n";
                    if (best_segment.parent == nullptr || force_rewire ||
                        segment->value > best_segment.value)
                    {
                        best_segment = segment->shallowCopy();
                        force_rewire = false;
                    }
                }
            }
            if (best_segment.parent == nullptr)
            {
                // No connection found and no previous trajectory
                return false;
            }
            else
            {
                // Apply best segment and rewire
                std::cout << "Apply best segment and rewire"
                          << "\n";
                segment->parent = best_segment.parent;
                segment->trajectory = best_segment.trajectory;
                segment->cost = best_segment.cost;
                planner_.getTrajectoryEvaluator().computeValue(segment);
                if (segment->parent == initial_parent)
                {
                    // Back to old parent
                    return ~force_rewire;
                }
                else if (initial_parent == nullptr)
                {
                    // Found new parent
                    std::cout << "Found new parent"
                              << "\n";
                    segment->parent->children.push_back(
                        std::unique_ptr<TrajectorySegment>(segment));
                    return true;
                }
                else
                {
                    for (int i = 0; i < initial_parent->children.size(); ++i)
                    {

                        if (initial_parent->children[i].get() == segment)
                        {
                            std::cout << " Move from existing parent"
                                      << "\n";
                            // Move from existing parent
                            segment->parent->children.push_back(
                                std::move(initial_parent->children[i]));
                            initial_parent->children.erase(initial_parent->children.begin() + i);
                            // update subtree
                            if (p_update_subsequent_)
                            {
                                std::vector<TrajectorySegment *> subtree;
                                segment->getTree(&subtree);
                                for (int j = 1; j < subtree.size(); ++j)
                                {
                                    planner_.getTrajectoryEvaluator().computeValue(subtree[j]);
                                }
                            }
                            return true;
                        }
                    }
                    // Rewiring failed (should not happen by construction)
                    return false;
                }
            }
        }

        bool InformedRRTStar::getSample(Eigen::Vector3d *sample_position, Eigen::Vector3d *target_position)
        {

            double x_min = (*target_position)[0] - bounding_volume_->x_min;
            double x_max = (*target_position)[0] + bounding_volume_->x_min;
            double y_min = (*target_position)[1] - bounding_volume_->y_min;
            double y_max = (*target_position)[1] + bounding_volume_->y_min;
            double z_min = (*target_position)[2] - bounding_volume_->z_min;
            double z_max = (*target_position)[2] + bounding_volume_->z_min;

            // sample from bounding volume (assumes box atm)
            (*sample_position)[0] = x_min + (double)rand() / RAND_MAX * (x_max - x_min);
            (*sample_position)[1] = y_min + (double)rand() / RAND_MAX * (y_max - y_min);
            (*sample_position)[2] = z_min + (double)rand() / RAND_MAX * (z_max - z_min);

            return true;
        }

        bool InformedRRTStar::connectPoses(const EigenTrajectoryPoint &start,
                                           const EigenTrajectoryPoint &goal,
                                           EigenTrajectoryPointVector *result,
                                           bool check_collision)
        {
            // try creating a linear trajectory and check for collision

            Eigen::Vector3d start_pos = start.position_W;
            Eigen::Vector3d direction = goal.position_W - start_pos;

            int n_points = std::ceil(direction.norm() / planner_.getSystemConstraints().v_max * p_sampling_rate_);
            if (check_collision)
            {
                for (int i = 0; i < n_points; ++i)
                {
                    if (!checkTraversable(start_pos + (double)i / (double)n_points * direction))
                    {
                        return false;
                    }
                }
            }
            // Build trajectory
            n_points = std::ceil(direction.norm() / planner_.getSystemConstraints().v_max *
                                 p_sampling_rate_);
            for (int i = 0; i < n_points; ++i)
            {
                EigenTrajectoryPoint trajectory_point;
                trajectory_point.position_W =
                    start_pos + (double)i / (double)n_points * direction;
                trajectory_point.setFromYaw(goal.getYaw());
                trajectory_point.time_from_start_ns =
                    static_cast<int64_t>((double)i / p_sampling_rate_ * 1.0e9);
                result->push_back(trajectory_point);
            }
            return true;
        }

        bool InformedRRTStar::resetTree(TrajectorySegment *root)
        {
            // Remove everything and populate with the semgents from root
            std::vector<TrajectorySegment *> currrent_tree;
            root->getTree(&currrent_tree);
            tree_data_.clear();
            for (int i = 0; i < currrent_tree.size(); ++i)
            {
                tree_data_.addSegment(currrent_tree[i]);
            }
            kdtree_ = std::unique_ptr<KDTree>(new KDTree(3, tree_data_));
            kdtree_->addPoints(0, tree_data_.points.size() - 1);
        }
        void InformedRRTStar::TreeData::clear()
        {
            points.clear();
            data.clear();
        }

        void InformedRRTStar::TreeData::addSegment(TrajectorySegment *to_add)
        {
            points.push_back(to_add->trajectory.back().position_W);
            data.push_back(to_add);
        }

    } // namespace trajectory_generator

    namespace trajectory_evaluator
    {

        // RRTStarEvaluatorAdapter (just delegate everything, call rewire on select
        // best)
        ModuleFactoryRegistry::Registration<InformedRRTStarEvaluatorAdapter>
            InformedRRTStarEvaluatorAdapter::registration("InformedRRTStarEvaluatorAdapter");

        InformedRRTStarEvaluatorAdapter::InformedRRTStarEvaluatorAdapter(PlannerI &planner)
            : TrajectoryEvaluator(planner) {}

        bool InformedRRTStarEvaluatorAdapter::computeGain(TrajectorySegment *traj_in)
        {
            return following_evaluator_->computeGain(traj_in);
        }

        bool InformedRRTStarEvaluatorAdapter::computeCost(TrajectorySegment *traj_in)
        {
            return following_evaluator_->computeCost(traj_in);
        }

        bool InformedRRTStarEvaluatorAdapter::computeValue(TrajectorySegment *traj_in)
        {
            return following_evaluator_->computeValue(traj_in);
        }

        // int InformedRRTStarEvaluatorAdapter::selectNextBest(TrajectorySegment *traj_in) {
        //     int next = following_evaluator_->selectNextBest(traj_in);
        //     generator_->rewireRoot(traj_in, &next);
        //     return next;
        // }

        bool InformedRRTStarEvaluatorAdapter::updateSegment(TrajectorySegment *segment)
        {
            return following_evaluator_->updateSegment(segment);
        }

        void InformedRRTStarEvaluatorAdapter::visualizeTrajectoryValue(
            VisualizationMarkers *markers, const TrajectorySegment &trajectory)
        {
            following_evaluator_->visualizeTrajectoryValue(markers, trajectory);
        }

        void InformedRRTStarEvaluatorAdapter::setupFromParamMap(Module::ParamMap *param_map)
        {
            // generator_ = dynamic_cast<trajectory_generator::InformedRRTStar *>(
            //     planner_.getFactory().readLinkableModule("InformedRRTStarGenerator"));
            // Create following evaluator
            std::string args; // default args extends the parent namespace
            std::string param_ns = (*param_map)["param_namespace"];
            setParam<std::string>(param_map, "following_evaluator_args", &args,
                                  param_ns + "/following_evaluator");
            following_evaluator_ =
                planner_.getFactory().createModule<TrajectoryEvaluator>(args, planner_,
                                                                        verbose_modules_);

            // setup parent
            TrajectoryEvaluator::setupFromParamMap(param_map);
        }

    } // namespace trajectory_evaluator
} // namespace active_3d_planning