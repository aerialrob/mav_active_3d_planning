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
            setParam<int>(param_map, "maximum_tries", &p_maximum_tries_, 20);
            setParam<double>(param_map, "goal_x", &p_goal_x_, 0.0);
            setParam<double>(param_map, "goal_y", &p_goal_y_, 0.0);
            setParam<double>(param_map, "goal_z", &p_goal_z_, 0.0);
            setParam<double>(param_map, "prob_divisions", &p_prob_divisions_, 10.0);

            previous_root_ = nullptr;
            home_pose_ = {0.0, 10.0, 1.5};
            current_goal_ = {p_goal_x_, p_goal_y_, p_goal_z_};
            // setup parent
            TrajectoryGenerator::setupFromParamMap(param_map);
            initializeDistribution();
        }

        bool InformedRRTStar::selectSegment(TrajectorySegment **result,
                                            TrajectorySegment *root)
        {
            if (previous_root_ != root)
            {
                resetTree(root);
                previous_root_ = root;
            }

            Eigen::Vector3d sample;
            Eigen::Vector3d current_position = root->trajectory.back().position_W;
            bool sample_found = false;
            int counter = 0;
            std::size_t ret_index;
            while (!sample_found && counter <= p_maximum_tries_)
            {
                getSample(&sample, &current_position);
                //std::cout << "Got sample " << sample << "\n";
                if (checkTraversable(sample))
                {
                    double query_pt[3] = {sample.x(), sample.y(), sample.z()};
                    double out_dist_sqr;
                    nanoflann::KNNResultSet<double> resultSet(1);
                    resultSet.init(&ret_index, &out_dist_sqr);
                    if (!kdtree_->findNeighbors(resultSet, query_pt,
                                                nanoflann::SearchParams(10)))
                    {
                        continue;
                    }
                    if (out_dist_sqr <= p_max_sample_dist_ * p_max_sample_dist_ && out_dist_sqr >= p_min_sample_dist_ * p_min_sample_dist_)
                    {
                        sample_found = true;
                    }
                }
                counter++;
            }

            //double sample_dist = (current_goal_ - sample).norm();
            //double current_dist = (current_goal_ - current_position).norm();
            Eigen::Vector2d sample_distance2d{current_goal_(0) - sample(0), current_goal_(1) - sample(1)};
            double sample_dist = sample_distance2d.norm();
            Eigen::Vector2d distance2d{current_goal_(0) - current_position(0), current_goal_(1) - current_position(1)};
            double current_dist = distance2d.norm();
            //std::cout << "Current dist new goal is " << current_dist << "\n";
            if (current_dist <= 1.0)
            {
                // /goal_reached_ = true;
                current_goal_ = home_pose_;
                //std::cout << "GOAL REACHED , new goal is " << current_goal_ << "\n";
                sample_dist = (current_goal_ - sample).norm();
                current_dist = (current_goal_ - current_position).norm();
                initializeDistribution();
            }

            if (sample_dist > current_dist)
            {
                updateDistribution(&sample, 1, false);
                //std::cout << "Updated weights " << weights_y_[0] << " " << weights_y_[1] << " " << weights_y_[2] << " " << weights_y_[3] << " " << weights_y_[4] << " " << weights_y_[5] << " " << weights_y_[6] << " " << weights_y_[7] << " " << weights_y_[8] << " " << weights_y_[9] << "\n";
            }

            if (!sample_found)
            {
                *result = nullptr;
                return false;
            }

            // Lower probability of sampling behind the current position

            connecting_sample_ = sample;
            *result = tree_data_.data[ret_index];
            return true;
        }

        bool InformedRRTStar::expandSegment(TrajectorySegment *target,
                                            std::vector<TrajectorySegment *> *new_segments)
        {
            if (!target)
            {
                // Segment selection failed
                return false;
            }

            EigenTrajectoryPointVector trajectory;
            EigenTrajectoryPoint start_point = target->trajectory.back();
            EigenTrajectoryPoint goal_point;
            goal_point.position_W = connecting_sample_;
            goal_point.setFromYaw(M_PI / 2);
            if (!connectPoses(start_point, goal_point, &trajectory))
            {
                //std::cout << "[EXPAND] Could not connect!\n";
                return false;
            }

            // Build result and add it to the kdtree
            TrajectorySegment *new_segment;
            new_segment = target->spawnChild();
            new_segment->trajectory = trajectory;
            new_segments->push_back(new_segment);
            tree_data_.addSegment(new_segment);
            kdtree_->addPoints(tree_data_.points.size() - 1,
                               tree_data_.points.size() - 1);

            // Compute gain and update prob distribution
            planner_.getTrajectoryEvaluator().computeGain(new_segment);
            new_segment->tg_visited = true;
            if (new_segment->gain >= 50)
            {
                Eigen::Vector3d useful_point = new_segment->trajectory.back().position_W;
                updateDistribution(&useful_point, 0, true);
            }
            return true;
            //std::cout << "[EXPAND] Added sample!\n";
        }

        bool InformedRRTStar::getCloseNeighbours(const Eigen::Vector3d &target_point, std::vector<Eigen::Vector3d> *close_neighbours)
        {
            // Also tried radius search here but that stuff blows for some reason...
            double query_pt[3] = {target_point.x(), target_point.y(), target_point.z()};
            std::size_t ret_index[p_n_neighbors_];
            double out_dist_sqrt[p_n_neighbors_];
            nanoflann::KNNResultSet<double> resultSet(p_n_neighbors_);
            resultSet.init(ret_index, out_dist_sqrt);
            bool found_neighbours = kdtree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));
            std::cout << "Found new candidates " << resultSet.size() << "\n";
            bool candidate_found = false;
            if (found_neighbours)
            {

                for (int i = 0; i < resultSet.size(); ++i)
                {
                    std::cout << "Candidate " << i << " distance " << out_dist_sqrt[i] << "\n";
                    if (out_dist_sqrt[i] <= p_max_sample_dist_ * p_max_sample_dist_ && out_dist_sqrt[i] > 0.25)
                    {
                        candidate_found = true;
                        //std::cout << "candidates " << tree_data_.data[ret_index[i]]->trajectory.back().position_W.x() << " " << tree_data_.data[ret_index[i]]->trajectory.back().position_W.y() << " " << tree_data_.data[ret_index[i]]->trajectory.back().position_W.z() << "\n";
                        //close_neighbours->push_back(tree_data_.data[ret_index[i]]);
                        close_neighbours->push_back(tree_data_.points[ret_index[i]]);
                    }
                }
            }
            return candidate_found;
        }

        bool InformedRRTStar::initializeDistribution()
        {
            //std::cout << "Initialize dist \n";
            weights_x_.clear();
            weights_y_.clear();
            for (int i = 0; i < p_prob_divisions_; i++)
            {
                weights_x_.push_back(10.0);
                weights_y_.push_back(10.0);
            }

            std::discrete_distribution<int> interm(weights_x_.begin(), weights_x_.end());
            distribution_prob_x_ = interm;
            distribution_prob_y_ = interm;
            return true;
        }

        bool InformedRRTStar::updateDistribution(Eigen::Vector3d *sampled_point, int dim, bool higher)
        {
            //std::cout << "Update Dist" << (*sampled_point)[0] << " " << (*sampled_point)[1] << "\n";

            double x_step = (local_volume_max_[0] - local_volume_min_[0]) / p_prob_divisions_;
            double y_step = (local_volume_max_[1] - local_volume_min_[1]) / p_prob_divisions_;

            double update_weight = 0.1;
            bool found_x = false;
            bool found_y = false;

            if (!higher)
            {
                update_weight = -0.5;
            }

            for (int i = 1; i < p_prob_divisions_; i++)
            {
                //std::cout << "insiede loop " << update_weight << "\n";
                if (dim == 0)
                {
                    if ((*sampled_point)[0] <= local_volume_min_[0] + i * x_step && !found_x)
                    {
                        weights_x_[i - 1] = weights_x_[i - 1] + update_weight;
                        std::discrete_distribution<int> updated_dist_x(weights_x_.begin(), weights_x_.end());
                        distribution_prob_x_ = updated_dist_x;
                        found_x = true;
                        break;
                        //std::cout << "RRT distribution " << i << distribution_prob.probabilities()[0] << " " << distribution_prob.probabilities()[1] << " " << distribution_prob.probabilities()[2] << " " << distribution_prob.probabilities()[3] << " " << distribution_prob.probabilities()[4] << " " << distribution_prob.probabilities()[5] << " " << distribution_prob.probabilities()[6] << " " << distribution_prob.probabilities()[7] << " " << distribution_prob.probabilities()[8] << " " << distribution_prob.probabilities()[9] << "\n";
                    }
                }

                if (dim == 1)
                {
                    //std::cout << " Comprobavions " << (*sampled_point)[1] << " " << local_volume_min_[1] << "step " << y_step << "\n";
                    //std::cout << "Update y \n";
                    if ((*sampled_point)[1] <= local_volume_min_[1] + i * y_step && !found_y)
                    {
                        weights_y_[i - 1] = weights_y_[i - 1] + update_weight;
                        std::discrete_distribution<int> updated_dist_y(weights_y_.begin(), weights_y_.end());
                        distribution_prob_y_ = updated_dist_y;
                        found_y = true;
                        //std::cout << "RRT distribution " << i << distribution_prob_y_.probabilities()[0] << " " << distribution_prob_y_.probabilities()[1] << " " << distribution_prob_y_.probabilities()[2] << " " << distribution_prob_y_.probabilities()[3] << " " << distribution_prob_y_.probabilities()[4] << " " << distribution_prob_y_.probabilities()[5] << " " << distribution_prob_y_.probabilities()[6] << " " << distribution_prob_y_.probabilities()[7] << " " << distribution_prob_y_.probabilities()[8] << " " << distribution_prob_y_.probabilities()[9] << "\n";
                        break;
                    }
                    //std::cout << "Update weights Y" << weights_y_[0] << " " << weights_y_[1] << " " << weights_y_[2] << " " << weights_y_[3] << " " << weights_y_[4] << " " << weights_y_[5] << " " << weights_y_[6] << " " << weights_y_[7] << " " << weights_y_[8] << " " << weights_y_[9] << "\n";
                }
            }
        }

        bool InformedRRTStar::rewireSample(TrajectorySegment *new_segment, std::vector<Eigen::Vector3d> &candidates, std::vector<TrajectorySegment *> *new_segments)
        {

            std::cout << "Rewiring"
                      << "\n";
            EigenTrajectoryPoint start_point = new_segment->trajectory.back();

            // TrajectorySegment *current = sample;
            // while (current)
            // {
            //     // the connection of the new segment to the root cannot be rewired
            //     // (loops!)
            //     close_neighbours.erase(std::remove(close_neighbours.begin(),
            //                                        close_neighbours.end(), current),
            //                            close_neighbours.end());
            //     current = current->parent;
            // }
            //std::cout << "Found candidates and removed some " << close_neighbours.size() << "\n";
            //std::vector<TrajectorySegment *> new_parent = {sample};
            std::cout << "[REWIRE] close neighbours " << candidates.size() << "\n";
            for (int i = 0; i < candidates.size(); ++i)
            {

                //TrajectorySegment *segment = candidates[i];
                EigenTrajectoryPointVector trajectory;
                EigenTrajectoryPoint start = start_point; //new_segment->trajectory.back();
                EigenTrajectoryPoint goal;
                goal.position_W = candidates[i]; //segment->trajectory.back();

                if (!connectPoses(start, goal, &trajectory))
                {
                    std::cout << "poses not connected "
                              << "\n";
                    continue;
                }
                else
                {
                    int index = 0;
                    TrajectorySegment *candidate_new_segment;
                    TrajectorySegment *new_segment;
                    if (findIndexFromPoint(candidates[i], *new_segments, candidate_new_segment))
                    {
                        std::cout << "[REWIRE] found index " << candidate_new_segment->trajectory.back().position_W << "\n";
                        //segment->parent = new_segment->parent;
                        //segment->trajectory = new_segment->trajectory;
                        //segment->cost = new_segment->cost;

                        new_segment = candidate_new_segment->spawnChild();
                        new_segment->trajectory = trajectory;

                        //std::cout << "Traj point" << trajectory.back().position_W << "\n";

                        new_segments->push_back(new_segment);
                        tree_data_.addSegment(new_segment);
                        kdtree_->addPoints(tree_data_.points.size() - 1,
                                           tree_data_.points.size() - 1);
                    }
                    std::cout << "[REWIRE] Index not found"
                              << "\n";
                }

                //     std::cout << "connectPoses"
                //               << "\n";
                //Eigen::Vector3d start_pos = start.position_W;
                //Eigen::Vector3d direction = goal.position_W - start_pos;
                //std::cout << "start pos " << start_pos.x() << " " << start_pos.y() << " " << start_pos.z() << "\n";
                //std::cout << "goal pos " << goal.position_W.x() << " " << goal.position_W.y() << " " << goal.position_W.z() << "\n";
                //std::cout << " direction " << direction[0] << " " << direction[1] << " " << direction[2] << "\n";
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

        bool InformedRRTStar::findIndexFromPoint(Eigen::Vector3d point, std::vector<TrajectorySegment *> &new_segments, TrajectorySegment *segment)
        {

            std::cout << "New segments size " << new_segments.size() << "\n";
            for (int i = 0; i < new_segments.size(); i++)
            {
                Eigen::Vector3d last_point = new_segments[i]->trajectory.back().position_W;

                std::cout << "QUery point " << point << " segment point " << last_point << "\n";
                if (last_point.x() == point.x() && last_point.x() == point.x() && last_point.x() == point.x())
                {
                    std::cout << "REWIRE Found index " << i << " last point " << last_point << "\n";
                    segment = new_segments[i];
                    return true;
                }
            }
            return false;
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
                std::cout << " No connection found and no previous trajectory"
                          << "\n";
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

            local_volume_min_[0] = (*target_position)[0] - bounding_volume_->x_min;
            local_volume_max_[0] = (*target_position)[0] + bounding_volume_->x_min;
            local_volume_min_[1] = (*target_position)[1] - bounding_volume_->y_min;
            local_volume_max_[1] = (*target_position)[1] + bounding_volume_->y_min;
            local_volume_min_[2] = (*target_position)[2] - bounding_volume_->z_min;
            local_volume_max_[2] = (*target_position)[2] + bounding_volume_->z_min;

            double prob_y = distribution_prob_y_(generator);
            double prob_x = distribution_prob_x_(generator);
            //std::cout << "PRob " << prob_y << " prob global " << (prob_y / p_prob_divisions_)<< "\n";
            //std::cout << " Local volume " << local_volume_min_[0] << " " << local_volume_max_[0] <<  "prob "<< prob_y << "\n";
            // sample from bounding volume (assumes box atm)
            (*sample_position)[0] = local_volume_min_[0] + (prob_x / p_prob_divisions_) * (local_volume_max_[0] - local_volume_min_[0]);
            (*sample_position)[1] = local_volume_min_[1] + (prob_y / p_prob_divisions_) * (local_volume_max_[1] - local_volume_min_[1]); //y_min + (double)rand() / RAND_MAX * (y_max - y_min);
            (*sample_position)[2] = local_volume_min_[2] + (double)rand() / RAND_MAX * (local_volume_max_[2] - local_volume_min_[2]);

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