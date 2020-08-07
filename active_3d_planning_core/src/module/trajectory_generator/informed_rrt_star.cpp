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

#define M_PI 3.14159265358979323846 /* pi */
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
            setParam<bool>(param_map, "rewire", &p_rewire_, false);
            setParam<double>(param_map, "yaw_range", &p_yaw_range_, M_PI);
            setParam<bool>(param_map, "rewire_update", &p_rewire_update_, false);
            setParam<double>(param_map, "prob_weight", &p_prob_weight_, false);
            setParam<bool>(param_map, "rewire_root", &p_rewire_root_, false);
            setParam<bool>(param_map, "reinsert_root", &p_reinsert_root_, false);
            setParam<double>(param_map, "tunnel_width", &p_tunnel_width_, 10.0);
            setParam<bool>(param_map, "apply_prob_dist", &p_apply_prob_dist_, false);
            setParam<bool>(param_map, "find_wall_direction", &p_find_wall_direction_, false);

            previous_root_ = nullptr;
            wall_direction_ = 0;
            last_wall_direction_ = 1;
            wall_detected_ = false;

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
                if (p_rewire_update_)
                {
                    rewireIntermediate(root);
                }
            }

            Eigen::Vector3d sample;
            double sampled_yaw;
            double current_yaw = root->trajectory.back().getYaw();
            bool sample_found = false;
            int counter = 0;
            std::size_t ret_index;
            setCurrentPosition(root);
            updateDistance2Goal();
            if (p_find_wall_direction_)
            {
                findWallDirection();
            }
            while (!sample_found && counter <= p_maximum_tries_)
            {
                //Eigen::Vector3d *sampled_position, double *sampled_yaw, Eigen::Vector3d *current_pose_, double *current_yaw
                getSample(&sample, &sampled_yaw, &current_yaw);
                //std::cout << "Got sample " << sample << " traversable " << checkTraversable(sample) << "\n";
                if (checkTraversable(sample))
                {
                    //std::cout << "Traversable" << "\n";
                    double query_pt[3] = {sample.x(), sample.y(), sample.z()};
                    double out_dist_sqr;
                    nanoflann::KNNResultSet<double> resultSet(1);
                    resultSet.init(&ret_index, &out_dist_sqr);
                    if (!kdtree_->findNeighbors(resultSet, query_pt,
                                                nanoflann::SearchParams(10)))
                    {
                        std::cout << "No neighbour\n";
                        continue;
                    }
                    if ((out_dist_sqr <= p_max_sample_dist_ * p_max_sample_dist_) && (out_dist_sqr >= p_min_sample_dist_ * p_min_sample_dist_))
                    {
                        sample_found = true;
                    }
                }
                counter++;
            }

            Eigen::Vector2d sample_distance2d{global_goal_(0) - sample(0), global_goal_(1) - sample(1)};
            double sample_dist = sample_distance2d.norm();

            if (!sample_found)
            {
                *result = nullptr;
                return false;
            }

            // Lower probability of sampling behind the current position

            connecting_sample_ = sample;
            connecting_yaw_ = sampled_yaw;
            *result = tree_data_.data[ret_index];
            return true;
        }

        void InformedRRTStar::setCurrentPosition(TrajectorySegment *root)
        {
            current_pose_ = root->trajectory.back().position_W;
        }
        void InformedRRTStar::updateDistance2Goal()
        {
            goal_distance_ = (global_goal_ - current_pose_).norm();
            if (new_global_goal_)
            {
                // If a new goal is given, initialize the prob distribution as the direction might have changed
                initializeDistribution();
            }
        }

        bool InformedRRTStar::findWallDirection()
        {

            if (!wall_detected_)
            {
                planner_.getTrajectoryEvaluator().initializeObservedVolume();
            }

            if (wall_direction_ == 0)
            {
                if (checkValidLocalVolume() && (observed_bounding_volume_[0][1] - observed_bounding_volume_[0][0] <= p_tunnel_width_))
                {
                    wall_detected_ = true;
                    return wall_detected_;
                }
                else if (checkValidLocalVolume() && (observed_bounding_volume_[1][1] - observed_bounding_volume_[1][0] <= p_tunnel_width_))
                {
                    wall_detected_ = true;
                    wall_direction_ = 1;
                    return wall_detected_;
                }
                else
                {
                    wall_detected_ = false;
                    return wall_detected_;
                }
            }
            else
            {
                if (checkValidLocalVolume() && (observed_bounding_volume_[1][1] - observed_bounding_volume_[1][0] <= p_tunnel_width_))
                {
                    wall_detected_ = true;
                    return wall_detected_;
                }
                else if (checkValidLocalVolume() && (observed_bounding_volume_[0][1] - observed_bounding_volume_[0][0] <= p_tunnel_width_))
                {
                    wall_detected_ = true;
                    wall_direction_ = 0;
                    return wall_detected_;
                }
                else
                {
                    wall_detected_ = false;
                    return wall_detected_;
                }
            }
        }

        bool InformedRRTStar::checkValidLocalVolume()
        {
            bool valid_x = false;
            bool valid_y = false;
            if ((current_pose_[0] > observed_bounding_volume_[0][0]) && (current_pose_[0] < observed_bounding_volume_[0][1]) && (observed_bounding_volume_[0][1] - observed_bounding_volume_[0][0] > 1e-4))
            {
                valid_x = true;
            }
            if ((current_pose_[1] > observed_bounding_volume_[1][0]) && (current_pose_[1] < observed_bounding_volume_[1][1]) && (observed_bounding_volume_[1][1] - observed_bounding_volume_[1][0] > 1e-4))
            {
                valid_y = true;
            }

            if (valid_x && valid_y)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        bool InformedRRTStar::expandSegment(TrajectorySegment *target,
                                            std::vector<TrajectorySegment *> *new_segments)
        {
            tree_is_reset_ = true;
            if (!target)
            {
                // Segment selection failed
                return false;
            }

            EigenTrajectoryPointVector trajectory;
            EigenTrajectoryPoint start_point = target->trajectory.back();
            EigenTrajectoryPoint goal_point;
            goal_point.position_W = connecting_sample_;

            // Use a completely random yaw
            //double random_yaw = (double)rand() / (double)RAND_MAX * p_yaw_range_; //(2 * (double)rand() / (double)RAND_MAX - 1) * p_yaw_range_;
            //goal_point.setFromYaw(random_yaw);

            // Set sampled yaw
            goal_point.setFromYaw(connecting_yaw_);
            if (!connectPoses(start_point, goal_point, &trajectory))
            {
                //std::cout << "Cannot connect sample " << start_point.position_W << " goal " << goal_point.position_W << "\n";
                return false;
            }

            // Build result and add it to the kdtree
            TrajectorySegment *new_segment;
            new_segment = target->spawnChild();
            new_segment->trajectory = trajectory;

            // Compute gain and update prob distribution
            planner_.getTrajectoryEvaluator().computeGain(new_segment);
            planner_.getTrajectoryEvaluator().computeCost(new_segment, &current_pose_);
            planner_.getTrajectoryEvaluator().computeValue(new_segment);
            new_segment->tg_visited = true;
            if (new_segment->gain > 0.0)
            {
                Eigen::Vector3d useful_point = new_segment->trajectory.back().position_W;
                double yaw = new_segment->trajectory.back().getYaw();
                if (wall_detected_)
                {
                    //updateDistribution(&useful_point, 0.0, wall_direction_, true, new_segment->gain);
                }
                //updateDistribution(&useful_point, yaw, 2, true, new_segment->gain);
                // Update yaw weights
                updateDistribution(&useful_point, yaw, 3, true, new_segment->gain);
            }

            // Find nearby parent candidates
            if (p_rewire_)
            {
                std::vector<TrajectorySegment *> candidate_parents;
                if (getCloseNeighbours(connecting_sample_, &candidate_parents))
                {
                    rewireToBestParent(new_segment, candidate_parents);
                }
            }

            if (new_segment->parent)
            {
                new_segments->push_back(new_segment);
                tree_data_.addSegment(new_segment);
                kdtree_->addPoints(tree_data_.points.size() - 1,
                                   tree_data_.points.size() - 1);

                return true;
            }
            else
            {
                std::cout << "[EXPAND] Sample not added no parent!\n";
            }
        }

        bool InformedRRTStar::getCloseNeighbours(const Eigen::Vector3d &target_point, std::vector<TrajectorySegment *> *close_neighbours)
        {
            // Also tried radius search here but that stuff blows for some reason...
            double query_pt[3] = {target_point.x(), target_point.y(), target_point.z()};
            std::size_t ret_index[p_n_neighbors_];
            double out_dist_sqrt[p_n_neighbors_];
            nanoflann::KNNResultSet<double> resultSet(p_n_neighbors_);
            resultSet.init(ret_index, out_dist_sqrt);
            bool found_neighbours = kdtree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));
            bool candidate_found = false;
            if (found_neighbours)
            {

                for (int i = 0; i < resultSet.size(); ++i)
                {
                    //std::cout << "Candidate " << i << " distance " << out_dist_sqrt[i] << "\n";
                    if (out_dist_sqrt[i] <= p_max_sample_dist_ * p_max_sample_dist_ && out_dist_sqrt[i] > 0.25)
                    {
                        candidate_found = true;
                        //std::cout << "candidates " << tree_data_.data[ret_index[i]]->trajectory.back().position_W.x() << " " << tree_data_.data[ret_index[i]]->trajectory.back().position_W.y() << " " << tree_data_.data[ret_index[i]]->trajectory.back().position_W.z() << "\n";
                        //close_neighbours->push_back(tree_data_.points[ret_index[i]]);
                        close_neighbours->push_back(tree_data_.data[ret_index[i]]);
                    }
                }
            }
            return candidate_found;
        }

        bool InformedRRTStar::initializeDistribution()
        {
            //std::cout << "Initialize dist \n";
            weights_wall_.clear();
            //weights_y_.clear();
            weights_z_.clear();
            weights_yaw_.clear();

            for (int i = 0; i < p_prob_divisions_; i++)
            {
                if (i > (p_prob_divisions_ / 3) - 3 || i < 2 * (p_prob_divisions_ / 3) + 2)
                {
                    weights_wall_.push_back(10.0);
                    //weights_y_.push_back(10.0);
                    weights_z_.push_back(10.0);
                    weights_yaw_.push_back(10.0);
                }
                else
                {
                    weights_wall_.push_back(1000.0);
                    weights_z_.push_back(1000.0);
                    weights_yaw_.push_back(10.0);
                }
            }

            std::discrete_distribution<int> interm(weights_wall_.begin(), weights_wall_.end());
            std::discrete_distribution<int> interm_yaw(weights_yaw_.begin(), weights_yaw_.end());
            distribution_prob_wall_ = interm;
            distribution_prob_z_ = interm;
            distribution_prob_yaw_ = interm_yaw;
            return true;
        }

        bool InformedRRTStar::updateDistribution(Eigen::Vector3d *sampled_point, double sampled_yaw, int dim, bool higher, double gain)
        {
            //std::cout << "Update Dist" << (*sampled_point)[0] << " " << (*sampled_point)[1] << "\n";

            double step = (local_volume_max_[dim] - local_volume_min_[dim]) / p_prob_divisions_;
            //double y_step = (local_volume_max_[1] - local_volume_min_[1]) / p_prob_divisions_;
            double z_step = (z_max_ - z_min_) / p_prob_divisions_;
            double yaw_step = (yaw_max_ - yaw_min_) / p_prob_divisions_;

            double update_weight = gain * p_prob_weight_;
            bool found_wall = false;
            bool found_z = false;
            bool found_yaw = false;

            if (!higher)
            {
                update_weight = -0.5;
            }

            for (int i = 1; i < p_prob_divisions_; i++)
            {
                //std::cout << "insiede loop " << update_weight << "\n";
                if (dim == 2)
                {
                    if ((*sampled_point)[2] <= z_min_ + i * z_step && !found_z)
                    {
                        weights_z_[i] = weights_z_[i] + update_weight;
                        std::discrete_distribution<int> updated_dist_z(weights_z_.begin(), weights_z_.end());
                        distribution_prob_z_ = updated_dist_z;
                        found_z = true;
                        break;
                    }
                    //std::cout << "Update weights Z" << weights_z_[0] << " " << weights_z_[1] << " " << weights_z_[2] << " " << weights_z_[3] << " " << weights_z_[4] << " " << weights_z_[5] << " " << weights_z_[6] << " " << weights_z_[7] << " " << weights_z_[8] << " " << weights_z_[9] << "\n";
                }

                if (dim == 3)
                {
                    if (sampled_yaw <= yaw_min_ + i * yaw_step && !found_yaw)
                    {
                        weights_yaw_[i] = weights_yaw_[i] + update_weight;
                        std::discrete_distribution<int> updated_dist_yaw(weights_yaw_.begin(), weights_yaw_.end());
                        distribution_prob_yaw_ = updated_dist_yaw;
                        found_yaw = true;
                        break;
                    }
                    //std::cout << "Update weights Z" << weights_z_[0] << " " << weights_z_[1] << " " << weights_z_[2] << " " << weights_z_[3] << " " << weights_z_[4] << " " << weights_z_[5] << " " << weights_z_[6] << " " << weights_z_[7] << " " << weights_z_[8] << " " << weights_z_[9] << "\n";
                }

                else
                {
                    if ((*sampled_point)[dim] <= local_volume_min_[dim] + i * step && !found_wall)
                    {
                        weights_wall_[i] = weights_wall_[i] + update_weight;
                        std::discrete_distribution<int> updated_dist_wall(weights_wall_.begin(), weights_wall_.end());
                        distribution_prob_wall_ = updated_dist_wall;
                        found_wall = true;
                        break;
                    }
                }
                auto it = max_element(std::begin(weights_yaw_), std::end(weights_yaw_));
                if (*it > 5000)
                {
                    initializeDistribution();
                }
                //std::cout <<" Weights " << weights_wall_[0] << " " << weights_wall_[1] << " " << weights_wall_[2] << " " << weights_wall_[3] << " " << weights_wall_[4] << " " << weights_wall_[5] << " " << weights_wall_[6] << " " << weights_wall_[7] << " " << weights_wall_[8] << " " << weights_wall_[9] << "\n";
                //std::cout << "RRT distribution " << i << distribution_prob_wall_.probabilities()[0] << " " << distribution_prob_wall_.probabilities()[1] << " " << distribution_prob_wall_.probabilities()[2] << " " << distribution_prob_wall_.probabilities()[3] << " " << distribution_prob_wall_.probabilities()[4] << " " << distribution_prob_wall_.probabilities()[5] << " " << distribution_prob_wall_.probabilities()[6] << " " << distribution_prob_wall_.probabilities()[7] << " " << distribution_prob_wall_.probabilities()[8] << " " << distribution_prob_wall_.probabilities()[9] << "\n";
            }
        }

        bool InformedRRTStar::rewireToBestParent(TrajectorySegment *segment,
                                                 const std::vector<TrajectorySegment *> &candidates, bool force_rewire)
        {
            // Evaluate all candidate parents and store the best one in the segment
            // Goal is the end point of the segment

            EigenTrajectoryPoint goal_point = segment->trajectory.back();
            // store the initial segment
            TrajectorySegment best_segment = segment->shallowCopy();
            TrajectorySegment *initial_parent = segment->parent;

            // Find best segment
            for (int i = 0; i < candidates.size(); ++i)
            {
                segment->trajectory.clear();
                segment->parent = candidates[i];
                if (connectPoses(candidates[i]->trajectory.back(), goal_point,
                                 &(segment->trajectory)))
                {
                    // Feasible connection: evaluate the trajectory
                    // planner_.getTrajectoryEvaluator().computeCost(segment);
                    // planner_.getTrajectoryEvaluator().computeValue(segment);
                    // segment->tg_visited = true;
                    if (best_segment.parent == nullptr || force_rewire ||
                        segment->gain > best_segment.gain)
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
                //std::cout << "Apply best segment and rewire"
                //          << "\n";
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
                    //std::cout << "Found new parent" << "\n";
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
                            //std::cout << " Move from existing parent"
                            //          << "\n";
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

        bool InformedRRTStar::getSample(Eigen::Vector3d *sampled_position, double *sampled_yaw, double *current_yaw)
        {

            if (new_global_goal_ || !wall_detected_)
            {
                //std::cout << "Got new goal\n";
                local_volume_min_[0] = current_pose_[0] + bounding_volume_->x_min;
                local_volume_max_[0] = current_pose_[0] + bounding_volume_->x_max;
                local_volume_min_[1] = current_pose_[1] + bounding_volume_->y_min;
                local_volume_max_[1] = current_pose_[1] + bounding_volume_->y_max;
            }
            else
            {
                if (wall_direction_ == 0)
                {
                    //std::cout << "wall 0\n";
                    local_volume_min_[0] = observed_bounding_volume_[0][0]; //-2.0; //(*current_pose_)[0] - bounding_volume_->x_min;
                    local_volume_max_[0] = observed_bounding_volume_[0][1]; //2.0;  //(*current_pose_)[0] + bounding_volume_->x_min;
                    local_volume_min_[1] = current_pose_[1] + bounding_volume_->y_min;
                    local_volume_max_[1] = current_pose_[1] + bounding_volume_->y_max;
                }
                else
                {
                    //std::cout << "wall 1\n";
                    local_volume_min_[0] = current_pose_[0] + bounding_volume_->x_min;
                    local_volume_max_[0] = current_pose_[0] + bounding_volume_->x_max;
                    local_volume_min_[1] = observed_bounding_volume_[1][0];
                    local_volume_max_[1] = observed_bounding_volume_[1][1];
                }
            }

            //Set the yaw towards the goal
            if (new_global_goal_)
            {
                Eigen::Vector2d direction_vector{global_goal_(0) - current_pose_[0], global_goal_(1) - current_pose_[1]};
                yaw_to_goal_ = atan((direction_vector(1) - 0.0) / (direction_vector(0) - 1.0));
                if ((direction_vector(0) - 1.0) < 1e-4 && (direction_vector(1) - 0.0) > 1e-4)
                {
                    yaw_to_goal_ += M_PI;
                }
                if ((direction_vector(0) - 1.0) < 1e-4 && (direction_vector(1) - 0.0) < 1e-4)
                {
                    yaw_to_goal_ -= M_PI;
                }
            }

            std::cout << "Wall direction " << wall_direction_ << "wall detected " << wall_detected_ << "Local volume x : " << local_volume_min_[0] << " " << local_volume_max_[0] << " " << local_volume_min_[1] << " " << local_volume_max_[1] << "\n";

            z_min_ = bounding_volume_->z_min;
            z_max_ = bounding_volume_->z_max;
            yaw_min_ = -p_yaw_range_;
            yaw_max_ = p_yaw_range_;

            //std::cout << "Yaw to goal " << yaw_to_goal_ << "current yaw " << *current_yaw << "\n";
            if (p_apply_prob_dist_)
            {

                double prob_wall_direction = distribution_prob_wall_(generator);
                double prob_z = distribution_prob_z_(generator);
                double prob_yaw = distribution_prob_yaw_(generator);

                (*sampled_position)[wall_direction_] = local_volume_min_[wall_direction_] + (prob_wall_direction / p_prob_divisions_) * (local_volume_max_[wall_direction_] - local_volume_min_[wall_direction_]);
                (*sampled_position)[!wall_direction_] = local_volume_min_[!wall_direction_] + (double)rand() / RAND_MAX * (local_volume_max_[!wall_direction_] - local_volume_min_[!wall_direction_]);
                //(*sampled_position)[2] = z_min_ + (prob_z / p_prob_divisions_) * (z_max_ - z_min_);
                (*sampled_position)[2] = z_min_ + (double)rand() / RAND_MAX * (z_max_ - z_min_);
                //*sampled_yaw = (yaw_to_goal - p_yaw_range_) + ((double)rand() / RAND_MAX) * (2 * p_yaw_range_);
                *sampled_yaw = yaw_to_goal_;
                //*sampled_yaw = yaw_min_ + (prob_yaw / p_prob_divisions_) * (yaw_max_ - yaw_min_);
                if (*sampled_yaw > M_PI)
                {
                    *sampled_yaw = M_PI;
                }
            }
            else
            {

                (*sampled_position)[0] = local_volume_min_[0] + (double)rand() / RAND_MAX * (local_volume_max_[0] - local_volume_min_[0]);
                (*sampled_position)[1] = local_volume_min_[1] + (double)rand() / RAND_MAX * (local_volume_max_[1] - local_volume_min_[1]); //y_min + (double)rand() / RAND_MAX * (y_max - y_min);
                (*sampled_position)[2] = z_min_ + (double)rand() / RAND_MAX * (z_max_ - z_min_);
                //*sampled_yaw = (yaw_to_goal - p_yaw_range_) + ((double)rand() / RAND_MAX) * (2 * p_yaw_range_);
                *sampled_yaw = yaw_to_goal_;
                if (*sampled_yaw > M_PI)
                {
                    *sampled_yaw = M_PI;
                }
            }

            //std::cout << "Got sample " << *sampled_yaw << "\n";
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

        bool InformedRRTStar::rewireIntermediate(TrajectorySegment *root)
        {
            // After updating, try rewire all from inside out
            std::vector<TrajectorySegment *> segments;

            // order w.r.t distance
            root->getTree(&segments);
            std::vector<std::pair<double, TrajectorySegment *>> distance_pairs;

            for (int i = 1; i < segments.size(); ++i)
            {
                distance_pairs.push_back(
                    std::make_pair((segments[i]->trajectory.back().position_W -
                                    root->trajectory.back().position_W)
                                       .norm(),
                                   segments[i]));
            }
            std::sort(distance_pairs.begin(), distance_pairs.end());

            // rewire
            std::vector<TrajectorySegment *> candidate_parents;
            std::vector<TrajectorySegment *> safe_parents;
            for (int i = 0; i < distance_pairs.size(); ++i)
            {
                candidate_parents.clear();
                if (!getCloseNeighbours(
                        distance_pairs[i].second->trajectory.back().position_W,
                        &candidate_parents))
                {
                    continue;
                }
                safe_parents.clear();
                for (int j = 0; j < candidate_parents.size(); ++j)
                {
                    // cannot rewire to own children (loops!)
                    TrajectorySegment *current = candidate_parents[j];
                    while (true)
                    {
                        if (current)
                        {
                            if (current == distance_pairs[i].second)
                            {
                                break;
                            }
                            current = current->parent;
                        }
                        else
                        {
                            safe_parents.push_back(candidate_parents[j]);
                            break;
                        }
                    }
                }
                rewireToBestParent(distance_pairs[i].second, safe_parents);
            }
            return true;
        }
        bool InformedRRTStar::rewireRoot(TrajectorySegment *root, int *next_segment)
        {
            if (!p_rewire_root_)
            {
                return true;
            }
            if (!tree_is_reset_)
            {
                // Force reset (dangling pointers!)
                resetTree(root);
            }
            tree_is_reset_ = false;

            TrajectorySegment *next_root = root->children[*next_segment].get();

            // Try rewiring non-next segments (to keept their branches alive)
            std::vector<TrajectorySegment *> to_rewire;
            root->getChildren(&to_rewire);
            to_rewire.erase(std::remove(to_rewire.begin(), to_rewire.end(), next_root), to_rewire.end());
            bool rewired_something = true;
            while (rewired_something)
            {
                rewired_something = false;
                for (int i = 0; i < to_rewire.size(); ++i)
                {
                    if (rewireRootSingle(to_rewire[i], next_root))
                    {
                        to_rewire.erase(to_rewire.begin() + i);
                        rewired_something = true;
                        break;
                    }
                }
            }

            // If necessary (some segments would die) reinsert old root
            if (p_reinsert_root_ && to_rewire.size() > 0)
            {
                EigenTrajectoryPointVector new_trajectory;
                if ((next_root->trajectory.back().position_W - root->trajectory.back().position_W).norm() > 0.0)
                {
                    // don't reinsert zero movement nodes
                    if (connectPoses(next_root->trajectory.back(), root->trajectory.back(), &new_trajectory, false))
                    {
                        TrajectorySegment *reinserted_root = next_root->spawnChild();
                        reinserted_root->trajectory = new_trajectory;
                        // take info from old root (without value since already seen) will be
                        // discarded/updated anyways
                        reinserted_root->info = std::move(root->info);
                        planner_.getTrajectoryEvaluator().computeCost(reinserted_root, &current_pose_);
                        planner_.getTrajectoryEvaluator().computeValue(reinserted_root);
                        tree_data_.addSegment(reinserted_root);
                        kdtree_->addPoints(tree_data_.points.size() - 1, tree_data_.points.size() - 1);

                        // rewire
                        for (TrajectorySegment *segment : to_rewire)
                        {
                            for (int i = 0; i < segment->parent->children.size(); ++i)
                            {
                                if (segment->parent->children[i].get() == segment)
                                {
                                    // Move from existing parent
                                    reinserted_root->children.push_back(
                                        std::move(segment->parent->children[i]));
                                    segment->parent->children.erase(segment->parent->children.begin() + i);
                                    segment->parent = reinserted_root;
                                }
                            }
                        }
                    }
                }
            }

            // Adjust the next best value
            for (int i = 0; i < root->children.size(); ++i)
            {
                if (root->children[i].get() == next_root)
                {
                    *next_segment = i;
                    break;
                }
            }
            return true;
        }

        bool InformedRRTStar::rewireRootSingle(TrajectorySegment *segment,
                                               TrajectorySegment *new_root)
        {
            // Try rewiring a single segment
            std::vector<TrajectorySegment *> candidate_parents;
            if (!getCloseNeighbours(segment->trajectory.back().position_W, &candidate_parents))
            {
                return false;
            }

            // remove all candidates that are still connected to the root
            std::vector<TrajectorySegment *> safe_candidates;
            TrajectorySegment *current;
            for (int i = 0; i < candidate_parents.size(); ++i)
            {
                current = candidate_parents[i];
                while (current)
                {
                    if (current == new_root)
                    {
                        safe_candidates.push_back(candidate_parents[i]);
                        break;
                    }
                    current = current->parent;
                }
            }
            if (safe_candidates.empty())
            {
                return false;
            }
            // force rewire
            return rewireToBestParent(segment, safe_candidates, true);
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

        bool InformedRRTStarEvaluatorAdapter::getObservedBoundingBox(std::vector<Eigen::Vector2d> *bounding_box)
        {
            return following_evaluator_->getObservedBoundingBox(bounding_box);
        }
        bool InformedRRTStarEvaluatorAdapter::computeCost(TrajectorySegment *traj_in, Eigen::Vector3d *current_position)
        {
            return following_evaluator_->computeCost(traj_in, current_position);
        }

        bool InformedRRTStarEvaluatorAdapter::computeValue(TrajectorySegment *traj_in)
        {
            return following_evaluator_->computeValue(traj_in);
        }

        int InformedRRTStarEvaluatorAdapter::selectNextBest(TrajectorySegment *traj_in)
        {
            int next = following_evaluator_->selectNextBest(traj_in);
            generator_->rewireRoot(traj_in, &next);
            return next;
        }

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
            generator_ = dynamic_cast<trajectory_generator::InformedRRTStar *>(
                planner_.getFactory().readLinkableModule("InformedRRTStarGenerator"));
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