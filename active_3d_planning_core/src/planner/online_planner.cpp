#define _USE_MATH_DEFINES

#include "active_3d_planning_core/planner/online_planner.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <stdio.h>

namespace active_3d_planning
{

    OnlinePlanner::OnlinePlanner(ModuleFactory *factory, Module::ParamMap *param_map)
        : factory_(factory), running_(false), planning_(false)
    {
        // Setup params
        OnlinePlanner::setupFromParamMap(param_map);
    }

    OnlinePlanner::~OnlinePlanner()
    {
        // shutdown filestream
        if (perf_log_file_.is_open())
        {
            perf_log_file_.close();
        }
        if (traj_log_file_.is_open())
        {
            traj_log_file_.close();
        }
    }

    void OnlinePlanner::setupFromParamMap(Module::ParamMap *param_map)
    {
        // Setup Params
        bool verbose_modules;
        bool build_modules_on_init;

        // Logging, printing and visualization
        setParam<bool>(param_map, "verbose", &p_verbose_, true);
        setParam<bool>(param_map, "verbose_modules", &verbose_modules, true);
        setVerbose(verbose_modules);
        setParam<bool>(param_map, "build_modules_on_init", &build_modules_on_init, false);
        setParam<bool>(param_map, "visualize", &p_visualize_, true);
        setParam<bool>(param_map, "log_performance", &p_log_performance_, false);
        setParam<bool>(param_map, "visualize_gain", &p_visualize_gain_, false);
        setParam<bool>(param_map, "highlight_executed_trajectory", &p_highlight_executed_trajectory_, false);

        // Sampling constraints
        setParam<int>(param_map, "max_new_segments", &p_max_new_segments_, 0); // set 0 for infinite
        setParam<int>(param_map, "min_new_segments", &p_min_new_segments_, 0);
        setParam<int>(param_map, "max_new_tries", &p_max_new_tries_, 0); // set 0 for infinite
        setParam<int>(param_map, "min_new_tries", &p_min_new_tries_, 0);
        setParam<double>(param_map, "min_new_value", &p_min_new_value_, 0.0);
        setParam<int>(param_map, "expand_batch", &p_expand_batch_, 1);

        setParam<bool>(param_map, "manual", &p_manual_, false);
        setParam<std::string>(param_map, "wp_file", &p_wp_file_, "none");

        p_expand_batch_ = std::max(p_expand_batch_, 1);

        // Setup members
        std::string args; // default args extends the parent namespace
        std::string param_ns = (*param_map)["param_namespace"];
        setParam<std::string>(param_map, "system_constraint_args", &args,
                              param_ns + "/system_constraints");
        system_constraints_ = getFactory().createModule<SystemConstraints>(args, *this, verbose_modules);
        setParam<std::string>(param_map, "map_args", &args,
                              param_ns + "/map");
        map_ = getFactory().createModule<Map>(args, *this, verbose_modules);
        tsdf_map_ = dynamic_cast<map::TSDFMap *>(&(getMap()));
        setParam<std::string>(param_map, "trajectory_generator_args", &args,
                              param_ns + "/trajectory_generator");
        trajectory_generator_ = getFactory().createModule<TrajectoryGenerator>(
            args, *this, verbose_modules);
        setParam<std::string>(param_map, "trajectory_evaluator_args", &args,
                              param_ns + "/trajectory_evaluator");
        trajectory_evaluator_ = getFactory().createModule<TrajectoryEvaluator>(
            args, *this, verbose_modules);
        setParam<std::string>(param_map, "back_tracker_args", &args,
                              param_ns + "/back_tracker");
        back_tracker_ = getFactory().createModule<BackTracker>(
            args, *this, verbose_modules);

        // Setup performance log
        if (p_log_performance_)
        {
            std::string log_dir("");
            setParam<std::string>(param_map, "performance_log_dir", &log_dir, "");
            if (log_dir == "")
            {
                // Wait for this param until planning start so it can be set after
                // constructor (e.g. from other nodes)
                printWarning("No directory for performance log set, performance log turned off.");
                p_log_performance_ = false;
            }
            else
            {
                logfile_ = log_dir + "/performance_log.csv";
                perf_log_file_.open(logfile_.c_str());
                perf_log_data_ = std::vector<double>(5, 0.0); // select, expand, gain, cost, value
                perf_cpu_timer_ = std::clock();
                perf_log_file_ << "RunTime,NTrajectories,NTrajAfterUpdate,Select,Expand,Gain,"
                                  "Cost,Value,NextBest,UpdateTG,UpdateTE,Visualization,Total";

                trajfile_ = log_dir + "/trajectory_log.csv";
                traj_log_file_.open(trajfile_.c_str());
                traj_log_data_ = std::vector<double>(3, 0.0); //gain, cost, value
                traj_log_file_ << "RunTime,Gain,Cost,Value,VoxelsMapped,VoxelsSensed,Percentage";
            }
        }

        // Force lazy initialization of modules (call every function once)
        if (build_modules_on_init)
        {
            // Empty set of arguments required to run everything
            TrajectorySegment temp_segment;
            TrajectorySegment *temp_pointer;
            std::vector<TrajectorySegment *> temp_vector;
            EigenTrajectoryPoint trajectory_point;
            trajectory_point.position_W = Eigen::Vector3d(0, 0, 0);
            trajectory_point.setFromYaw(0);
            temp_segment.trajectory.push_back(trajectory_point);
            temp_segment.spawnChild()->trajectory.push_back(trajectory_point);
            std::vector<Eigen::Vector2d> observed_bounding_box;
            std::cout << "get bounding box\n";
            trajectory_evaluator_->getObservedBoundingBox(&observed_bounding_box);
            std::cout << "get bounding box\n";
            trajectory_generator_->setObservedBoundingBox(&observed_bounding_box);
            std::cout << "set bounding box\n";
            trajectory_generator_->selectSegment(&temp_pointer, &temp_segment);
            std::cout << "select bounding box\n";
            trajectory_generator_->expandSegment(temp_pointer, &temp_vector);
            std::cout << "expand bounding box\n";
            trajectory_generator_->updateSegment(&temp_segment);

            temp_segment = TrajectorySegment();
            temp_segment.trajectory.push_back(trajectory_point);
            trajectory_evaluator_->computeGain(&temp_segment);
            //Eigen::Vector3d pos{0.0, 0.0, 0.0};
            trajectory_evaluator_->computeCost(&temp_segment, &current_position_);
            trajectory_evaluator_->computeValue(&temp_segment);
            trajectory_evaluator_->updateSegment(&temp_segment);

            temp_segment.spawnChild()->trajectory.push_back(trajectory_point);
            trajectory_evaluator_->selectNextBest(&temp_segment);
        }
    }

    // logging and printing: default to std::cout in worst case
    void OnlinePlanner::printInfo(const std::string &text)
    {
        std::cout << "Info: " << text << std::endl;
    }

    void OnlinePlanner::printWarning(const std::string &text)
    {
        std::cout << "Warning: " << text << std::endl;
    }

    void OnlinePlanner::printError(const std::string &text)
    {
        std::cout << "Error: " << text << std::endl;
    }

    void OnlinePlanner::initializePlanning()
    {
        // Setup initial trajectory Segment at current location
        target_position_ = current_position_;
        target_yaw_ = yawFromQuaternion(current_orientation_);
        current_segment_ = std::unique_ptr<TrajectorySegment>(new TrajectorySegment());
        EigenTrajectoryPoint trajectory_point;
        trajectory_point.position_W = target_position_;
        trajectory_point.setFromYaw(target_yaw_);
        current_segment_->trajectory.push_back(trajectory_point);

        // Setup counters
        info_timing_ = std::clock();
        info_count_ = 1;
        info_killed_next_ = 0;
        info_killed_update_ = 0;
        new_segments_ = 0;
        new_segment_tries_ = 0;
        min_new_value_reached_ = p_min_new_value_ == 0.0;
        vis_completed_count_ = 0;
        std::fill(perf_log_data_.begin(), perf_log_data_.begin() + 5, 0.0);

        // Launch expansion as current goal is considered reached
        target_reached_ = true;

        if (p_manual_)
        {
            loadWPs();
        }
    }

    bool OnlinePlanner::loadWPs()
    {

        std::fstream file;
        file.open(p_wp_file_, std::fstream::in);
        if (!file.is_open())
        {
            LOG(ERROR) << "Could not open file to load graph points: " << p_wp_file_;
            return false;
        }
        else
        {
            std::string line;
            while (getline(file, line))
            {
                std::istringstream line_stream(line);

                double x, y, z, qx, qy, qz, qw;
                line_stream >> x >> y >> z >> qx >> qy >> qz >> qw;
                EigenTrajectoryPoint point;
                point.position_W = Eigen::Vector3d{x, y, z};
                point.orientation_W_B = Eigen::Quaterniond{qw, qx, qy, qz};
                wp_list_.push_back(point);
            }
        }
        wp_id_ = 0;
    }

    void OnlinePlanner::planningLoop()
    {
        // This is the main loop, adjust updating and implement exit conditions
        running_ = true;
        while (running_)
        {
            if (planning_)
            {
                loopIteration();
            }
            // [check update /exit conditions here]
        }
    }

    void OnlinePlanner::loopIteration()
    {
        // Continuosly expand the trajectory space
        //std::cout << "Starting loop \n";
        for (int i = 0; i < p_expand_batch_; ++i)
        {
            expandTrajectories();
        }
        //std::cout << "Loop Iteration expan trajectories\n";
        if (!min_new_value_reached_)
        {
            //  recursively check whether the minimum value is reached
            min_new_value_reached_ = checkMinNewValue(current_segment_);
        }
        //std::cout << "Target reached " << target_reached_ << "\n";
        // After finishing the current segment, execute the next one
        if (target_reached_)
        {
            wp_id_++;
            if (new_segment_tries_ >= p_max_new_tries_ && p_max_new_tries_ > 0)
            {
                // Maximum tries reached: force next segment
                requestNextTrajectory();
            }
            else
            {
                if (new_segment_tries_ < p_min_new_tries_)
                {
                    return; // check minimum tries reached
                }
                if (new_segments_ < p_min_new_segments_)
                {
                    return; // check minimum successful expansions reached
                }
                if (!min_new_value_reached_)
                {
                    return; // check minimum value reached
                }
                // All requirements met
                requestNextTrajectory();
            }
        }
        //std::cout << "FinishedLoop \n";
    }

    bool OnlinePlanner::requestNextTrajectory()
    {
        std::int16_t planner_status;
        if (current_segment_->children.empty())
        {
            std::cout << "No trajectories available: call the backtracker \n";
            back_tracker_->trackBack(current_segment_.get());
            return false;
        }

        //std::cout << "[Online planner] request next traj \n";
        // Performance tracking
        double perf_runtime;
        double perf_vis = 0.0;
        double perf_next;
        double perf_uptg;
        double perf_upte;
        std::clock_t timer;
        if (p_log_performance_)
        {
            timer = std::clock();
        }

        // Visualize candidates
        std::vector<TrajectorySegment *> trajectories_to_vis;
        current_segment_->getTree(&trajectories_to_vis);
        trajectories_to_vis.erase(
            trajectories_to_vis.begin()); // remove current segment (root)
        if (p_visualize_)
        {
            publishTrajectoryVisualization(trajectories_to_vis);
        }
        int num_trajectories = trajectories_to_vis.size();
        if (p_verbose_ || p_log_performance_)
        {
            perf_runtime = std::chrono::duration<double>(std::clock() - info_timing_).count() / 1e6;
            info_timing_ = std::clock();
            if (p_verbose_)
            {
                std::stringstream ss;
                ss << "Replanning!\n(" << std::setprecision(3) << perf_runtime << "s elapsed, "
                   << num_trajectories - info_count_ + 1 << " new, " << num_trajectories << " total, "
                   << info_killed_next_ + 1 << " killed by root change, " << info_killed_update_
                   << " killed while updating).";
                printInfo(ss.str());
            }
            if (num_trajectories < 2 && perf_runtime > 50)
            {
                planner_status = 0;
            }
            else
            {
                planner_status = 1;
            }
            publishStatus(planner_status);
        }
        if (p_log_performance_)
        {
            perf_vis += (double)(std::clock() - timer) / CLOCKS_PER_SEC;
            timer = std::clock();
        }
        // std::cout << "[Online planner] vizualize traj \n";
        // Select best next trajectory and update root
        int next_segment = trajectory_evaluator_->selectNextBest(current_segment_.get());
        current_segment_ = std::move(current_segment_->children[next_segment]);
        Eigen::Vector3d sensed_info;
        tsdf_map_->getSensedInfo(&sensed_info);
        std::stringstream segment_info;
        segment_info << "Next best segment: gain " << current_segment_->gain << " cost: "
                     << current_segment_->cost << " value: " << current_segment_->value
                     << " percentage sensed: " << sensed_info[2];
        printInfo(segment_info.str());

        traj_log_data_ = {current_segment_->gain, current_segment_->cost, current_segment_->value};
        current_segment_->parent = nullptr;
        current_segment_->gain = 0.0;
        current_segment_->cost = 0.0;
        current_segment_->value = 0.0;
        trajectory_generator_->new_global_goal_ = false;
        if (p_log_performance_)
        {
            perf_next = (double)(std::clock() - timer) / CLOCKS_PER_SEC;
        }
        trajectories_to_vis.clear();
        current_segment_->getTree(&trajectories_to_vis);
        //std::cout << "[Online planner] current segment get tree \n";
        info_killed_next_ = num_trajectories - trajectories_to_vis.size();

        // Move
        EigenTrajectoryPointVector trajectory;
        trajectory_generator_->extractTrajectoryToPublish(&trajectory, *current_segment_);
        current_segment_->trajectory = trajectory;
        requestMovement(trajectory);
        target_position_ = trajectory.back().position_W;
        target_yaw_ = trajectory.back().getYaw();
        back_tracker_->segmentIsExecuted(*current_segment_);
        //std::cout << "[Online planner] move \n";
        // Visualize
        if (p_log_performance_)
        {
            timer = std::clock();
        }
        if (p_visualize_)
        {
            publishEvalVisualization(*current_segment_);
            publishCompletedTrajectoryVisualization(*current_segment_);
        }
        if (p_log_performance_)
        {
            perf_vis += (double)(std::clock() - timer) / CLOCKS_PER_SEC;
            timer = std::clock();
        }

        // Update tree
        // recursive tree pass from root to leaves
        updateGeneratorStep(current_segment_.get());
        //std::cout << "[Online planner] updateGeneratorStep \n";
        if (p_log_performance_)
        {
            perf_uptg = (double)(std::clock() - timer) / CLOCKS_PER_SEC;
            timer = std::clock();
        }
        updateEvaluatorStep(current_segment_.get());
        //std::cout << "[Online planner] updateEvaluatorStep \n";
        if (p_log_performance_)
        {
            perf_upte = (double)(std::clock() - timer) / CLOCKS_PER_SEC;
        }
        trajectories_to_vis.clear();
        current_segment_->getTree(&trajectories_to_vis);
        info_killed_update_ = num_trajectories - trajectories_to_vis.size() - info_killed_next_;

        // Performance log and printing
        if (p_verbose_ || p_log_performance_)
        {
            trajectories_to_vis.clear();
            current_segment_->getTree(&trajectories_to_vis);
            info_count_ = trajectories_to_vis.size();

            if (p_log_performance_)
            {
                perf_log_file_ << "\n"
                               << perf_runtime << "," << num_trajectories << ","
                               << info_count_ << "," << perf_log_data_[0] << ","
                               << perf_log_data_[1] << "," << perf_log_data_[2] << ","
                               << perf_log_data_[3] << "," << perf_log_data_[4] << ","
                               << perf_next << "," << perf_uptg << "," << perf_upte << ","
                               << perf_vis << "," << (double)(std::clock() - perf_cpu_timer_) / CLOCKS_PER_SEC;
                std::fill(perf_log_data_.begin(), perf_log_data_.begin() + 5, 0.0); // reset count
                //perf_cpu_timer_ = std::clock();
                Eigen::Vector3d sensed_info;
                tsdf_map_->getSensedInfo(&sensed_info);
                traj_log_file_ << "\n"
                               << perf_runtime << "," << traj_log_data_[0] << ","
                               << traj_log_data_[1] << "," << traj_log_data_[2] << ","
                               << sensed_info[0] << "," << sensed_info[1] << ","
                               << sensed_info[2];
                perf_cpu_timer_ = std::clock();
            }
        }

        // Update tracking values
        new_segment_tries_ = 0;
        new_segments_ = 0;
        min_new_value_reached_ = p_min_new_value_ == 0.0;
        target_reached_ = false;
        return true;
    }

    void OnlinePlanner::expandTrajectories()
    {
        // Check max number of tries already and min value found
        if (p_max_new_segments_ > 0 && new_segments_ >= p_max_new_segments_ &&
            min_new_value_reached_)
        {
            return;
        }

        // Select expansion target
        std::clock_t timer;
        if (p_log_performance_)
        {
            timer = std::clock();
        }
        TrajectorySegment *expansion_target;
        if (p_manual_)
        {
            expansion_target = current_segment_.get();
        }
        else
        {
            std::vector<Eigen::Vector2d> observed_bounding_box;
            trajectory_evaluator_->getObservedBoundingBox(&observed_bounding_box);
            trajectory_generator_->setObservedBoundingBox(&observed_bounding_box);
            trajectory_generator_->selectSegment(&expansion_target, current_segment_.get());
        }
        if (p_log_performance_)
        {
            perf_log_data_[0] += (double)(std::clock() - timer) / CLOCKS_PER_SEC;
            timer = std::clock();
        }

        // Expand the target
        std::vector<TrajectorySegment *> created_segments;
        bool success;
        if (p_manual_)
        {
            if (wp_id_ < wp_list_.size())
            {
                TrajectorySegment *new_segment;
                new_segment = expansion_target->spawnChild();
                EigenTrajectoryPointVector trajectory;
                EigenTrajectoryPoint start_point = expansion_target->trajectory.back();
                EigenTrajectoryPoint goal_point = wp_list_[wp_id_];

                //Compute Trajectory
                Eigen::Vector3d start_pos = start_point.position_W;
                Eigen::Vector3d direction = goal_point.position_W - start_pos;
                double p_sampling_rate_ = 20.0;
                int n_points = std::ceil(direction.norm() / this->getSystemConstraints().v_max * p_sampling_rate_);

                // Build trajectory
                n_points = std::ceil(direction.norm() / this->getSystemConstraints().v_max *
                                     p_sampling_rate_);
                for (int i = 0; i < n_points; ++i)
                {
                    EigenTrajectoryPoint trajectory_point;
                    trajectory_point.position_W =
                        start_pos + (double)i / (double)n_points * direction;
                    trajectory_point.setFromYaw(goal_point.getYaw());
                    trajectory_point.time_from_start_ns =
                        static_cast<int64_t>((double)i / p_sampling_rate_ * 1.0e9);
                    trajectory.push_back(trajectory_point);
                }
                //trajectory.push_back(start_point);
                //trajectory.push_back(goal_point);
                new_segment->trajectory = trajectory;
                created_segments.push_back(new_segment);
                success = true;
            }
            else
            {
                planning_ = false;
                running_ = false;
            }
        }
        else
        {
            success = trajectory_generator_->expandSegment(expansion_target, &created_segments);
        }

        //std::cout << "Created segments " << created_segments.size() << "\n";
        if (p_log_performance_)
        {
            perf_log_data_[1] += (double)(std::clock() - timer) / CLOCKS_PER_SEC;
            timer = std::clock();
        }
        new_segment_tries_++;
        if (success)
        {
            new_segments_++;
        }
        //std::cout << "[Online planner] Segment expanded \n";
        // Evaluate newly added segments: Gain
        for (int i = 0; i < created_segments.size(); ++i)
        {
            trajectory_evaluator_->computeGain(created_segments[i]);
        }
        if (p_log_performance_)
        {
            perf_log_data_[2] += (double)(std::clock() - timer) / CLOCKS_PER_SEC;
            timer = std::clock();
        }
        //std::cout << "[Online planner] Evaluate gain \n";
        // Costs
        for (int i = 0; i < created_segments.size(); ++i)
        {
            trajectory_evaluator_->computeCost(created_segments[i], &current_position_);
        }
        if (p_log_performance_)
        {
            perf_log_data_[3] += (double)(std::clock() - timer) / CLOCKS_PER_SEC;
            timer = std::clock();
        }
        //std::cout << "[Online planner] Evaluate cost \n";
        // Final value
        for (int i = 0; i < created_segments.size(); ++i)
        {
            trajectory_evaluator_->computeValue(created_segments[i]);
        }
        if (p_log_performance_)
        {
            perf_log_data_[4] += (double)(std::clock() - timer) / CLOCKS_PER_SEC;
        }
        // std::cout << "[Online planner] Evaluate value \n";
    }

    void OnlinePlanner::publishTrajectoryVisualization(
        const std::vector<TrajectorySegment *> &trajectories)
    {
        // Display all trajectories in the input and erase previous ones
        double max_value = trajectories[0]->value;
        double min_value = trajectories[0]->value;
        double max_gain = trajectories[0]->gain;
        double min_gain = trajectories[0]->gain;
        TrajectorySegment *goal = trajectories[0];
        for (int i = 1; i < trajectories.size(); ++i)
        {
            if (trajectories[i]->value >= max_value)
            {
                max_value = trajectories[i]->value;
                goal = trajectories[i];
            }
            if (trajectories[i]->value < min_value)
            {
                min_value = trajectories[i]->value;
            }
            if (trajectories[i]->gain > max_gain)
            {
                max_gain = trajectories[i]->gain;
            }
            if (trajectories[i]->gain < min_gain)
            {
                min_gain = trajectories[i]->gain;
            }
        }

        VisualizationMarkers value_markers, gain_markers, text_markers, goal_markers;
        VisualizationMarker msg;
        for (int i = 0; i < trajectories.size(); ++i)
        {
            // Setup marker message
            msg = VisualizationMarker();
            msg.orientation.w() = 1.0;
            msg.type = VisualizationMarker::POINTS;
            msg.id = i;
            msg.ns = "candidate_trajectories";
            msg.scale.x() = 0.04;
            msg.scale.y() = 0.04;
            msg.color.a = 0.4;
            msg.action = VisualizationMarker::OVERWRITE;

            // Color according to relative value (blue when indifferent)
            if (max_value != min_value)
            {
                double frac =
                    (trajectories[i]->value - min_value) / (max_value - min_value);
                msg.color.r = std::min((0.5 - frac) * 2.0 + 1.0, 1.0);
                msg.color.g = std::min((frac - 0.5) * 2.0 + 1.0, 1.0);
                msg.color.b = 0.0;
                if (trajectories[i]->value == max_value)
                {
                    goal = trajectories[i];
                }
            }
            else
            {
                msg.color.r = 0.3;
                msg.color.g = 0.3;
                msg.color.b = 1.0;
            }

            // points
            for (int j = 0; j < trajectories[i]->trajectory.size(); ++j)
            {
                msg.points.push_back(trajectories[i]->trajectory[j].position_W);
            }
            value_markers.addMarker(msg);

            // visualize gain
            if (p_visualize_gain_)
            {
                msg = VisualizationMarker();
                msg.orientation.w() = 1.0;
                msg.type = VisualizationMarker::SPHERE;
                msg.action = VisualizationMarker::OVERWRITE;
                msg.id = i;
                msg.ns = "canidate_gains";
                msg.scale.x() = 0.15;
                msg.scale.y() = 0.15;
                msg.scale.z() = 0.15;
                msg.position = trajectories[i]->trajectory.back().position_W;

                // Color according to relative value (blue when indifferent)
                if (trajectories[i]->value < 0)
                {
                    msg.color.r = 0.5;
                    msg.color.g = 0.0;
                    msg.color.b = 0.5;
                }
                else
                {
                    if (min_value != max_value)
                    {
                        double frac = (trajectories[i]->value - min_value) / (max_value - min_value);
                        msg.color.r = std::min((0.5 - frac) * 2.0 + 1.0, 1.0);
                        msg.color.g = std::min((frac - 0.5) * 2.0 + 1.0, 1.0);
                        msg.color.b = 0.0;
                    }
                    else
                    {
                        msg.color.r = 0.3;
                        msg.color.g = 0.3;
                        msg.color.b = 1.0;
                    }
                }

                msg.color.a = 1.0;
                gain_markers.addMarker(msg);
            }

            // Text
            msg = VisualizationMarker();
            msg.type = VisualizationMarker::TEXT_VIEW_FACING;
            msg.id = i;
            msg.ns = "candidate_text";
            msg.scale.x() = 0.2;
            msg.scale.y() = 0.2;
            msg.scale.z() = 0.2;
            msg.color.r = 0.0f;
            msg.color.g = 0.0f;
            msg.color.b = 0.0f;
            msg.color.a = 1.0;
            msg.position = trajectories[i]->trajectory.back().position_W;
            std::stringstream stream;
            stream << std::fixed << std::setprecision(2) << trajectories[i]->gain << "/"
                   << std::fixed << std::setprecision(2) << trajectories[i]->cost << "/"
                   << std::fixed << std::setprecision(2) << trajectories[i]->value;
            msg.text = stream.str();
            msg.action = VisualizationMarker::OVERWRITE;
            text_markers.addMarker(msg);
        }
        // visualize goal
        msg = VisualizationMarker();
        msg.orientation.w() = 1.0;
        msg.type = VisualizationMarker::POINTS;
        msg.id = 0;
        msg.ns = "current_goal";
        msg.scale.x() = 0.1;
        msg.scale.y() = 0.1;
        msg.color.r = 0.0;
        msg.color.g = 0.9;
        msg.color.b = 0.0;
        msg.color.a = 1.0;
        msg.action = VisualizationMarker::OVERWRITE;
        while (goal->parent)
        {
            // points
            if (goal->parent->parent)
            {
                for (int j = 0; j < goal->trajectory.size(); ++j)
                {
                    msg.points.push_back(goal->trajectory[j].position_W);
                }
            }
            goal = goal->parent;
        }
        goal_markers.addMarker(msg);

        // visualize
        publishVisualization(value_markers);
        publishVisualization(gain_markers);
        publishVisualization(text_markers);
        publishVisualization(goal_markers);
    }

    void OnlinePlanner::publishCompletedTrajectoryVisualization(
        const TrajectorySegment &trajectories)
    {
        // Continuously increment the already traveled path
        VisualizationMarker msg;
        msg.orientation.w() = 1.0;
        msg.type = VisualizationMarker::POINTS;
        msg.ns = "completed_trajectory";
        msg.id = vis_completed_count_;
        msg.action = VisualizationMarker::ADD;
        if (p_highlight_executed_trajectory_)
        {
            msg.scale.x() = 0.1;
            msg.scale.y() = 0.1;
            msg.color.r = 1.0;
            msg.color.g = 0.0;
            msg.color.b = 0.0;
            msg.color.a = 1.0;
        }
        else
        {
            msg.scale.x() = 0.03;
            msg.scale.y() = 0.03;
            msg.color.r = 0.5;
            msg.color.g = 0.5;
            msg.color.b = 0.5;
            msg.color.a = 0.8;
        }

        // points
        for (int i = 0; i < trajectories.trajectory.size(); ++i)
        {
            msg.points.push_back(trajectories.trajectory[i].position_W);
        }
        VisualizationMarkers array_msg;
        array_msg.addMarker(msg);
        publishVisualization(array_msg);
        vis_completed_count_++;
    }

    void OnlinePlanner::publishEvalVisualization(const TrajectorySegment &trajectory)
    {
        // Visualize the gain of the current segment
        VisualizationMarkers msg;
        trajectory_evaluator_->visualizeTrajectoryValue(&msg, trajectory);
        int i = 0;
        for (VisualizationMarker &marker : msg.getMarkers())
        {
            marker.id = i;
            i++;
            marker.ns = "trajectory_evaluation";
            marker.action = VisualizationMarker::OVERWRITE;
        }
        publishVisualization(msg);
    }

    bool OnlinePlanner::checkMinNewValue(
        const std::unique_ptr<TrajectorySegment> &segment)
    {
        // Recursively check whether the minimum value is reached
        if (segment->value >= p_min_new_value_)
        {
            return true;
        }
        else
        {
            for (int i = 0; i < segment->children.size(); ++i)
            {
                if (checkMinNewValue(segment->children[i]))
                {
                    return true;
                }
            }
        }
        return false;
    }

    void OnlinePlanner::updateGeneratorStep(TrajectorySegment *target)
    {
        // Recursive removal
        int j = 0;
        for (int i = 0; i < target->children.size(); ++i)
        {
            if (trajectory_generator_->updateSegment(target->children[j].get()))
            {
                j++;
            }
            else
            {
                target->children.erase(target->children.begin() + j);
            }
        }
        // remaining children
        for (int i = 0; i < target->children.size(); ++i)
        {
            updateGeneratorStep(target->children[i].get());
        }
    }

    void OnlinePlanner::updateEvaluatorStep(TrajectorySegment *target)
    {
        // Recursive removal
        int j = 0;
        for (int i = 0; i < target->children.size(); ++i)
        {
            if (trajectory_evaluator_->updateSegment(target->children[j].get()))
            {
                j++;
            }
            else
            {
                target->children.erase(target->children.begin() + j);
            }
        }
        // remaining children
        for (int i = 0; i < target->children.size(); ++i)
        {
            updateEvaluatorStep(target->children[i].get());
        }
    }

} // namespace active_3d_planning