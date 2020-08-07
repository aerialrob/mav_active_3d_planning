#!/bin/bash

# *** Params ***
n_experiments=7
n_repetitions=3

# Setting directories
data_dir="/home/bianca.bendris/Documents/A4R/Simulation_Experiments/Data/"		
config_dir="/home/bianca.bendris/Documents/A4R/Simulation_Experiments/configs/"
filename="experiment"
type=".yaml"

clear_voxblox_maps=true		# Irreversibly remove maps after evaluation to save disk space

for (( e=1; e<=n_experiments; e++ ))
do  
    target_dir=$data_dir$e
    echo "Starting experiment ${e} series of ${n_repetitions} runs at '${target_dir}'!"

    # Create dir if it doesn't exist
    if [ ! -d "$target_dir" ]; then
        mkdir $target_dir
    fi

    cfg=$config_dir$filename$e$type
    echo "Using config file ${cfg}"

    # Run the experiments
    for (( i=1; i<=n_repetitions; i++ ))
    do  
        # run experiment
        roslaunch a4r_local tunnel_experiment_series.launch data_directory:=$target_dir planner_config:=$cfg


        # evaluate
        roslaunch active_3d_planning_app_reconstruction evaluate_experiment.launch target_directory:=$target_dir method:=recent series:=false clear_voxblox_maps:=$clear_voxblox_maps evaluate:=true

        bag_dir=$(ls -td $target_dir/*  | grep "2020*" | head -1)
        echo $bag_dir
        bag_file=$bag_dir"/visualization.bag"
        echo $bag_file

        roslaunch active_3d_planning_app_reconstruction replay_and_compute_metrics.launch target_directory:=$bag_dir rosbag_file:=$bag_file
        
        echo "Remove some files..."
        rm $bag_file
        voxblox_maps=$bag_dir"/voxblox_maps"
        meshes=$bag_dir"/meshes"
        rm -r $voxblox_maps
        rm -r $meshes
        
        echo "Sleeping 1m"
        sleep 1m
    
    done

    #roslaunch mav_active_3d_planning evaluate_experiment.launch target_directory:=$target_dir series:=true 
    echo "Finished ${e} series."
done
echo "Finished all simulations!"