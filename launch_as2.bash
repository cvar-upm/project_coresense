#!/bin/bash

usage() {
    echo "  options:"
    echo "      -c: motion controller plugin (pid_speed_controller, differential_flatness_controller), choices: [pid, df]. Default: pid"
    echo "      -w: world config file. Default: config/world.yaml"
    echo "      -n: select drones namespace to launch, values are comma separated. By default, it will get all drones from world description file"
    echo "      -r: use real hardware tmuxinator config (aerostack2_real.yaml). Default: simulation (aerostack2.yaml)"
    echo "      -g: launch using gnome-terminal instead of tmux. Default not set"
}

# Initialize variables with default values
motion_controller_plugin="pid"
simulation_config="config/world.yaml"
drones_namespace_comma=""
use_gnome="false"
tmuxinator_config="tmuxinator/aerostack2.yaml"

# Arg parser
while getopts "cw:n:rg" opt; do
  case ${opt} in
    c )
      motion_controller_plugin="${OPTARG}"
      ;;
    w )
      simulation_config="${OPTARG}"
      ;;
    n )
      drones_namespace_comma="${OPTARG}"
      ;;
    r )
      tmuxinator_config="tmuxinator/aerostack2_real.yaml"
      ;;
    g )
      use_gnome="true"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[wrt]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

# If no drone namespaces are provided, get them from the world description config file
if [ -z "$drones_namespace_comma" ]; then
  drones_namespace_comma=$(python3 utils/get_drones.py -p ${simulation_config} --sep ',')
fi
IFS=',' read -r -a drone_namespaces <<< "$drones_namespace_comma"

# Check if motion controller plugins are valid
case ${motion_controller_plugin} in
  pid )
    motion_controller_plugin="pid_speed_controller"
    ;;
  df )
    motion_controller_plugin="differential_flatness_controller"
    ;;
  * )
    echo "Invalid motion controller plugin: ${motion_controller_plugin}" >&2
    usage
    exit 1
    ;;
esac

# Select between tmux and gnome-terminal
tmuxinator_mode="start"
tmuxinator_end="wait"
tmp_file="/tmp/as2_project_launch_${drone_namespaces[@]}.txt"
if [[ ${use_gnome} == "true" ]]; then
  tmuxinator_mode="debug"
  tmuxinator_end="> ${tmp_file} && python3 utils/tmuxinator_to_genome.py -p ${tmp_file} && wait"
fi

# Launch aerostack2 for each drone namespace
for namespace in ${drone_namespaces[@]}; do
  base_launch="false"
  if [[ ${namespace} == ${drone_namespaces[0]} ]]; then
    base_launch="true"
  fi
  eval "tmuxinator ${tmuxinator_mode} -n ${namespace} -p ${tmuxinator_config} \
    drone_namespace=${namespace} \
    simulation_config_file=${simulation_config} \
    motion_controller_plugin=${motion_controller_plugin} \
    base_launch=${base_launch} \
    ${tmuxinator_end}"

  sleep 0.1 # Wait for tmuxinator to finish
done

# Attach to tmux session
if [[ ${use_gnome} == "false" ]]; then
  tmux attach-session -t ${drone_namespaces[0]}
# If tmp_file exists, remove it
elif [[ -f ${tmp_file} ]]; then
  rm ${tmp_file}
fi
