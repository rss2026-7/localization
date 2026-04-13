#!/bin/bash
# Usage: ./set_pf_params.sh [num_particles] [num_beams]
#
# Examples:
#   ./set_pf_params.sh 100 50      # 100 particles, 50 beams
#   ./set_pf_params.sh 200         # 200 particles, keep current beams
#   ./set_pf_params.sh "" 30       # keep current particles, 30 beams

NODE="/particle_filter"

if [ -z "$1" ] && [ -z "$2" ]; then
    echo "Current parameters:"
    ros2 param get $NODE num_particles
    ros2 param get $NODE num_beams_per_particle
    echo ""
    echo "Usage: $0 [num_particles] [num_beams_per_particle]"
    exit 0
fi

if [ -n "$1" ]; then
    echo "Setting num_particles -> $1"
    ros2 param set $NODE num_particles "$1"
fi

if [ -n "$2" ]; then
    echo "Setting num_beams_per_particle -> $2"
    ros2 param set $NODE num_beams_per_particle "$2"
fi

echo ""
echo "Current values:"
ros2 param get $NODE num_particles
ros2 param get $NODE num_beams_per_particle
