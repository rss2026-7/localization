#!/bin/bash
# Measures the Hz of the particle filter output topic.
#
# Usage: ./measure_hz.sh [seconds]   (default: 5 seconds)
#
# Prints the average Hz over the measurement window.

TOPIC="/pf/pose/odom"
DURATION="${1:-5}"

echo "Measuring $TOPIC for ${DURATION}s..."
echo "Current params:"
ros2 param get /particle_filter num_particles 2>/dev/null
ros2 param get /particle_filter num_beams_per_particle 2>/dev/null
echo "---"

# ros2 topic hz prints stats every second; collect for DURATION then summarize
timeout "$DURATION" ros2 topic hz "$TOPIC" --window 100 2>&1 | tail -1
