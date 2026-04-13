#!/usr/bin/env bash
# sweep.sh — runs full_sim.launch.xml for every (num_particles, num_beams) combo.
#
# Usage:  bash sweep.sh

BEAMS=(100 200 300 400)
PARTICLES=(200 250 300 350 400)

echo "============================================================"
echo "  PF parameter sweep"
echo "  beams:     ${BEAMS[*]}"
echo "  particles: ${PARTICLES[*]}"
echo "  $(( ${#BEAMS[@]} * ${#PARTICLES[@]} )) trials total"
echo "============================================================"

for beams in "${BEAMS[@]}"; do
    for particles in "${PARTICLES[@]}"; do
        echo ""
        echo "------------------------------------------------------------"
        echo "  Trial: num_beams_per_particle=$beams  num_particles=$particles"
        echo "------------------------------------------------------------"

        # setsid gives the launch its own process group so killing it
        # doesn't take out this script
        setsid ros2 launch localization full_sim.launch.xml \
            num_beams_per_particle:="$beams" \
            num_particles:="$particles" &
        LAUNCH_PID=$!

        echo "  Waiting for ground_truth_publisher to start..."
        for i in $(seq 1 30); do
            sleep 1
            ros2 node list 2>/dev/null | grep -q ground_truth_publisher && break
        done

        echo "  Running (10 s data collection)..."
        for i in $(seq 1 30); do
            sleep 1
            ros2 node list 2>/dev/null | grep -q ground_truth_publisher || break
        done

        sleep 1  # let output flush

        # Kill the launch process; ros2 launch propagates SIGTERM to all children
        kill "$LAUNCH_PID" 2>/dev/null
        wait "$LAUNCH_PID" 2>/dev/null

        echo "  Trial complete."
        sleep 3
    done
done

echo ""
echo "============================================================"
echo "  Sweep complete. CSV files: $(pwd)/pf_error_*.csv"
echo "============================================================"
