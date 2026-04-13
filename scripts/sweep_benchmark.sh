#!/bin/bash
# Sweeps through combinations of num_particles and num_beams, measuring Hz for each.
#
# Usage: ./sweep_benchmark.sh
#
# Prerequisites: particle filter must be running and initialized (click initial pose in RViz).
# Also needs the car to be receiving /scan and /odom data (sim or real).

NODE="/particle_filter"
TOPIC="/pf/pose/odom"
DURATION=8  # seconds per measurement
TMPFILE=$(mktemp /tmp/hz_bench.XXXXXX)

PARTICLES=(200 250 300 350 400)
BEAMS=(100 150 200 400)

echo "particles,beams,hz"

for p in "${PARTICLES[@]}"; do
    for b in "${BEAMS[@]}"; do
        ros2 param set $NODE num_particles "$p" > /dev/null 2>&1
        ros2 param set $NODE num_beams_per_particle "$b" > /dev/null 2>&1
        sleep 2  # let it stabilize after param change

        # Write hz output to a temp file to avoid losing data when timeout kills the process
        timeout "$DURATION" ros2 topic hz "$TOPIC" --window 50 > "$TMPFILE" 2>&1

        hz=$(grep "average rate" "$TMPFILE" | tail -1 | grep -oP '[\d.]+' | head -1)
        hz=${hz:-0}

        echo "$p,$b,$hz"
    done
done

rm -f "$TMPFILE"
