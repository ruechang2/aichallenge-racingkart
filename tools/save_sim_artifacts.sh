#!/bin/bash
# Copy AWSIM's own records out of the simulator container before it is removed.
#
# Two of them are not in ./output on their own and are lost with the container:
#
#   Player.log         — the only place the *opponents'* lap crossings appear
#                        ("[NpcLap] C2 lap 1: 81.81s"), which is what lets the
#                        dashboard reconstruct our position over the race. AWSIM
#                        exposes only final_position anywhere else.
#   result-summary.json — per-lap times straight from the judge. The controller's
#                        own log reports cumulative time in a race session, so
#                        this is the authority (see dashboard/README.md).
#
# Usage:  tools/save_sim_artifacts.sh [container] [dest_dir]
# Defaults to the compose simulator container and the newest output/*/d1.

set -u
container="${1:-aichallenge-racingkart-simulator-1}"
dest="${2:-}"

if [ -z "$dest" ]; then
    dest=$(ls -dt output/*/d1 2>/dev/null | head -1)
fi
if [ -z "$dest" ] || [ ! -d "$dest" ]; then
    echo "save_sim_artifacts: no destination directory (tried '${dest}')" >&2
    exit 1
fi

if ! docker ps -a --format '{{.Names}}' | grep -qx "$container"; then
    echo "save_sim_artifacts: no container named '${container}'" >&2
    exit 1
fi

# HOME=/tmp in compose, so Unity writes here rather than under a real home.
player_log=/tmp/.config/unity3d/TIERIV/AWSIM/Player.log
saved=0
for src in "$player_log" /aichallenge/result-summary.json; do
    name=$(basename "$src")
    # NEVER clobber. /aichallenge is a bind mount shared by every run, so the
    # result-summary.json sitting there can easily be an OLDER race's -- copying
    # it over the one AWSIM already wrote into this run's directory destroys the
    # only record of what actually happened. (Learned the hard way: it ate a
    # run's final_position on 2026-08-10.) Whatever is already in the run
    # directory was written for that run and wins.
    if [ -e "${dest}/${name}" ]; then
        echo "keeping existing ${dest}/${name}"
        continue
    fi
    if docker exec "$container" test -f "$src" 2>/dev/null; then
        docker exec "$container" cat "$src" > "${dest}/${name}" && saved=$((saved + 1))
        echo "saved ${dest}/${name}"
    else
        # A stopped container cannot exec; fall back to a copy from its filesystem.
        if docker cp "${container}:${src}" "${dest}/${name}" 2>/dev/null; then
            saved=$((saved + 1))
            echo "saved ${dest}/${name} (from stopped container)"
        else
            echo "save_sim_artifacts: ${src} not found in ${container}" >&2
        fi
    fi
done

[ "$saved" -gt 0 ]
