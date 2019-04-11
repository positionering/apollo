#!/usr/bin/env bash
bash stop_docker.sh || true
bash docker/scripts/dev_start.sh -l -t apriltags
#docker exec realsense_dev_twizy1 ./realsenset265/build/pose-grej &

