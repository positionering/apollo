#!/usr/bin/env bash
/bin/bash start_apriltag.sh /dev/null 2>&1 &
/bin/bash start_leocam.sh &
/bin/bash start_wheelodometry.sh
