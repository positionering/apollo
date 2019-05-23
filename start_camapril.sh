#!/usr/bin/env bash
cyber_launch start modules/drivers/camera/launch/camera.launch &
cyber_launch start modules/drivers/apriltags/launch/common.launch
