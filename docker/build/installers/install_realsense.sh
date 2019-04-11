#!/usr/bin/env bash

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

# Fail on first error.
set -e

git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
cp config/99-realsense-libusb.rules /etc/udev/rules.d/
udevadm control --reload-rules && udevadm trigger
echo 'hid_sensor_custom' | sudo tee -a /etc/modules
mkdir build && cd build
cmake ../
make uninstall && make clean && make -j8 && sudo make install
