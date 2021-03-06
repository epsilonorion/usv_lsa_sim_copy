#!/bin/bash
# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# DEP PACKAGES
apt -qq install --no-install-recommends --allow-unauthenticated -y \
  build-essential python-catkin-tools python-pip dvi2ps dvipng binutils \
  mesa-utils module-init-tools x-window-system ros-$ROS_DISTRO-python-orocos-kdl \
  libfftw3-* libxml++2.6-* libsdl-image1.2-dev libsdl-dev python-rosinstall python-rosinstall-generator \
  python-wstool build-essential python-rosdep python-wxtools python-lxml python-pathlib python-h5py \
  python-scipy python-geolinks python-gdal

