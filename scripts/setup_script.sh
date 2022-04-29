#!/bin/bash

#  Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

ROOT=/opt/robot
current_dir=$(pwd)

echo "*************** INSTALLATION DIR: $ROOT ****************"

exit_setup()
{
    echo "Setup FAILED! : Make sure you have active network connection"
    cd $current_dir
    exit 1
}

# Install inputs package
pip3 install inputs
if [ "$?" -ne "0" ]; then
    exit_setup
fi

# Create the directory path, if it does not exist
mkdir -p $ROOT

# Install scuttlepy
ls $ROOT | grep "scuttlepy"
if [ "$?" -ne "0" ]; then
    echo "Cloning scuttlepy project."
    git clone https://github.com/ansarid/scuttlepy.git $ROOT/scuttlepy
    if [ "$?" -ne "0" ]; then
        exit_setup
    fi

    pip3 install $ROOT/scuttlepy
    if [ "$?" -ne "0" ]; then
        exit_setup
    fi
fi

# Install ros-imu-bno055
ls $ROOT | grep "ros-imu-bno055"
if [ "$?" -ne "0" ]; then
    echo "Cloning ros-imu-bno055 project."
    git clone https://github.com/dokkwon/ros-imu-bno055.git $ROOT/ros-imu-bno055
    if [ "$?" -ne "0" ]; then
        exit_setup
    fi
fi

# Install SCUTTLE_ROS
ls $ROOT | grep "scuttle_ws"
if [ "$?" -ne "0" ]; then
    echo "Cloning SCUTTLE_ROS project."
    git clone -b noetic https://github.com/scuttlerobot/SCUTTLE_ROS/ $ROOT/scuttle_ws
    if [ "$?" -ne "0" ]; then
        exit_setup
    fi
fi

# Install TI GPIO Python
ls $ROOT | grep "ti-gpio-py"
if [ "$?" -ne "0" ]; then
    echo "Cloning TI.GPIO python project."
    git clone --single-branch --branch master https://github.com/TexasInstruments/ti-gpio-py.git $ROOT/ti-gpio-py
    if [ "$?" -ne "0" ]; then
        exit_setup
    fi
fi

sync

echo "Setup Done!"
