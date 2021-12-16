#!/usr/bin/env bash

# Shell script scripts to install jetfalcon dependencies
# -------------------------------------------------------------------------
#Copyright © 2021 Wei-Chih Lin , kjoelovelife@gmail.com 

#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in all
#copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.
# -------------------------------------------------------------------------
# reference
# https://chtseng.wordpress.com/2019/05/01/nvida-jetson-nano-%E5%88%9D%E9%AB%94%E9%A9%97%EF%BC%9A%E5%AE%89%E8%A3%9D%E8%88%87%E6%B8%AC%E8%A9%A6/
#
# -------------------------------------------------------------------------

if [[ `id -u` -eq 0 ]] ; then
    echo "Do not run this with sudo (do not run random things with sudo!)." ;
    exit 1 ;
fi

# Parameters
sleep_time=3s
ubuntu_distro=$(grep RELEASE /etc/lsb-release | awk -F '=' '{print $2}')
ros_distro=$ROS_DISTRO
hardware_architecture=$(uname -i)
project_name="jetfalcon"

# Functions 

text_color(){
<<"###comment"
	Select color of text
	  Args: 
	    $1: color
		$2: ground, default is fore, can select "fore" or "back"
	  Return: code of color
###comment
	color=("black" "red" "green" "yellow" "blue" "purple" "cyan" "white") 
	declare -A color_code
	initial_color_code=30
	for _color in ${color[*]};
	do
		color_code[$_color]=$initial_color_code
		initial_color_code=$(($initial_color_code + 1))
	done

	if [ $# -eq 1 ]; then
		echo "${color_code[$1]}"
	else
		if [ "$2" == "fore" ]; then
			echo "${color_code[$1]}"
		elif [ "$2" == "back" ]; then
			echo $((${color_code[$1]} + 10))
		else
			echo "${color_code[$1]}"
		fi
	fi
}

power_mode(){
<<'###comment'
    Setup Jetson nano power mode.
    Args:
      hardware: 
###comment
    architecture=$1
    if [ "${architecture}" == "aarch64"  ] ; then
        sudo nvpmodel -m0
        sudo nvpmodel -q
    else 
        echo "You don't use Jeton-nano. Will not setup power mode."
    fi
}

apt_install(){
<<'###comment'
    Use apt install ros2 dependencies for this project
    Args:
      $1: project name
###comment
    _ros_distro=$1
    sudo apt install python-pip \
                     python-frozendict \
                     python-lxml \
	                 python-bs4 \
                     python-tables \
                     python-sklearn \
                     python-rospkg \
                     python-termcolor \
                     python-sklearn \
                     python-dev \
                     python-smbus \
                     git \
                     cmake \
                     libxslt-dev \
                     libxml2-dev \
                     apt-file \
                     iftop \
                     atop \
                     ntpdate \
                     libatlas-base-dev \
                     ipython \
                     libmrpt-dev \
                     mrpt-apps \
                     ros-${_ros_distro}-slam-gmapping \
                     ros-${_ros_distro}-map-server \
                     ros-${_ros_distro}-navigation \
                     ros-${_ros_distro}-vision-msgs \
                     ros-${_ros_distro}-image-transport \
                     ros-${_ros_distro}-image-publisher \
                     ros-${_ros_distro}-teleop-twist-keyboard \
                     byobu \
                     terminator \
                     #Adafruit-MotorHAT \
                     #Adafruit-SSD1306

}

setup_authority(){
    sudo usermod -aG i2c $USER
}

setup_ydlidar(){
<<'###comment'
    Add udev rules for Ydlidar
    Args:
      $1: project name
###comment
    workspace="$HOME/$1/catkin_ws/src/ydlidar_ros"
    echo "Setup YDLidar X4 , and it information in ~/${workspace}/README.md "
    cd ${workspace}/startup
    sudo chmod 777 ./*
    sudo sh initenv.sh
    sudo udevadm control --reload-rules
    sudo udevadm trigger   
}

no_machine(){
<<'###comment'
    Downloads and Install NoMachine
###comment
    architecture=$1
    if [ "${architecture}" == "aarch64"  ]; then
        if test -d "/usr/NX"; then
            echo "NoMachine existed and skip the installations step."
        else
            echo "Downloads Nomachine in $HOME/Downloads and install from the directory."
            wget https://www.nomachine.com/free/arm/v8/deb -O ~/Downloads/nomachine.deb
            sudo dpkg -i ~/Downloads/nomachine.deb
        fi
    else
        echo -e "\e[$(text_color yellow)mYou don't use Jetson nano! You can check the webiste to install noMachine: https://www.nomachine.com/ .\e[0m"
    fi
}

# Install
power_mode $hardware_architecture
apt_install $ros_distro
setup_authority
setup_ydlidar $project_name 
no_machine $hardware_architecture
