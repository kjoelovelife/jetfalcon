#!/usr/bin/env bash

# set parameter 
project="jetfalcon"
package="${project}_bringup"
workspace="${HOME}/${project}/catkin_ws/src/${package}/param"

dir=$(ls -l $workspace | awk '/^d/ {print $NF}')

# set functions
check_file(){
    # check file existed or not
	# $1: directory name
	if test -e $workspace/$1/$HOSTNAME".yaml"; then
		check=true
	else
		check=false
	fi
	echo $check
}


copy_file(){
	#Copy file from original path to target path in workspace
	for parameter in $dir;
	do
		check=$(check_file $parameter)
		if [ "$check" = true ] ; then
			echo "File $parameter/$HOSTNAME.yaml existed and pass."
		else
			echo "copy file $parameter/default.yaml"
			cp "$workspace/$parameter/default.yaml" "$workspace/$parameter/${HOSTNAME}.yaml"
		fi
	done
}

echo "Setting up parameter files with project ${project} ... "
echo "workspace: ${workspace}"
copy_file