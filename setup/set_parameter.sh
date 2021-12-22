#!/usr/bin/env bash

# set parameter 
project="jetfalcon"
workspace="${HOME}/${project}/catkin_ws/src"

# set functions

function text_color(){
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

function get_path(){
<<'###comment'
	Check file path
	  Args:
	    $1: function
	return: fold path
###comment
	package="${project}_$1"
	echo "${workspace}/${project}_$1/param"
}

function check_file(){
<<'###comment'
	Check file existed or not
	Args:
		$1: directory name
###comment
	if test -e $1; then
		check=true
	else
		check=false
	fi
	echo $check
}

function copy_parameter(){
	#Copy file from original path to target path in workspace
	package_number=$#
	for arg in $@;
	do
		for parameter in $(ls -l $(get_path $arg) | awk '/^d/ {print $NF}');
		do
			file="$(get_path $arg)/$parameter/$HOSTNAME.yaml"
			if [ "$(check_file $file)" = true ]; then
				echo "  File $parameter/$HOSTNAME.yaml existed and pass."
			else
		    	if [ "$(check_file $(get_path $arg)/$parameter/default.yaml)" = true ]; then
					echo "  Copy file $parameter/default.yaml to $file"
					cp "$(get_path $arg)/$parameter/default.yaml" $file
				else
					echo -e "  \e[$(text_color yellow)mThere are special parameters with \"$parameter\". Please check these file.\e[0m"
				fi
			fi
			folder=($(ls -l $(get_path $arg)/$parameter | awk '/^d/ {print $NF}'))
			if [ ${#folder[0]} -gt 0 ]; then
				for _parameter in ${folder[@]}
				do
					file="$(get_path $arg)/$parameter/$_parameter/$HOSTNAME.yaml"
					if [ "$(check_file $file)" = true ]; then
					    echo "  File $parameter/$_parameter/$HOSTNAME.yaml existed and pass."
					else
		    			if [ "$(check_file $(get_path $arg)/$parameter/$_parameter/default.yaml)" = true ]; then
							echo "  Copy file $parameter/$_parameter/default.yaml to $file"
							cp "$(get_path $arg)/$parameter/$_parameter/default.yaml" $file
						else
							echo -e "  \e[$(text_color yellow)mThere are special parameters with \"$parameter/$_parameter\". Please check these file.\e[0m"
						fi
					fi
				done
			else
				:
			fi
		done
	done
}

function check_path(){
    folder=$(ls -l $1 | awk '/^d/ {print $NF}')
	echo $folder
}

echo "Setting up parameter files with project ${project} ... "
#echo "workspace: ${workspace}"
#copy_file
copy_parameter "bringup" "slam" "nav"