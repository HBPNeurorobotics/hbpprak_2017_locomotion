#!/bin/bash
# start stop script to run or kill all external lauron controller


LAURON_PID_FILE=/tmp/lauron_control.pid

function start {
echo 'starting Lauron SNN'
  # check if lauron software is already running
  if [ -f $LAURON_PID_FILE ]
  then
    lauron_control_PID=`cat $LAURON_PID_FILE`
    isrunning=`ps x | grep $lauron_control_PID | grep -v grep`
    if [ -n "$isrunning" ]
    then
      echo "Lauron controller is already running. Do not start it again!"
      return 1
    else
      # remove the pid file, if no process is running
      rm $LAURON_PID_FILE
    fi
  fi

  # source the ros hbp package
  source $ROS_HBP_PACKAGES_SETUP_FILE

  # start the lauron ROS controllers and save PID of the roslaunch process in a file
  roslaunch --pid=$LAURON_PID_FILE lauron_control lauron_control.launch &

  # wait until ros controller are loaded completly before setting lauron to walking mode
  # hint: ros controllers are loaded in fixed order and the last one publishes topic '/robot_leg5_gamma_joint_pos_cntr/state'
  TIMEOUT=30
  while [[ "$(rostopic list | grep /robot_leg5_gamma_joint_pos_cntr/state)" == "" ]] && [[ $TIMEOUT -ne 0 ]]; do
    sleep 1
    TIMEOUT=$TIMEOUT-1
  done
  # exit script with an error, in case of controllers are not loaded (timeout)
  if [[ $TIMEOUT -eq 0 ]]; then
    return 1
  fi

  # set walking mode
  rostopic pub -1 /robot/lauron/walking_preset  std_msgs/UInt8 "data: 2"

  # enable walking
  rostopic pub -1 /robot/lauron/desired_robot_state  std_msgs/UInt8 "data: 4"

  rostopic pub -1 /robot/lauron/desired_robot_state  std_msgs/UInt8 "data: 9"

  # to make sure lauron really is walking, this has to executed again
  rostopic pub -1 /robot/lauron/desired_robot_state  std_msgs/UInt8 "data: 4"

  rostopic pub -1 /robot/lauron/desired_robot_state  std_msgs/UInt8 "data: 9"

  # deactivate ros virtual env
  deactivate

  return 0
}

function stop {


  # get lauron control (roslaunch) PID from file and send kill command
  ROS_LAUNCH_PID=`cat $LAURON_PID_FILE`
  if [ $ROS_LAUNCH_PID ]; then
    kill -INT $ROS_LAUNCH_PID

    # wait until controller_manager/spawner has been completly shutdown
    echo "wait for ros launch pid to be finished: pid = $ROS_LAUNCH_PID"
    TIMEOUT=30
    while [[ $(ps -p$ROS_LAUNCH_PID -o pid=) ]] && [[ $TIMEOUT -ne 0 ]]; do 
      sleep 1
      TIMEOUT=$TIMEOUT-1
    done
    # exit script with an error, in case of controllers are not loaded (timeout)
    if [[ $TIMEOUT -eq 0 ]]; then
      return 1
    fi
  fi

  # delete the pid storage file
  rm $LAURON_PID_FILE


  # deactivate ros virtual env
  deactivate

  echo "Done stopping all lauron control processes."

  return 0
}

mode=$1

case $mode in
'start')
    # Make sure that there is no other Lauron instance running
    stop
    start
    exit 0
    ;;
'stop')
    stop
    exit 0
    ;;
*)
    echo "ERROR: lauron_ext_controller - Unkown input mode $mode!"
    exit 1
    ;;
esac

