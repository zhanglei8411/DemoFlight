#!/bin/bash

var=`ps -aux | grep -v grep | grep DJI`

if [ -n "$var" ];then

    echo "DJI is running" >> /tmp/a.out

else
    echo "Here is DJI_Onboard_API_Cmdline_test"
    echo "Here is DJI_Onboard_API_Cmdline_test"
    echo "Here is DJI_Onboard_API_Cmdline_test"

    djipwd=/home/ubuntu/Work/New-code/Developing/Linux-Cmdline/output

    oldpwd=`pwd`

    cd $djipwd

    ./DJI_Onboard_API_Cmdline_Test &

    cd $oldpwd
fi

