#!/bin/bash

var=`ps -aux | grep -v grep | grep DJI`

if [ -n "$var" ];then

    echo "DJI is running" >> /tmp/a.out

else
    echo "Here is DJI_Onboard_API_Cmdline_test"
    echo "Here is DJI_Onboard_API_Cmdline_test"
    echo "Here is DJI_Onboard_API_Cmdline_test"

    xunyipwd=/home/ubuntu/Work/Test/output

    oldpwd=`pwd`

    cd $xunyipwd

    ./DJI_Onboard_API_Cmdline_Test &

    cd $oldpwd
fi

