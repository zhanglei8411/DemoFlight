#!/bin/bash

var=`ps -aux | grep -v grep | grep xunyi-controlled-exe`

if [ -n "$var" ];then

    echo "Controlled is running" >> /tmp/a.out

else
    echo "Here is Xunyi_Controlled_exe_test"
    echo "Here is Xunyi_Controlled_exe_test"
    echo "Here is Xunyi_Controlled_exe_test"

    xunyipwd=/home/ubuntu/Work/Test/output

    oldpwd=`pwd`

    cd $xunyipwd

    ./xunyi-controlled-exe &

    cd $oldpwd
fi

