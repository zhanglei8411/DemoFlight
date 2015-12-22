#!/bin/bash

echo "Here is DJI_Onboard_API_Cmdline_test"
echo "Here is DJI_Onboard_API_Cmdline_test"
echo "Here is DJI_Onboard_API_Cmdline_test"

djipwd=/home/ubuntu/Work/Aircraft/5.auto_flight_test/output

oldpwd=`pwd`

cd $djipwd

#ls

./DJI_Onboard_API_Cmdline_Test 30

cd $oldpwd

#ls
