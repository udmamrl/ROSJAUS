#!/bin/bash
# The script can open tabs and run commands in Ubuntu
# Author: Cheng-Lung Lee (2013.April.04)

# 
# if you are not using it just put $PWD
T1_PATH="$PWD"
T2_PATH="$PWD"

# The command you want to run
# without "&& bash" , the terminal/tab will close after running the code
T1_Command="roslaunch ROSJAUS JuniorRTE.launch"
T2_Command="roslaunch ROSJAUS WaypointCOP.launch"

# Title for each tab
T1_Title="JuniorRTE"
T2_Title="WaypointCOP"

# use sleep to generate delay
T1_Delay="sleep 1"
T2_Delay="sleep 5"

# scripts for each tab to run
echo "#!/bin/bash"                   >  $PWD/temp1.sh
echo $T1_Delay                       >> $PWD/temp1.sh
echo cd $PWD                         >> $PWD/temp1.sh
echo cd $T1_PATH                     >> $PWD/temp1.sh
echo $T1_Command                     >> $PWD/temp1.sh
chmod +x $PWD/temp1.sh

echo "#!/bin/bash"                   >  $PWD/temp2.sh
echo $T2_Delay                       >> $PWD/temp2.sh
echo cd $PWD                         >> $PWD/temp2.sh
echo cd $T2_PATH                     >> $PWD/temp2.sh
echo $T2_Command                     >> $PWD/temp2.sh
chmod +x $PWD/temp2.sh

# the command to run 4 tabs
gnome-terminal --tab --title=$T1_Title -e "./temp1.sh" --tab-with-profile=Default --title=$T2_Title -e "./temp2.sh" 
# delay for a little bit then remove the temp?.sh
sleep 5
$T2_Delay
# clean up
rm $PWD/temp?.sh

