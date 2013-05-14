#!/bin/bash
# The script can open tabs and run commands in Ubuntu
# Author: Cheng-Lung Lee (2013.April.04)

# 
# if you are not using it just put $PWD
T1_PATH="$PWD"
T2_PATH="$PWD"
T3_PATH="$PWD"

# The command you want to run
# without "&& bash" , the terminal/tab will close after running the code
T1_Command="roslaunch ROSJAUS JuniorRTE.launch"
T2_Command="roslaunch JAUS_move_base_simple JAUS_move_base.launch "
T3_Command="roslaunch ROSJAUS ROSJAUS_outdoor.launch"

# Title for each tab
T1_Title="JuniorRTE"
T2_Title="move_base"
T3_Title="ROSJAUS"

# use sleep to generate delay
T1_Delay="sleep 1"
T2_Delay="sleep 5"
T3_Delay="sleep 10"

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

echo "#!/bin/bash"                   >  $PWD/temp3.sh
echo $T3_Delay                       >> $PWD/temp3.sh
echo cd $PWD                         >> $PWD/temp3.sh
echo cd $T3_PATH                     >> $PWD/temp3.sh
echo $T3_Command                     >> $PWD/temp3.sh
chmod +x $PWD/temp3.sh

# the command to run 4 tabs
gnome-terminal --tab --title=$T1_Title -e "./temp1.sh" --tab-with-profile=Default --title=$T2_Title -e "./temp2.sh" --tab-with-profile=Default --title=$T3_Title -e "./temp3.sh" 

# delay for a little bit then remove the temp?.sh
sleep 5
$T3_Delay
# clean up
rm $PWD/temp?.sh

