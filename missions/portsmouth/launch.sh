#!/bin/bash 
COMMUNITY="alpha"

#-------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-------------------------------------------------------
TIME_WARP=1
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
	printf "%s [SWITCHES] [time_warp]   \n" $0
	printf "  --help, -h         \n" 
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    else 
	printf "Bad Argument: %s \n" $ARGI
	exit 0
    fi
done

#-------------------------------------------------------
#  Part 2: Create the .moos and .bhv files. 
#-------------------------------------------------------
#nsplug $COMMUNITY.moos OA.moos


#-------------------------------------------------------
#  Part 3: Launch the processes
#-------------------------------------------------------
printf "Launching the %s MOOS Community (WARP=%s) \n"  $COMMUNITY $TIME_WARP
pAntler alpha.moos --MOOSTimeWarp=$TIME_WARP >& /dev/null &

sleep 0.5
printf "Launching ENC_Contact \n"
python ~/anaconda2/ENC_Search.py >& /dev/null &

sleep 2
printf "Launching ENC_Print \n"
python ~/anaconda2/ENC_Print.py >& /dev/null &

sleep 0.5
printf "Launching ENC_WPT_check \n"
python ~/anaconda2/ENC_WPT_check.py >& /dev/null &

sleep 0.25
printf "Launching ENC_SFoV \n"
#python ~/anaconda2/ENC_SFoV.py >& /dev/null &

sleep 0.2
printf "Launching ENC_Camera \n"
#python ~/anaconda2/ENC_SFoV_GUI.py >& /dev/null &

uMAC alpha.moos


printf "Killing all processes ... \n"
kill %1 %2 %3 %4 #%5 %6
#mykill
printf "Done killing processes.   \n"

