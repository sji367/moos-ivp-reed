#!/bin/bash 
COMMUNITY="laptop"

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
#  Part 2: Launch the processes
#-------------------------------------------------------
nsplug $COMMUNITY.moos new_$COMMUNITY.moos -f

printf "Launching the %s MOOS Community (WARP=%s) \n"  $COMMUNITY $TIME_WARP
pAntler new_$COMMUNITY.moos --MOOSTimeWarp=$TIME_WARP >& /dev/null &

sleep 0.25
printf "Launching Image Grabber \n"
#python ../../src/Python/ZBoat_image_grab.py >& /dev/null &

sleep 0.25
printf "Launching ENC_SFoV \n"
#python ../../src/Python/ENC_SFoV.py >& /dev/null &

sleep 0.2
printf "Launching ENC_SFoV_GUI \n"
#python ../../src/Python/ENC_Sensor_GUI.py >& /dev/null &

uMAC new_$COMMUNITY.moos

printf "Killing all processes ... \n"
kill %1 #%2 %3 %4
mykill
printf "Done killing processes.   \n"
