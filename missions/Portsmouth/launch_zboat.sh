#!/bin/bash 
COMMUNITY="zboat"

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
printf "Launching the %s MOOS Community (WARP=%s) \n"  $COMMUNITY $TIME_WARP
pAntler $COMMUNITY.moos --MOOSTimeWarp=$TIME_WARP >& /dev/null &

uMAC $COMMUNITY.moos

sleep 0.2
printf "Launching Config Reader \n"
python ../../src/Python/config_reader.py >& /dev/null &

sleep 1
printf "Launching pub_points \n"
python ../../src/Python/pub_points.py >& /dev/null &



printf "Killing all processes ... \n"
kill %1 %2 %3
mykill
./../../scripts/turn_off_zboat.py
printf "Done killing processes.   \n"
