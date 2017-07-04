#!/bin/bash

# Parse the .alog files
aloggrep "$1"/"$1".alog NAV_X "$2"x.alog
aloggrep "$1"/"$1".alog NAV_Y "$2"y.alog
#aloggrep "$1"/"$1".alog NAV_HEADING "$2"head.alog
#aloggrep "$1"/"$1".alog VIEW_SEGLIST "$2"seglist.alog
#aloggrep "$1"/"$1".alog pENC_Print "$2"print.alog
aloggrep "$1"/"$1".alog Next_WPT "$2"wpt.alog
aloggrep "$1"/"$1".alog MIN_DIST "$2"dist.alog


# Remove the first 5 lines
sed -i 1,5d "$2"x.alog
sed -i 1,5d "$2"y.alog
#sed -i 1,5d "$2"head.alog
#sed -i 1,5d "$2"seglist.alog
#sed -i 1,5d "$2"print.alog
sed -i 1,5d "$2"wpt.alog
sed -i 1,5d "$2"dist.alog

