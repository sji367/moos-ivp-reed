#!/bin/bash

#ALOG="$1"/"$1".alog

alogclip "$1"/"$1".alog "$1"/c.alog "$3" "$4"
ALOG="$1"/c.alog

# Parse the .alog files
aloggrep $ALOG NAV_X "$2"_x.alog
aloggrep $ALOG NAV_Y "$2"_y.alog
aloggrep $ALOG Next_WPT "$2"_wpt.alog
aloggrep $ALOG MIN_DIST "$2"_dist.alog


# Remove the first 5 lines
sed -i 1,5d "$2"_x.alog
sed -i 1,5d "$2"_y.alog
sed -i 1,5d "$2"_wpt.alog
sed -i 1,5d "$2"_dist.alog

# Rename
mkdir "$2"
cp "$1"/"$1".alog "$2"/"$2".alog
