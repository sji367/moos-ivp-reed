#!/bin/bash

GRID=20
BUFFER=5
SEGMENT=5
OUTPUTFILE="MAT"
HELP="0"
PRINT_HELP="1"

print_usage() {

    echo "Available options:
-g or--grid_size or--gridsize or--grid [grid_size]
-b or --buffer_dist or --buffer [buffer_distance]
-s or --segment_dist or --segment [segment_distance]
-o or --output [output_file_type]"
}

# Use -gt 1 to consume two arguments per pass in the loop (e.g. each
# argument has a corresponding value to go with it).
# Use -gt 0 to consume one or more arguments per pass in the loop (e.g.
# some arguments don't have a corresponding value to go with it such
# as in the --default example).
# note: if this is set to -gt 0 the /etc/hosts part is not recognized ( may be a bug )
if [ "$#" -eq 1 ]; then
	while [[ $# -gt 0 ]]
	do
	key="$1"
	case $key in
	    *) # unknown option
	    print_usage
	    exit 1
	    ;;
	esac
	shift # past argument or value
	done

else
	while [[ $# -gt 1 ]]
	do
	key="$1"

	case $key in
	    -g|--grid_size|--gridsize|--grid)
	    GRID="$2"
	    shift # past argument
	    ;;
	    -b|--buffer_dist|--buffer)
	    echo "$2"
	    BUFFER="$2"
	    shift # past argument
	    ;;
	    -s|--segment_dist|--segment)
	    SEGMENT="$2"
	    shift # past argument
	    ;;
	    -o|--output)
	    OUTPUTFILE="$2"
	    shift # past argument
	    ;;
	esac
	shift # past argument or value
	done

	# Build the shape files for the grid
	./../../bin/ENC2Grid

	printf "Launching ENC2Grid \n"

	# Build the grid and make it into a .mat file
	python ../../src/Python/ENC2Grid.py "$GRID" "$OUTPUTFILE"
fi


