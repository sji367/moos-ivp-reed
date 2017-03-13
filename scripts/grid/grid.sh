#!/bin/bash

# Build the shape files for the grid
./../../bin/ENC2Grid

# Build the grid and make it into a .mat file
python ../../src/Python/ENC2Grid.py >& /dev/null &

