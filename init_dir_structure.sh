#!/bin/bash

## clear directory 'raw'
echo "clear directory 'raw'"
# remove directory structure if exists
if [ -d "raw" ]; then
    rm -rf raw
fi
# build directory structure for 'raw'
mkdir -p raw/stereo/left/images
mkdir -p raw/stereo/right/images
mkdir raw/stereo/left/timestamps
mkdir raw/stereo/right/timestamps

## clear directory 'processed'
echo "clear directory 'processed'"
# remove directory structure if exists
if [ -d "processed" ]; then
    rm -rf processed
fi
# build directory structure for 'processed'
mkdir processed
