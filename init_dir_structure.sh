#!/bin/bash

## clear directory 'raw'
echo "init directory 'raw'"
# remove directory structure if exists
if [ -d "raw" ]; then
    rm -rf raw
fi

# build directory skeleton for 'raw'
mkdir -p raw/stereo/left/images
mkdir -p raw/stereo/right/images
mkdir -p raw/depth/images
mkdir -p raw/semantic_segmentation/images
