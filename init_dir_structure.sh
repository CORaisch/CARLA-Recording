#!/bin/bash

## clear directory 'raw'
echo "init directory 'raw'"
# remove directory structure if exists
if [ -d "raw" ]; then
    rm -rf raw
fi

# build directory skeleton for 'raw'
mkdir -p raw/rgb/left/images
mkdir -p raw/rgb/right/images
mkdir -p raw/depth/left/images
mkdir -p raw/depth/right/images
mkdir -p raw/semantic_segmentation/left/images
mkdir -p raw/semantic_segmentation/right/images
