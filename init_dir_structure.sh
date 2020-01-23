#!/bin/bash

## clear directory 'raw'
echo "init directory 'raw'"
# remove directory structure if exists
if [ -d "raw" ]; then
    rm -rf raw
fi

# build directory skeleton for 'raw'
mkdir -p raw/rgb/left
mkdir -p raw/rgb/right
mkdir -p raw/depth/left
mkdir -p raw/depth/right
mkdir -p raw/semantic_segmentation/left
mkdir -p raw/semantic_segmentation/right
