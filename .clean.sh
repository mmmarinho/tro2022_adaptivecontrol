#!/bin/bash

####################################################
## Stand-alone example clean script
## Murilo M. Marinho (www.murilomarinho.info)
####################################################
#
# The original script builds and install in-source,
# which might be an issue for some people. This commands
# cleans ALL files that were generated, including the
# build folders. So be careful.
#

GREEN='\033[1;32m'
RED='\033[1;31m'
NC='\033[0m' # No Color

echo -e ${RED}"This script will ATTEMPT to delete all files installed by the .build.sh command."${NC}
echo -e ${RED}"Files removed this way cannot be recovered."${NC}
read -p "Are you sure? [N/y]" -n 1 -r
echo    # (optional) move to a new line
if [[ $REPLY =~ ^[Yy]$ ]]
then
    # Process CMAKE manifest files
    echo -e ${GREEN}"[1/2] Processing manifest files."${NC}
    manifest_files=$(find . -name "install_manifest.txt")
    if [ -z $manifest_files ]; then
      echo -e ${GREEN}"No manifest files found."${NC}
    else
        for manifest_file in ${manifest_files[@]}; do
          echo -e ${GREEN}"Processing "${manifest_file}"."${NC}
          echo -e ${RED}"Deleting all files below."${NC}
          cat ${manifest_file}
          echo
          xargs rm < ${manifest_file}
        done
    fi
    
    # Remove build folders
    echo -e ${GREEN}"[2/2] Removing build folders described below."${NC}
    build_folders=$(find . -name "build" -type d)
    if [ -z $build_folders ]; then
        echo -e ${GREEN}"No build folders found."${NC}
    else
        for build_folder in ${build_folders[@]}; do
          echo -e ${RED}"Deleting folder below."${NC}
          echo ${build_folder}
          rm -r ${build_folder}
        done
    fi
fi

# Ref  https://stackoverflow.com/questions/5947742/how-to-change-the-output-color-of-echo-in-linux
