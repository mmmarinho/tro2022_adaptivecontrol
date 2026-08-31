# (C) Copyright 2023-2026 Murilo Marques Marinho (www.murilomarinho.info)
#
# This file is part of adaptive_control_example.
#
# SPDX-License-Identifier: MIT
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

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
