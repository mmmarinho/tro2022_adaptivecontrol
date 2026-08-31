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
## Stand-alone example create release script
## Murilo M. Marinho (www.murilomarinho.info)
####################################################

release_folder=tro2022_adaptivecontrol_example

# Make release folder
rm -rf ${release_folder}
mkdir ${release_folder}
cd ${release_folder}

# Copy files
cp -r ../bin .
cp -r ../lib .
cp ../.run.sh run_example.sh

# Compress
cd ..
tar -cJf tro2022_adaptivecontrol_example.tar.xz tro2022_adaptivecontrol_example/
