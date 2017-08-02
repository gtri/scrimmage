#!/bin/sh
# ---------------------------------------------------------------------------
# @section LICENSE
#  
# Copyright (c) 2016 Georgia Tech Research Institute (GTRI) 
#               All Rights Reserved
#  
# The above copyright notice and this permission notice shall be included in 
# all copies or substantial portions of the Software.
#  
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
# DEALINGS IN THE SOFTWARE.
# ---------------------------------------------------------------------------
# @file filename.ext
# @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu> 
# @author Eric Squires <eric.squires@gtri.gatech.edu>
# @version 1.0
# ---------------------------------------------------------------------------
# @brief A brief description.
# 
# @section DESCRIPTION
# A long description.
# ---------------------------------------------------------------------------

#branch_name="$(git symbolic-ref HEAD 2>/dev/null)" ||
#branch_name="(unnamed branch)"     # detached HEAD
#branch_name=${branch_name##refs/heads/}
#
## $1 - original-branch 
## $2 - patch-file.patch
#
#echo "==========> BRANCH NAME: $branch_name"
#if [ $branch_name = $1 ]; then
#    echo "APPLY!"
#    git checkout -b gtri-patch
#    git apply $(readlink -f "$2")
#    git commit -a -m "GTRI patch"    
#fi

# How to create a git patch:
# 1.) Make changes in git repository
# 2.) git add <thechanges>
# 3.) git commit -m "patch message"
# 4.) git show HEAD > my.patch

PATCH_FILE=$(readlink -f "$1")

# Only apply the patch if it hasn't been applied yet
git apply --check $PATCH_FILE > /dev/null 2>&1
STATUS=$?
if [ $STATUS -eq 0 ]; then
    git apply $PATCH_FILE
fi
