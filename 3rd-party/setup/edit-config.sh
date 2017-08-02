#!/bin/bash
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
function write_source_line {
    if [ -e "$1" ]; then
        SOURCE_LINE="source $(readlink -f $2)"
        grep "$SOURCE_LINE" "$1" > /dev/null 2>&1
        if [ $? != 0 ]; then
            echo $SOURCE_LINE >> $1
        fi
    fi
}

# The output directory is the first argument
SCRIMMAGE_LOCAL_CONFIG_DIR=$(readlink -f "$1")
PROJECT_NAME="$2"

touch ${SCRIMMAGE_LOCAL_CONFIG_DIR}/setup.bash
write_source_line "${SCRIMMAGE_LOCAL_CONFIG_DIR}/setup.bash" "${SCRIMMAGE_LOCAL_CONFIG_DIR}/env/${PROJECT_NAME}-setenv"
