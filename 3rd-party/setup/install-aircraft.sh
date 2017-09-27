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

#relpath finds the relative path between two directories. It used in the
#symbolic linking below
relpath() {
    # both $1 and $2 are absolute paths beginning with /
    # $1 must be a canonical path; that is none of its directory
    # components may be ".", ".." or a symbolic link
    #
    # returns relative path to $2/$target from $1/$source
    source=$1
    target=$2

    common_part=$source
    result=

    while [ "${target#"$common_part"}" = "$target" ]; do
        # no match, means that candidate common part is not correct
        # go up one level (reduce common part)
        common_part=$(dirname "$common_part")
        # and record that we went back, with correct / handling
        if [ -z "$result" ]; then
            result=..
        else
            result=../$result
        fi
    done

    if [ "$common_part" = / ]; then
        # special case for root (no common path)
        result=$result/
    fi

    # since we now have identified the common part,
    # compute the non-common part
    forward_part=${target#"$common_part"}

    # and now stick all parts together
    if [ -n "$result" ] && [ -n "$forward_part" ]; then
        result=$result$forward_part
    elif [ -n "$forward_part" ]; then
        # extra slash removal
        result=${forward_part#?}
    fi

    printf '%s\n' "$result"
}

COPY_FILES=false
while getopts ":c" opt; do
    case $opt in
        c)
            COPY_FILES=true
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            exit 1
            ;;
        :)
            echo "Option -$OPTARG requires an argument." >&2
            exit 1
            ;;
    esac
done
shift $((OPTIND -1))

if [ "$#" -lt 1 ]; then
    echo "usage: $0 <jsbsim-code root directory>"
    exit -1;
fi

ROOT_DIR=$(readlink -f "$1")

# Jump to the directory that holds this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd ${DIR} >& /dev/null

# Make sym links from this repo's scripts dir to jsbsim's
SRC_DIR=$(readlink -f ../../data/scripts)
DST_DIR=$(readlink -f ${ROOT_DIR}/scripts)
REL_PATH="$(relpath "$DST_DIR" "$SRC_DIR")"
#ln -f -s $REL_PATH/* $DST_DIR/ # The wildcard in ln doesn't work in VMs
if [ $COPY_FILES == true ]; then
    find $SRC_DIR -name '*.xml' | xargs -i cp {} ${DST_DIR}/
else
    find $SRC_DIR -name '*.xml' -printf '%f\n' | xargs -i ln -f -s "$REL_PATH/{}" "$DST_DIR"
fi

# Make sym links from this repo's aircraft dir to jsbsim's
SRC_DIR=$(readlink -f ../../data/aircraft/)
DST_DIR=$(readlink -f ${ROOT_DIR}/aircraft)
REL_PATH="$(relpath "$DST_DIR" "$SRC_DIR")"
#ln -f -s $REL_PATH/* $DST_DIR/ # The wildcard in ln doesn't work in VMs
if [ $COPY_FILES == true ]; then
    find $SRC_DIR -maxdepth 1 -type d ! -name "aircraft" | xargs -i cp -r {} ${DST_DIR}/
else
    find $SRC_DIR -maxdepth 1 -type d ! -name "aircraft" -printf '%f\n' | xargs -i ln -f -s $REL_PATH/{} "$DST_DIR"
fi

popd >& /dev/null
