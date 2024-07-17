#!/bin/bash
cd "$(dirname "$0")"

clang-format --version 2> /dev/null || (echo "clang-format is not installed. Please install clang-format to format Scrimmage" && exit 1)

find ../{src,include,tools} -iname "*.cpp" \
    -o -iname "*.h" \
    -o -iname "*.hpp" | tee /dev/tty | xargs clang-format -i 
