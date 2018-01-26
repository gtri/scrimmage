#!/bin/bash

# Get the artifacts
mkdir -p artifacts
pushd artifacts >& /dev/null
   id=$(docker create gtri/avia:installer)
   docker cp $id:/root/avia/avia/deploy/installer - > installer.tar.gz
   docker rm -v $id
   tar xvf installer.tar.gz
popd >& /dev/null
