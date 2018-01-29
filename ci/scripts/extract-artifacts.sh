#!/bin/bash

# Get the artifacts
mkdir -p artifacts
pushd artifacts >& /dev/null
   id=$(docker create scrimmage/centos6-installer:latest)
   docker cp $id:/root/rpms - > rpms.tar.gz
   docker rm -v $id
   tar xvf rpms.tar.gz
popd >& /dev/null
