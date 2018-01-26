#!/bin/bash
#
# This script reads the current user's ID ($UID) and write it to the
# .env file. We do this because dockerfiles and docker-compose files
# do not have access to shell variables, only environment
# variables. $UID is a shell variable and we can't make a call to it
# from docker-compose
# See: https://github.com/docker/compose/issues/2380
#
# Now, docker-compose does allow for variable substitution. It can
# read variables specified in an .env file.
# See: https://vsupalov.com/docker-env-vars/
#
# So, the current pipline is as follows:
# The 'makefile' runs the 'create_env_file' script which creates the
# '.env' file containing the user ID. It also runs the
# 'docker-compose.yml' file which reads the user ID from the '.env'
# file and passess it as an argument to the corresponding dockerfile.


echo "# THIS FILE IS CREATED BY THE CREATE_ENV_FILE SCRIPT" > .env
echo "# DO NOT MODIFY" >> .env
echo "USER_ID=$UID" >> .env
