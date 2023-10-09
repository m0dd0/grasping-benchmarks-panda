#!/bin/bash
set -e # exit on first error

# execute any command entered after the docker run command
echo "execute command $@"
exec "$@"