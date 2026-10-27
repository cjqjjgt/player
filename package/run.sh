#!/bin/sh
export LD_LIBRARY_PATH=$(dirname $0)/lib:$LD_LIBRARY_PATH
exec $(dirname $0)/bin/rk3572_player "$@"
