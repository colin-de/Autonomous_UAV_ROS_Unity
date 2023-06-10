#!/usr/bin/env bash

set -e

pushd .

### PIXHAWK SITL
cd $RRG_QUADROTOR_DEPS/src/pixhawk_fla
make run_sitl_quad

popd

trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT
wait
