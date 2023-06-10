#!/usr/bin/env bash

set -e



DISTRO_DIR=$(rospack find simulation)/unity_distros

cd ${DISTRO_DIR}

if [ ! -d ${1} ]; then
  tar xzf ${1}.tar.gz
fi

cd ${1}

./$1.x86_64

trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT
wait
