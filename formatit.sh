#!/usr/bin/env bash

pushd $(dirname $BASH_SOURCE) > /dev/null

find src -iname "*.h" -o -iname "*.cpp" | xargs uncrustify --replace --no-backup -c uncrustify.cfg

popd > /dev/null
