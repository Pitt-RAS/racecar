#!/usr/bin/env bash

pushd $(dirname $BASH_SOURCE) > /dev/null

find src -iname "*.h" -o -iname "*.cpp" | xargs uncrustify --check -c uncrustify.cfg

if test $? -ne 0
then
    echo "uncrustify requested changes"
    exit 1
fi


find src -iname "*.py" | xargs flake8 --max-line-length 120

if test $? -ne 0
then
    echo "flake8 lint failed"
    exit 1
fi

popd > /dev/null

echo "Lint passed"

exit 0
