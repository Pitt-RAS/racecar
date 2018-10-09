#!/usr/bin/env bash

pushd $(dirname $BASH_SOURCE) > /dev/null

find src -iname "*.cpp" | xargs uncrustify --check -c uncrustify.cfg
LINT_STATUS=$?

if test $LINT_STATUS -ne 0
then
    echo "uncrustify requested changes"
    exit 1
fi

popd > /dev/null

echo "Lint passed"

exit 0
