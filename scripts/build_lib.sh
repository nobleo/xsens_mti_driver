#!/bin/bash

# pushd directory where lib source files are
pushd ../lib/xspublic
# Build library
${MAKE}
# popd directory
popd
