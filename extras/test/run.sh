#!/bin/sh
# Builds and runs the host tests for examples/SatNode.
set -e
here=$(cd "$(dirname "$0")" && pwd)
src="$here/../../examples/SatNode"
out="${TMPDIR:-/tmp}/sat_test"
${CXX:-c++} -std=c++17 -O1 -g -Wall -Wextra -fsanitize=address,undefined -I"$src" \
  "$here/sat_test.cpp" "$src/Frame.cpp" "$src/Node.cpp" -o "$out"
"$out"
