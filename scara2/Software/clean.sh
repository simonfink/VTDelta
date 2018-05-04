#!/bin/bash

script="$(readlink -f $0)"
script_dir="$(dirname $script)"

. "$script_dir/config.sh.in"

rm -rf "$build_dir" "$install_dir"


