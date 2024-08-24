#!/bin/bash

which idf.py >/dev/null || {
    source ~/export-esp.sh >/dev/null 2>&1
}

# Need to tell which font sizes to include
# https://github.com/slint-ui/slint/issues/4956
# export SLINT_FONT_SIZES="8,10,12,14,16,18,20,32,42,60,100";

case "$1" in
"" | "release")
    cargo build --release
    ;;
"debug")
    cargo build
    ;;
*)
    echo "Wrong argument. Only \"debug\"/\"release\" arguments are supported"
    exit 1
    ;;
esac
