#!/bin/bash
# Wrapper to fix protoc version detection for raft-proto build.
# protobuf-build 0.14.1 only accepts major==3, but protoc 34.x uses new versioning.
if [[ "$1" == "--version" ]]; then
    echo "libprotoc 3.21.0"
else
    exec /opt/homebrew/bin/protoc "$@"
fi
