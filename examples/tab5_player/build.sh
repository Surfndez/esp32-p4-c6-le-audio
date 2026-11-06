#!/bin/sh
# Builds the tab5_player example in the pinned ESP-IDF container (nothing is flashed).
# Usage: sh examples/tab5_player/build.sh [idf.py arguments]   (default: build) -> build/tab5_player.bin
# ccache: LE_AUDIO_CCACHE=0 turns it off; cache directory LE_AUDIO_CCACHE_DIR (default ~/.cache/le-audio-ccache).
set -e
IDF_IMAGE="espressif/idf:v5.5.5@sha256:a9231d0697ab8f7517cc072e93b7c83e04907bfbfba80b6440d7dbbf90665cf2"
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
sh "$ROOT/tools/fetch_deps.sh"
CC_HOST="${LE_AUDIO_CCACHE_DIR:-$HOME/.cache/le-audio-ccache}"
mkdir -p "$CC_HOST"
# Git Bash on Windows: host paths as C:/..., container paths without MSYS conversion.
case "$(uname -s)" in
    MINGW*|MSYS*)
        ROOT="$(cd "$ROOT" && pwd -W)"
        CC_HOST="$(cd "$CC_HOST" && pwd -W)"
        export MSYS_NO_PATHCONV=1
        ;;
esac
docker run --rm -v "$ROOT:/src" -w /src/examples/tab5_player \
    -v "$CC_HOST:/ccache" -e IDF_CCACHE_ENABLE="${LE_AUDIO_CCACHE:-1}" -e CCACHE_DIR=/ccache \
    -e CCACHE_BASEDIR=/src -e CCACHE_MAXSIZE=5G -e CCACHE_NOHASHDIR=1 \
    "$IDF_IMAGE" bash -c "git config --global --add safe.directory '*' && idf.py ${*:-build}"
