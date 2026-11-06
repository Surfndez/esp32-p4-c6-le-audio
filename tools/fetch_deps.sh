#!/bin/sh
# Fetches third-party source dependencies into third_party/ (not committed), pinned to exact commits:
#   liblc3  v1.1.3  (Google, Apache-2.0)        - LC3 encoder / decoder (p4_le_audio)
#   dr_libs         (David Reid, public domain / MIT-0) - dr_mp3 decoder (examples/tab5_player)
# Re-running is cheap: an existing checkout at the right commit is kept. Needs git.
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TP="$ROOT/third_party"
mkdir -p "$TP"

fetch() {
    name=$1
    url=$2
    commit=$3
    dir="$TP/$name"
    if [ -d "$dir/.git" ] && [ "$(git -C "$dir" rev-parse HEAD 2>/dev/null)" = "$commit" ]; then
        echo "$name: $commit (present)"
        return 0
    fi
    rm -rf "$dir"
    git init -q "$dir"
    git -C "$dir" remote add origin "$url"
    # Shallow fetch of one commit (GitHub allows fetching reachable commits by SHA)
    git -C "$dir" fetch -q --depth 1 origin "$commit"
    git -C "$dir" -c core.autocrlf=false checkout -q FETCH_HEAD
    got="$(git -C "$dir" rev-parse HEAD)"
    [ "$got" = "$commit" ] || { echo "$name: got $got, expected $commit" >&2; exit 1; }
    echo "$name: $commit (fetched)"
}

fetch liblc3 https://github.com/google/liblc3.git 96a3af0beb5487aca3b98a4b992a539a1f6d80d1
fetch dr_libs https://github.com/mackron/dr_libs.git dfe8377631000664666519fdb83da193fd8037f4
