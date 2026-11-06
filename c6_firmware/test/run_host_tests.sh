#!/bin/sh
# Cyberdeck: host tests of the pure logic of the C6 payload (gcc with warnings and sanitizers): the image guard
# and the CIS adapter's self-disable after crashes.
set -e
DIR="$(cd "$(dirname "$0")/.." && pwd)"
TMP="${TMPDIR:-/tmp}"
CFLAGS="-std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -I$DIR/main"
gcc $CFLAGS "$DIR/test/test_fw_guard_policy.c" "$DIR/main/fw_guard_policy.c" -o "$TMP/fw_guard_policy_test"
"$TMP/fw_guard_policy_test"
gcc $CFLAGS "$DIR/test/test_cis_c6_crash.c" "$DIR/main/cis_c6_crash.c" -o "$TMP/cis_c6_crash_test"
"$TMP/cis_c6_crash_test"
