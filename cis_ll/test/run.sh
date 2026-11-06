#!/bin/sh
# CIS engine host tests: gcc C11, -Wall -Wextra -Werror, ASan + UBSan. Run in the espressif/idf image
# (see docs/c6-link-layer.md). Result: the last line is "ALL PASS", otherwise a non-zero exit code.
set -e
D=$(cd "$(dirname "$0")/.." && pwd)
B=${TMPDIR:-/tmp}/cis_ll_test
mkdir -p "$B"
CFLAGS="-std=c11 -O1 -g -Wall -Wextra -Werror -Wshadow -Wconversion -Wno-sign-conversion -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer -I$D"
SRC="$D/cis_proto.c $D/cis_crypto.c $D/cis_engine.c $D/test/aes_ref.c"
rc=0
for t in test_proto test_crypto test_engine; do
	[ -f "$D/test/$t.c" ] || continue
	gcc $CFLAGS -o "$B/$t" "$D/test/$t.c" $SRC || { echo "BUILD FAIL $t"; exit 2; }
	if ! "$B/$t"; then rc=1; fi
done
if [ $rc -eq 0 ]; then echo "ALL PASS"; else echo "FAILURES"; fi
exit $rc
