#!/bin/sh
# CIS simulator: builds (gcc C11, -Werror, ASan/UBSan) and runs all scenarios.
#  1) sim_ref: the simulator + a test Central (test_central.c) - harness self-test, unit tests.
#  2) sim_eng: the simulator + the engine from ../*.c (when cis_engine.c exists), otherwise the stub stub_engine.c.
# Running from Windows:
#   docker run --rm --entrypoint bash -v "$PWD:/src" espressif/idf:v5.5.5 -c "sh /src/cis_ll/sim/run.sh"
#   (from the repository root; Git Bash on Windows: MSYS_NO_PATHCONV=1 and "$(pwd -W)")
# Logs: cis_ll/sim/out/logs_{ref,eng}/<scenario>.log; only summaries on screen.
set -u
SIM_DIR=$(cd "$(dirname "$0")" && pwd)
CIS_DIR=$(cd "$SIM_DIR/.." && pwd)
OUT=${SIM_OUT:-$SIM_DIR/out}
mkdir -p "$OUT/logs_ref" "$OUT/logs_eng"

CC=${CC:-gcc}
CFLAGS="-std=c11 -Wall -Wextra -Werror -O1 -g -fno-omit-frame-pointer \
	-fsanitize=address,undefined -fno-sanitize-recover=all -I$SIM_DIR -I$CIS_DIR"
SIM_SRC="sim_core.c sim_crypto.c sim_phy.c sim_periph.c sim_acl.c sim_platform.c sim_host.c sim_main.c sim_selftest.c"
SRC=""
for f in $SIM_SRC; do SRC="$SRC $SIM_DIR/$f"; done

rc=0
echo "== build sim_ref (test Central) =="
if ! $CC $CFLAGS $SRC "$SIM_DIR/test_central.c" -lm -o "$OUT/sim_ref" 2>"$OUT/build_ref.log"; then
	cat "$OUT/build_ref.log"
	exit 1
fi
echo "== self-test: unit vectors + harness with test Central =="
# hwseq with the test Central MUST fail (the Central assumes radio_owns_seq = false) - see the README.
"$OUT/sim_ref" --selftest --logdir "$OUT/logs_ref" \
	--scenario ideal --scenario loss1 --scenario loss5 --scenario drift --scenario coex10 \
	--scenario hostjitter --scenario nse1 --scenario nse3 --scenario reject \
	--scenario aclloss --scenario noenc --scenario neg_mic --scenario neg_skew --scenario neg_nohop \
	--scenario neg_hwseq || rc=1
# The test Central has FT = 1: at NSE 1 the loss ~= the air loss (a coin flip around the 1 % threshold), so two
# earbuds are checked with NSE 3.
"$OUT/sim_ref" --logdir "$OUT/logs_ref" --scenario twobuds --max-nse 3 || rc=1
echo "== info: hwseq with test Central (expected FAIL: ACL sequencer vs CIS SN/NESN rules) =="
"$OUT/sim_ref" --logdir "$OUT/logs_ref" --scenario hwseq >/dev/null 2>&1 \
	&& echo "hwseq: unexpectedly PASS" || echo "hwseq: FAIL as expected (see logs_ref/hwseq.log)"

if [ -f "$CIS_DIR/cis_engine.c" ]; then
	ENG_SRC=$(ls "$CIS_DIR"/*.c)
	echo "== build sim_eng (cis_ll engine: $(echo $ENG_SRC | wc -w) files) =="
else
	ENG_SRC="$SIM_DIR/stub_engine.c"
	echo "== build sim_eng (STUB - cis_ll/cis_engine.c absent) =="
fi
if ! $CC $CFLAGS $SRC $ENG_SRC -lm -o "$OUT/sim_eng" 2>"$OUT/build_eng.log"; then
	cat "$OUT/build_eng.log"
	exit 1
fi
echo "== scenarios against the engine (G2) =="
# The list comes from the program (like --all: without neg_*), so that a new scenario becomes a gate without changes
# here. hwseq is a model of the ACL sequencer (radio_owns_seq = 1): it must fail - it shows risk R1, which is why the
# C6 adapter writes the engine SN/NESN.
SCEN=$("$OUT/sim_eng" --help 2>/dev/null | awk '/^scenarios:/ { f = 1; next } f && NF { print $1 }' \
	| grep -v '^neg_' | grep -vx hwseq)
if [ -z "$SCEN" ]; then
	echo "sim_eng: no scenario list (--help) - aborted"
	exit 1
fi
ARGS=""
for s in $SCEN; do ARGS="$ARGS --scenario $s"; done
"$OUT/sim_eng" --logdir "$OUT/logs_eng" $ARGS || rc=1
echo "== info: hwseq against the engine (expected FAIL, risk R1; not a gate) =="
if "$OUT/sim_eng" --logdir "$OUT/logs_eng" --scenario hwseq >"$OUT/logs_eng/hwseq_summary.txt" 2>&1; then
	echo "hwseq: unexpectedly PASS (see logs_eng/hwseq_summary.txt)"
else
	echo "hwseq: FAIL as expected ($(grep -m1 -i 'loss' "$OUT/logs_eng/hwseq_summary.txt" | tr -s ' '))"
fi
exit $rc
