#!/usr/bin/env bash
# Build and run host unit tests for the sensor modules.
#
#   CC=gcc ./Tests/run_tests.sh
#   CC="python -m ziglang cc" ./Tests/run_tests.sh     # no gcc on Windows
#
set -u
cd "$(dirname "$0")/.."

CC="${CC:-cc}"
CFLAGS="-std=c99 -Wall -Wextra -Werror -g -O0"
INC="-IDrivers/MPU6050 -IDrivers/Hall -IServices/Speed -IServices/EventDetection -ITests"
OUT=Tests/build
mkdir -p "$OUT"

EXE=""
case "$(uname -s)" in MINGW*|MSYS*|CYGWIN*) EXE=".exe" ;; esac

# test name | sources under test
TESTS=(
  "test_mpu6050|Drivers/MPU6050/mpu6050.c"
  "test_hall|Drivers/Hall/hall.c"
  "test_speed|Services/Speed/speed.c Drivers/Hall/hall.c"
  "test_event_detection|Services/EventDetection/event_detection.c"
  "test_integration|Drivers/MPU6050/mpu6050.c Drivers/Hall/hall.c Services/Speed/speed.c Services/EventDetection/event_detection.c"
)

fail=0
for entry in "${TESTS[@]}"; do
  name="${entry%%|*}"
  srcs="${entry#*|}"
  [ -f "Tests/$name.c" ] || { echo "SKIP $name (no Tests/$name.c)"; continue; }
  echo "=== $name"
  # shellcheck disable=SC2086
  if ! $CC $CFLAGS $INC "Tests/$name.c" $srcs -lm -o "$OUT/$name$EXE"; then
    echo "BUILD FAILED: $name"; fail=1; continue
  fi
  "$OUT/$name$EXE" || fail=1
done

[ $fail -eq 0 ] && echo "ALL TESTS PASSED" || echo "SOME TESTS FAILED"
exit $fail
