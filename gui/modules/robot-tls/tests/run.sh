#!/bin/bash
set -u
MODE=${1:?usage: run.sh swift|kotlin}
HERE=$(cd "$(dirname "$0")" && pwd)
MOD=$(dirname "$HERE")
W=$(mktemp -d)
PIDS=()
cleanup() { kill "${PIDS[@]}" 2>/dev/null; wait 2>/dev/null; rm -rf "$W"; }
trap cleanup EXIT
cd "$W" || exit 1

key() { openssl ecparam -name prime256v1 -genkey -noout -out "$1.pem" 2>/dev/null && openssl req -new -x509 -key "$1.pem" -out "$1.crt" -days 1 -subj "/CN=$1" 2>/dev/null; }
key ec; key ec2
openssl req -x509 -newkey rsa:2048 -nodes -keyout rsa.pem -out rsa.crt -days 1 -subj /CN=rsa 2>/dev/null
FP=$(openssl ec -in ec.pem -pubout -outform DER 2>/dev/null | openssl dgst -sha256 | sed 's/^.*= *//')
echo "expected fp $FP"
mkdir -p www/api/robot && printf '{"serial_number":" RBQ-T1 "}' > www/api/robot/serial_number
head -c 16777216 /dev/zero > www/big
s_server() { (cd www && exec openssl s_server -accept "$1" -cert "../$2.crt" -key "../$2.pem" -WWW -quiet) >/dev/null 2>&1 & PIDS+=($!); }
s_server 8080 ec; S8080=$!
s_server 18084 ec
s_server 18086 ec
s_server 18083 rsa
python3 "$HERE/fake.py" servers & PIDS+=($!)
sleep 1

case $MODE in
  swift)
    swiftc -O -o t "$MOD/ios/RobotTlsCore.swift" "$HERE/main.swift" || exit 1
    run() { ./t "$@"; } ;;
  kotlin)
    G=~/.gradle/caches/modules-2/files-2.1
    KV=${KOTLIN_VERSION:-2.1.20}
    j() { find "$G/$1" -name "*.jar" ! -name "*-sources.jar" 2>/dev/null | sort | tail -1; }
    STD=$(j org.jetbrains.kotlin/kotlin-stdlib/$KV); JSON=$(j org.json/json)
    [ -n "$STD" ] && [ -n "$JSON" ] || { echo "kotlin $KV stdlib / org.json 이 gradle 캐시에 없다"; exit 1; }
    KC=$(j org.jetbrains.kotlin/kotlin-compiler-embeddable/$KV):$STD:$(j org.jetbrains.kotlin/kotlin-daemon-embeddable/$KV):$(j org.jetbrains.kotlin/kotlin-script-runtime/$KV):$(j org.jetbrains.kotlin/kotlin-reflect/$KV):$(j org.jetbrains.intellij.deps/trove4j):$(j org.jetbrains.kotlinx/kotlinx-coroutines-core-jvm):$(j org.jetbrains/annotations/13.0)
    SRC="$MOD/android/src/main/java/expo/modules/robottls/RobotTlsModule.kt"
    { grep '^import ' "$SRC" | grep -v '^import expo\.'; sed -n '/^private const val PROBE_MS/,$p' "$SRC"; cat "$HERE/Main.kt"; } > CoreTest.kt
    java -cp "$KC" org.jetbrains.kotlin.cli.jvm.K2JVMCompiler -no-stdlib -no-reflect -cp "$STD:$JSON" -d out CoreTest.kt 2>&1 | grep -v '^warning' || true
    [ -f out/CoreTestKt.class ] || exit 1
    run() { java -cp "out:$STD:$JSON" CoreTestKt "$@"; } ;;
  *) echo "usage: run.sh swift|kotlin"; exit 2 ;;
esac

run main "$FP"; rc1=$?
kill "$S8080"; wait "$S8080" 2>/dev/null
python3 "$HERE/fake.py" swap ec ec2 & PIDS+=($!)
sleep 1
run swap "$FP"; rc2=$?
[ $rc1 = 0 ] && [ $rc2 = 0 ] && echo "ALL PASS ($MODE)" || { echo "FAILED ($MODE)"; exit 1; }
