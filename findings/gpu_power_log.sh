#!/bin/bash
# GPU power + frequency + idle logger for Intel Arc Pro B70 (Battlemage, xe driver)
# Usage: ./gpu_power_log.sh <output.csv> [interval_ms]
# Logs at 500ms by default. Stop with Ctrl-C or kill.
HWMON=/sys/class/drm/card0/device/hwmon/hwmon7
GT=/sys/class/drm/card0/device/tile0/gt0
OUT=${1:-gpu_log.csv}
INTERVAL_MS=${2:-500}
SLEEP_S=$(echo "scale=3; $INTERVAL_MS/1000" | bc)
echo "Logging GPU stats to $OUT every ${INTERVAL_MS}ms"
echo "time_s,energy_uJ,act_freq_MHz,idle_residency_ms,temp_pkg_C,fan_rpm" > "$OUT"
while true; do
  T=$(date +%s.%N)
  E=$(cat $HWMON/energy1_input 2>/dev/null)
  F=$(cat $GT/freq0/act_freq 2>/dev/null)
  IR=$(cat $GT/gtidle/idle_residency_ms 2>/dev/null)
  TP=$(cat $HWMON/temp2_input 2>/dev/null)
  FN=$(cat $HWMON/fan1_input 2>/dev/null)
  echo "$T,$E,$F,$IR,$((TP/1000)),$FN" >> "$OUT"
  sleep $SLEEP_S
done
