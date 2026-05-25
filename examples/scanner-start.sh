#!/bin/bash
# EDACS ProVoice DSD-FME launcher — two-dongle mode

RECORDINGS_DIR="/var/lib/scanner/recordings"
CHANNEL_MAP="/etc/scanner/channel_map.csv"
GROUPS_CSV="/etc/scanner/groups.csv"
LOG_FILE="/var/log/scanner/dsd-fme.log"

# -------------------------------------------------------
CTRL_FREQ="851.675M"    # Control channel — LCN 3 (confirmed)
CC_DEVICE="0"           # CC dongle: stays on control channel (SN: 12964344)
CC_GAIN="40"
CC_PPM="0"
CC_SQUELCH="30"         # RMS threshold for analog squelch; idle noise floor ~3, carrier ~100+

VC_DEVICE="1"           # VC dongle: tunes to voice grants (SN: 36503037)
VC_GAIN="40"
VC_PPM="0"
VC_BW="24"

EDACS_MODE="-fh"        # EDACS Standard/Networked (confirmed by CC decode)
# -------------------------------------------------------

# Kill any stale rtl_tcp holding the VC dongle
pkill -f "rtl_tcp.*-d ${VC_DEVICE}" 2>/dev/null; sleep 0.5

mkdir -p "${RECORDINGS_DIR}"
echo "[$(date)] Scanner starting (two-dongle mode)..." >> "${LOG_FILE}"

exec dsd-fme \
  -fp \
  ${EDACS_MODE} \
  -i rtl:${CC_DEVICE}:${CTRL_FREQ}:${CC_GAIN}:${CC_PPM}:24:${CC_SQUELCH}:2 \
  -o null \
  -T \
  -C ${CHANNEL_MAP} \
  -G ${GROUPS_CSV} \
  -7 ${RECORDINGS_DIR}/ \
  -P \
  -j ${VC_DEVICE}:${VC_GAIN}:${VC_PPM}:${VC_BW} \
  2>>"${LOG_FILE}"
