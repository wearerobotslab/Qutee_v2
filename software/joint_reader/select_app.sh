#!/bin/bash
# Choose which firmware the next build makes (the same as menuconfig: QUTEE settings -> Application).
# Usage, from software/:  joint_reader/select_app.sh joint_reader   |   joint_reader/select_app.sh qutee
# Then build with "idf.py reconfigure build" (inside the qutee_idf container).
cd "$(dirname "$0")/.." || exit 1
case "$1" in
  joint_reader) on=CONFIG_QUTEE_APP_JOINT_READER; off=CONFIG_QUTEE_APP_FULL ;;
  qutee|full)   on=CONFIG_QUTEE_APP_FULL;         off=CONFIG_QUTEE_APP_JOINT_READER ;;
  *) echo "usage: $0 joint_reader|qutee"; exit 1 ;;
esac
[ -f sdkconfig ] || { echo "no sdkconfig yet: build once first"; exit 1; }
sed -i -e "/^$on=/d" -e "/^# $on is not set/d" -e "/^$off=/d" -e "/^# $off is not set/d" sdkconfig
printf '%s=y\n# %s is not set\n' "$on" "$off" >> sdkconfig
grep -E 'CONFIG_QUTEE_APP_' sdkconfig
