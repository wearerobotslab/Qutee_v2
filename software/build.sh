#!/bin/bash
# Build one of the Qutee applications inside the qutee_idf Docker image (from software/, in WSL or Linux).
#   ./build.sh qutee               # the base robot alone       -> build/qutee/qutee_qutee.bin
#   ./build.sh nn                  # neural network controller  -> build/nn/qutee_nn.bin
#   ./build.sh homeokinesis        # homeokinesis controller    -> build/homeokinesis/qutee_homeokinesis.bin
#   ./build.sh joint_reader        # joint reader               -> build/joint_reader/qutee_joint_reader.bin
#   ./build.sh nn menuconfig       # any other idf.py command (the settings are shared by all applications)
set -e
cd "$(dirname "$0")"
app="$1"
case "$app" in
  qutee|nn|homeokinesis|joint_reader) ;;
  *) echo "usage: $0 qutee|nn|homeokinesis|joint_reader [idf.py command, default: build]"; exit 1 ;;
esac
shift
cmd="${*:-build}"
tty=""
[ -t 0 ] && tty="-it"  # menuconfig needs a terminal
docker run --rm $tty --user espidf -v "$(pwd)":/micro_ros_espidf_component -w /micro_ros_espidf_component qutee_idf \
  bash -lc ". \$IDF_PATH/export.sh && idf.py -B build/$app -DQUTEE_APP=$app $cmd"
