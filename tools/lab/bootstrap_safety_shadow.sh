#!/usr/bin/env bash
# Bootstrap a lab-fork checkout on a Pi that only has the normal Mowgli install.
# It never replaces the runtime checkout, image, map volume, or STM32 firmware.
set -euo pipefail

runtime_dir="${MOWGLI_RUNTIME_DIR:?Set MOWGLI_RUNTIME_DIR to the Pi runtime Mowgli checkout}"
source_dir="${MOWGLI_SAFETY_SOURCE:-$HOME/mowgli-safety-shadow-source}"
repo_url="${MOWGLI_SAFETY_REPO_URL:-https://github.com/liltaket/mowglinext-lab.git}"
repo_ref="${MOWGLI_SAFETY_REPO_REF:?Set MOWGLI_SAFETY_REPO_REF to the pushed branch containing the safety overlay}"

[[ -f "$runtime_dir/docker/.env" ]] || { echo "Runtime checkout is invalid: $runtime_dir" >&2; exit 1; }
if [[ -e "$source_dir" ]]; then
  echo "Refusing to overwrite existing safety source checkout: $source_dir" >&2
  exit 1
fi

mkdir -p "$(dirname "$source_dir")"
git clone --branch "$repo_ref" --single-branch "$repo_url" "$source_dir"
MOWGLI_RUNTIME_DIR="$runtime_dir" MOWGLI_SAFETY_SOURCE="$source_dir" \
  "$source_dir/tools/lab/deploy_safety_shadow.sh" apply
