#!/usr/bin/env bash
set -euo pipefail

# Apply the audited RxmRAWX HPG-rover extension to the exact source pinned in
# dependencies.repos. Run after `vcs import src < dependencies.repos`.
workspace_root="${1:-$(cd "$(dirname "$0")/../../.." && pwd)}"
ublox_dir="${workspace_root}/src/ublox"
patch_file="${workspace_root}/src/oit_gnss_ros2/patches/ublox-3.0.0-f9p-rxmrawx.patch"
expected_commit="3a3e1c253722b68e350a60aee6ef399d8c2304cb"

if [[ ! -d "${ublox_dir}/.git" ]]; then
  echo "u-blox source not found: ${ublox_dir}" >&2
  exit 1
fi
if [[ "$(git -C "${ublox_dir}" rev-parse HEAD)" != "${expected_commit}" ]]; then
  echo "Unexpected u-blox revision; refusing to apply F9P RxmRAWX patch." >&2
  exit 1
fi
git -C "${ublox_dir}" apply --check "${patch_file}" || {
  echo "Patch is already applied or does not match the pinned source." >&2
  exit 1
}
git -C "${ublox_dir}" apply "${patch_file}"
echo "Applied F9P RxmRAWX patch. Build the workspace before launching GNSS."
