#!/usr/bin/env bash
# A .sha256 beside every asset, matching OpenCADStudio's release convention. Written
# with the bare filename so `sha256sum -c` works wherever the file is downloaded to.
set -euo pipefail
for f in "$@"; do
  [ -e "$f" ] || { echo "no such asset: $f" >&2; exit 1; }
  case "$f" in *.sha256) continue;; esac
  ( cd "$(dirname "$f")" && sha256sum "$(basename "$f")" > "$(basename "$f").sha256" )
  cat "$f.sha256"
done
