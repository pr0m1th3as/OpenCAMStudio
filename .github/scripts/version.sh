#!/usr/bin/env bash
# The release version. On a tag it is the tag; on a workflow_dispatch rehearsal there
# is no tag, so fall back to the workspace version with a -dev suffix, which keeps the
# artifact names honest about not being a release.
set -euo pipefail
if [[ "${GITHUB_REF:-}" == refs/tags/v* ]]; then
  V="${GITHUB_REF_NAME}"
else
  V="v$(sed -n 's/^version = "\(.*\)"/\1/p' Cargo.toml | head -1)-dev"
fi
echo "v=$V" >> "$GITHUB_OUTPUT"
echo "version: $V"
