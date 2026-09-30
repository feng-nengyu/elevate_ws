#!/usr/bin/env bash
set -eo pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$workspace/test_buttons_wzl.bash" --click "$@"
