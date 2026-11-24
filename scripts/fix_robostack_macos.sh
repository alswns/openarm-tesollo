#!/usr/bin/env bash
set -euo pipefail

if [[ "$(uname -s)" != "Darwin" ]]; then
  exit 0
fi

prefix="$(cd "$(dirname "$0")/../.conda" && pwd)"
yaml_library="${prefix}/lib/libyaml-0.1.dylib"
if [[ ! -f "${yaml_library}" ]]; then
  echo "Expected RoboStack libyaml library at ${yaml_library}" >&2
  exit 1
fi

while IFS= read -r -d '' library; do
  if otool -L "${library}" 2>/dev/null | grep -q '@rpath/libyaml-0.2.dylib'; then
    install_name_tool -change '@rpath/libyaml-0.2.dylib' '@rpath/libyaml-0.1.dylib' "${library}"
    codesign --force --sign - "${library}"
  fi
done < <(find "${prefix}/lib" -type f \( -name '*.dylib' -o -name '*.so' \) -print0)
