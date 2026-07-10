#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CLANG_FORMAT="${CLANG_FORMAT:-clang-format}"

if ! command -v "${CLANG_FORMAT}" >/dev/null 2>&1; then
  echo "error: ${CLANG_FORMAT} not found" >&2
  exit 1
fi

mapfile -t FILES < <(
  find "${ROOT}/include/vectorview" "${ROOT}/src/plugin" "${ROOT}/src/gui" \
    "${ROOT}/src/filters" "${ROOT}/src/common" "${ROOT}/tests" \
    -type f \( -name '*.cpp' -o -name '*.h' \) 2>/dev/null | sort
)

if [ "${#FILES[@]}" -eq 0 ]; then
  echo "no first-party source files found"
  exit 0
fi

"${CLANG_FORMAT}" -i "${FILES[@]}"
echo "formatted ${#FILES[@]} file(s)"
