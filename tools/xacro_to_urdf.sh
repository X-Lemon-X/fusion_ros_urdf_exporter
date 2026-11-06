#!/usr/bin/env bash
# Convert an exported package's xacro to a plain URDF.
#
# Writes <package>/urdf/<package>.urdf. By default the mesh paths are
# relative to that file (../meshes/<name>.stl), so the package folder can be
# moved or copied anywhere. Use this for importers that read only plain URDF,
# e.g. the Isaac Sim URDF importer.
#
# Usage:
#   xacro_to_urdf.sh <package_folder> [--package] [--collision]
#
#   --package     keep package://<package>/... mesh paths (for ROS tools)
#   --collision   add a <collision> per link, a copy of its visual mesh
#                 (the exporter writes none; simulators need it, or the
#                 robot falls through the ground)
#
# Needs xacro: a sourced ROS 2 (found under /opt/ros if not sourced) or
# `pip install xacro`. Run it again after every export or override change.
set -euo pipefail

usage() {
    echo "Usage: $0 <package_folder> [--package] [--collision]" >&2
    exit 1
}

pkg_arg=""
keep_package=false
collision=false
for arg in "$@"; do
    case $arg in
        --package) keep_package=true ;;
        --collision) collision=true ;;
        -*) usage ;;
        *) [[ -z $pkg_arg ]] || usage; pkg_arg=$arg ;;
    esac
done
[[ -n $pkg_arg ]] || usage

[[ -d $pkg_arg ]] || { echo "Not a folder: $pkg_arg" >&2; exit 1; }
pkg_dir=$(cd "$pkg_arg" && pwd)
pkg=$(basename "$pkg_dir")
src="$pkg_dir/urdf/$pkg.urdf.xacro"
out="$pkg_dir/urdf/$pkg.urdf"
[[ -f $src ]] || { echo "Not an exported package, missing: $src" >&2; exit 1; }

if ! command -v xacro >/dev/null 2>&1; then
    for setup in /opt/ros/*/setup.bash; do
        [[ -f $setup ]] || continue
        set +u
        # shellcheck disable=SC1090
        source "$setup"
        set -u
        break
    done
fi
command -v xacro >/dev/null 2>&1 || {
    echo "xacro not found: source ROS 2 or run 'pip install xacro'" >&2
    exit 1
}

# Write to a temporary file first, so a failed run leaves no broken URDF.
tmp=$(mktemp "$pkg_dir/urdf/.$pkg.urdf.XXXXXX")
trap 'rm -f "$tmp"' EXIT
xacro "$src" > "$tmp"
if ! $keep_package; then
    sed -i.bak "s#package://$pkg/#../#g" "$tmp" && rm -f "$tmp.bak"
fi
if $collision; then
    # After each <visual>…</visual>, add a <collision> with the same origin
    # and geometry (material removed). xacro itself needs Python, so
    # python3 is there.
    python3 - "$tmp" <<'PY'
import re, sys
path = sys.argv[1]
text = open(path, encoding="utf-8").read()

def add_collision(m):
    indent, body = m.group(1), m.group(2)
    body = re.sub(r"\s*<material\b.*?(</material>|/>)", "", body, flags=re.S)
    return f"{m.group(0)}\n{indent}<collision>{body}</collision>"

text = re.sub(r"([ \t]*)<visual>(.*?)</visual>", add_collision, text, flags=re.S)
open(path, "w", encoding="utf-8").write(text)
PY
fi
mv "$tmp" "$out"
trap - EXIT

# Check that every mesh the URDF names exists.
missing=0
if ! $keep_package; then
    while read -r mesh; do
        if [[ ! -f "$pkg_dir/urdf/$mesh" ]]; then
            echo "Missing mesh: $mesh" >&2
            missing=1
        fi
    done < <(grep -o 'filename="[^"]*"' "$out" | sed 's/^filename="//; s/"$//' | sort -u)
fi

links=$(grep -c '<link ' "$out" || true)
joints=$(grep -c '<joint ' "$out" || true)
collisions=$(grep -c '<collision>' "$out" || true)
echo "Wrote $out ($links links, $joints joints, $collisions collisions)"
exit $missing
