#!/usr/bin/env bash
# Create and push a signed tag for a release on the develop commit merged into origin/main.
# Usage: scripts/tag_release.sh <version>   e.g. scripts/tag_release.sh 3.18.18
set -euo pipefail

if [ $# -ne 1 ]; then
    echo "Usage: $0 <version>" >&2
    exit 1
fi

version="$1"
if ! [[ ${version} =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "invalid version: ${version}" >&2
    exit 1
fi

git fetch origin main --tags

if git rev-parse -q --verify "refs/tags/${version}" >/dev/null; then
    echo "tag ${version} already exists locally. Delete it first: git tag -d ${version}" >&2
    exit 1
fi
if git ls-remote --exit-code --tags origin "refs/tags/${version}" >/dev/null 2>&1; then
    echo "tag ${version} already exists on origin" >&2
    exit 1
fi

# origin/main is a merge commit of develop. Tag the merged develop commit (the second parent)
# so that the tag is reachable from develop (for git describe and catkin_generate_changelog).
if ! target="$(git rev-parse -q --verify origin/main^2)"; then
    echo "origin/main is not a merge commit. Is the develop to main pull request merged with a merge commit?" >&2
    exit 1
fi

# the version on the merged develop commit must match the requested version
for pkg in driving_log_replayer_v2 driving_log_replayer_v2_analyzer driving_log_replayer_v2_msgs; do
    pkg_version="$(git show "${target}:${pkg}/package.xml" | sed -n 's|.*<version>\(.*\)</version>.*|\1|p' | head -n 1)"
    if [ "${pkg_version}" != "${version}" ]; then
        echo "${pkg}/package.xml version is ${pkg_version} on the second parent of origin/main, expected ${version}. Is the release PR merged?" >&2
        exit 1
    fi
done

echo "Tag ${version} will be created on the develop commit merged into origin/main:"
git log -1 --oneline "${target}"
read -r -p "Continue? [y/N] " answer
if [ "${answer}" != "y" ]; then
    echo "aborted"
    exit 1
fi

git tag -s "${version}" "${target}" -m "${version}"
git push origin "refs/tags/${version}:refs/tags/${version}"
