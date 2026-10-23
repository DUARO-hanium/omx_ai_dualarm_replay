#!/usr/bin/env bash
set -euo pipefail
source /etc/os-release
[[ $ID == ubuntu && $VERSION_ID == 24.04 && $(uname -m) == x86_64 ]] || {
    echo 'Automatic installation targets Ubuntu 24.04 x86_64.' >&2; exit 1;
}
if (( EUID == 0 )); then elevate=(); else elevate=(sudo); fi
"${elevate[@]}" apt-get update
"${elevate[@]}" apt-get install -y ca-certificates curl
task_tmp=$(mktemp -d)
trap 'rm -rf "$task_tmp"' EXIT
curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o "$task_tmp/docker.asc"
"${elevate[@]}" install -d -m 0755 /etc/apt/keyrings
"${elevate[@]}" install -m 0644 "$task_tmp/docker.asc" /etc/apt/keyrings/docker.asc
cat > "$task_tmp/docker.sources" <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: ${UBUNTU_CODENAME:-$VERSION_CODENAME}
Components: stable
Architectures: amd64
Signed-By: /etc/apt/keyrings/docker.asc
EOF
"${elevate[@]}" install -m 0644 "$task_tmp/docker.sources" /etc/apt/sources.list.d/docker.sources
"${elevate[@]}" apt-get update
"${elevate[@]}" apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin
"${elevate[@]}" systemctl enable --now docker
