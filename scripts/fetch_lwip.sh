#!/bin/bash
# Download lwIP into lib/lwip (same layout as CI).
set -e

LWIP_VERSION="STABLE-2_2_0_RELEASE"
LWIP_DIR="lib/lwip"

if [ -f "$LWIP_DIR/src/core/init.c" ]; then
    echo "lwIP already present, skipping download."
    exit 0
fi

echo "Cloning lwIP ${LWIP_VERSION}..."
TMPDIR=$(mktemp -d)
git clone --depth 1 --branch "${LWIP_VERSION}" \
    https://github.com/lwip-tcpip/lwip.git "$TMPDIR/lwip"

mkdir -p "$LWIP_DIR"
cp -r "$TMPDIR/lwip/src" "$LWIP_DIR/"
rm -rf "$TMPDIR"
echo "lwIP downloaded to ${LWIP_DIR}"
