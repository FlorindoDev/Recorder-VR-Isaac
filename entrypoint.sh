#!/usr/bin/env bash
set -e

# Genera un registry coerente per il container
mkdir -p /root/.config/openvr
cat << EOF > /root/.config/openvr/openvrpaths.vrpath
{
  "runtime": [
    "/home/steam/steamvr"
  ]
  "version": 1
}
EOF
chmod 644 /root/.config/openvr/openvrpaths.vrpath

exec bash
