#!/bin/bash
# Automatically get the parent directory of the script, i.e., the root of the repository
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

echo "Starting Docker build from repository root: $REPO_ROOT"

# Use -f to specify the Dockerfile path, and use the repository root as the build context
docker build -f docker/Dockerfile.humble-cuda -t exact-mppi:humble-cuda .
