#!/bin/bash
set -e

echo "📦 Importing dependencies..."

# crea src se non esiste
mkdir -p src

# importa repo
vcs import src < dependencies.repos

echo "🔧 Building workspace..."

source /opt/ros/humble/setup.bash
colcon build --symlink-install

echo "🔌 Sourcing..."

source install/setup.bash

echo "✅ Setup completo!"