#!/bin/bash

# Workspace Restoration Script for Ubuntu 20.04.06
# This script restores your development environment setup

echo "🚀 Restoring workspace setup..."

# Check if Windows-SSD is mounted, if not, mount it
echo "🔍 Checking if Windows-SSD is mounted..."
if [ ! -d "/media/danielterra/Windows-SSD" ] || [ -z "$(ls -A /media/danielterra/Windows-SSD 2>/dev/null)" ]; then
    echo "📁 Windows-SSD not mounted. Attempting to mount..."

    # Create mount point if it doesn't exist
    sudo mkdir -p /media/danielterra/Windows-SSD

    # Mount the specific Windows-SSD partition
    echo "🔧 Mounting /dev/nvme1n1p3 (Windows-SSD)..."
    sudo mount /dev/nvme1n1p3 /media/danielterra/Windows-SSD

    if [ $? -eq 0 ]; then
        echo "✅ Windows-SSD mounted successfully!"
    else
        echo "❌ Failed to mount Windows-SSD."
        echo "You may need to mount it manually with:"
        echo "sudo mount /dev/nvme1n1p3 /media/danielterra/Windows-SSD"
        exit 1
    fi
else
    echo "✅ Windows-SSD is already mounted!"
fi# Wait a moment for the mount to settle
sleep 2

# Navigate to project directory
PROJECT_DIR="/media/danielterra/Windows-SSD/Users/danie/Documents/Documents/MESTRADO/25-2_robotica-movel"
if [ ! -d "$PROJECT_DIR" ]; then
    echo "❌ Project directory not found: $PROJECT_DIR"
    echo "Please check if Windows-SSD is properly mounted."
    exit 1
fi

cd "$PROJECT_DIR"
echo "📂 Changed to project directory: $(pwd)"

# Open VS Code with the workspace
echo "📝 Opening VS Code workspace..."
code . &

# Wait a moment for VS Code to start
sleep 3

# Open file manager to the project folder
echo "📁 Opening file manager..."
nautilus . &

# Open Chrome (you can add specific URLs if needed)
echo "🌐 Opening Chrome..."
google-chrome &

# Open CoppeliaSim with T1.ttt project
echo "🤖 Opening CoppeliaSim with T1.ttt project..."
cd /home/danielterra/Documents/programs/CoppeliaSim_Edu_V4_9_0_rev6_Ubuntu20_04
./coppeliaSim.sh "/media/danielterra/Windows-SSD/Users/danie/Documents/Documents/MESTRADO/25-2_robotica-movel/T1/T1.ttt" &
cd "/media/danielterra/Windows-SSD/Users/danie/Documents/Documents/MESTRADO/25-2_robotica-movel"

# Open terminals with specific configurations
echo "💻 Opening terminals..."
# Terminal 1 - Main project terminal
gnome-terminal --working-directory="$(pwd)" --title="Main Terminal" &

# Terminal 2 - Additional terminal
gnome-terminal --working-directory="$(pwd)" --title="Secondary Terminal" &

echo "✅ Workspace restoration complete!"
echo "All applications should be starting up now."
