#!/bin/bash

# Auto-mount setup script for Windows-SSD
# This script configures automatic mounting of Windows-SSD at boot

echo "🔧 Setting up auto-mount for Windows-SSD..."

# Get the UUID of the Windows-SSD partition
UUID=$(sudo blkid /dev/nvme1n1p3 | grep -oP 'UUID="\K[^"]+')

if [ -z "$UUID" ]; then
    echo "❌ Could not find UUID for Windows-SSD partition"
    exit 1
fi

echo "📋 Found Windows-SSD UUID: $UUID"

# Create mount point if it doesn't exist
sudo mkdir -p /media/danielterra/Windows-SSD

# Backup current fstab
sudo cp /etc/fstab /etc/fstab.backup.$(date +%Y%m%d_%H%M%S)
echo "💾 Backed up current fstab"

# Add entry to fstab for auto-mounting
FSTAB_ENTRY="UUID=$UUID /media/danielterra/Windows-SSD ntfs defaults,uid=1000,gid=1000,umask=022,auto,rw 0 0"

echo "📝 Adding to fstab:"
echo "$FSTAB_ENTRY"

echo "$FSTAB_ENTRY" | sudo tee -a /etc/fstab

# Test the fstab entry
echo "🧪 Testing fstab configuration..."
sudo mount -a

if mountpoint -q /media/danielterra/Windows-SSD; then
    echo "✅ Windows-SSD auto-mount configured successfully!"
    echo "🎉 The drive will now mount automatically at boot."
else
    echo "❌ Auto-mount test failed. Restoring backup..."
    sudo mv /etc/fstab.backup.$(date +%Y%m%d)* /etc/fstab
    exit 1
fi

echo ""
echo "🔍 Current mount status:"
df -h | grep Windows-SSD
