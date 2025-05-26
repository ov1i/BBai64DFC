#!/bin/sh
set -e

echo "Starting eMMC flash..."

# Abort if we're running from eMMC
if grep -q mmcblk0 /proc/cmdline; then
  echo "You're running from eMMC! Aborting to prevent overwrite."
  exit 1
fi

EMMC_DEV="/dev/mmcblk0"

# Clear first few MB of eMMC
dd if=/dev/zero of=$EMMC_DEV bs=1M count=10

# Partition eMMC: 128MB boot (FAT), rest rootfs (ext4)
parted $EMMC_DEV --script -- mklabel msdos
parted $EMMC_DEV --script -- mkpart primary fat32 1MiB 128MiB
parted $EMMC_DEV --script -- mkpart primary ext4 128MiB 100%

mkfs.vfat ${EMMC_DEV}p1
mkfs.ext4 ${EMMC_DEV}p2

# Mount
mkdir -p /mnt/emmc_boot /mnt/emmc_root
mount ${EMMC_DEV}p1 /mnt/emmc_boot
mount ${EMMC_DEV}p2 /mnt/emmc_root

# Copy boot files
cp -v /boot/* /mnt/emmc_boot/

# Sync rootfs
rsync -aAXv / /mnt/emmc_root \
  --exclude=/mnt --exclude=/proc --exclude=/sys \
  --exclude=/dev --exclude=/tmp --exclude=/run \
  --exclude=/var/lib/flashed.flag

# Create flag
mkdir -p /mnt/emmc_root/var/lib
touch /mnt/emmc_root/var/lib/flashed.flag

sync
umount /mnt/emmc_boot /mnt/emmc_root

echo "Flash complete. Power off and remove SD to boot from eMMC."
