#!/bin/sh
set -eu

log(){ echo "[flash-to-emmc] $*"; }
fail(){ echo "[flash-to-emmc][ERR] $*" >&2; exit 1; }

# Detect SD vs eMMC via /sys removable flag
detect_devs() {
  SD=""
  EMMC=""
  for b in /sys/block/mmcblk*; do
    [ -d "$b" ] || continue
    name=$(basename "$b")
    if [ -f "$b/removable" ] && [ "$(cat "$b/removable")" = "1" ]; then
      SD="/dev/$name"
    else
      EMMC="/dev/$name"
    fi
  done

  [ -n "$SD" ]   || fail "Could not detect SD device"
  [ -n "$EMMC" ] || fail "Could not detect eMMC device"

  # refuse to run if rootfs is already on eMMC
  rootdev=$(findmnt -no SOURCE / | sed 's/[0-9]*$//')
  if [ "$rootdev" = "$EMMC" ]; then
    fail "Already running from eMMC ($EMMC); aborting."
  fi

  echo "$SD" "$EMMC"
}

SD_DEV="" EMMC_DEV=""
read SD_DEV EMMC_DEV <<EOF
$(detect_devs)
EOF
log "SD=$SD_DEV  eMMC=$EMMC_DEV"

# Unmount any mounted eMMC partitions
for p in $(lsblk -ln -o NAME "$EMMC_DEV" | tail -n +2); do
  m="/dev/$p"
  if mount | grep -q "^$m "; then
    umount -f "$m" || true
  fi
done

# Partition eMMC: GPT, 256MiB FAT32 boot + rest ext4
log "Partitioning $EMMC_DEV"
parted -s "$EMMC_DEV" mklabel gpt
parted -s "$EMMC_DEV" mkpart boot fat32 1MiB 257MiB
parted -s "$EMMC_DEV" set 1 boot on
parted -s "$EMMC_DEV" mkpart rootfs ext4 257MiB 100%

# Wait for kernel to create nodes
udevadm settle
BOOT_PART="${EMMC_DEV}p1"
ROOT_PART="${EMMC_DEV}p2"

# Make filesystems
log "Creating filesystems"
mkfs.vfat -F32 -n BOOT "$BOOT_PART"
mkfs.ext4 -F -L rootfs "$ROOT_PART"

# Mount target
mkdir -p /mnt/emmc-boot /mnt/emmc-root
mount "$ROOT_PART" /mnt/emmc-root
mkdir -p /mnt/emmc-root/boot
mount "$BOOT_PART" /mnt/emmc-boot

# Rsync live rootfs → eMMC (stay on same filesystem with -x)
log "Syncing rootfs (this can take a while)…"
rsync -aHAXx --delete \
  --exclude="/mnt/*" \
  --exclude="/proc/*" \
  --exclude="/sys/*" \
  --exclude="/dev/*" \
  --exclude="/run/*" \
  --exclude="/tmp/*" \
  / /mnt/emmc-root/

# Copy /boot (kernels, DTBs, extlinux/uEnv, etc.)
log "Copying /boot"
rsync -aHAX --delete /boot/ /mnt/emmc-boot/
sync

# Marker file on eMMC so first boot can detect it came from SD cloning
date > /mnt/emmc-root/etc/EMMC_CLONED_FROM_SD

umount /mnt/emmc-boot || true
umount /mnt/emmc-root || true
rmdir /mnt/emmc-boot /mnt/emmc-root || true

log "Done. You can power-cycle and switch U-Boot to boot from eMMC."
exit 0
