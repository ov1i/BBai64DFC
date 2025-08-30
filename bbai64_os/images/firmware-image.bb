SUMMARY = "BB AI-64 OS Firmware Image"
LICENSE = "MIT"

inherit core-image

IMAGE_FEATURES += "ssh-server-dropbear"
IMAGE_INSTALL += " \
packagegroup-core-boot \
kernel-modules \
sudo \
systemd \
systemd-networkd \
systemd-analyze \
i2c-tools \
strace \
iproute2 \
network-remote-init \
xz \
ti-ipc \
firmwares \
env-init \
auto-flasher-emmc \
"
EXTRA_IMAGE_FEATURES ?= " allow-empty-password empty-root-password allow-root-login"

SYSTEMD_AUTO_ENABLE = "enable"


IMAGE_ROOTFS_SIZE ?= "8192"
WKS_FILE = "sdimage-2part.wks"
IMAGE_ROOTFS_EXTRA_SPACE:append = "${@bb.utils.contains("DISTRO_FEATURES", "systemd", " + 4096", "", d)}"
