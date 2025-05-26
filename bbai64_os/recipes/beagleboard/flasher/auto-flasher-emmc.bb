SUMMARY = "Auto flasher to eMMC recipe"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI = "file://flash-to-emmc.sh \
           file://flash-to-emmc.service"

inherit systemd

SYSTEMD_SERVICE:${PN} = "flash-to-emmc.service"

do_install:append() {
    install -d ${D}${bindir}
    install -m 0755 ${WORKDIR}/sources-unpack/flash-to-emmc.sh ${D}${bindir}/flash-to-emmc.sh

    install -d ${D}${sysconfdir}/systemd/system
    install -m 0644 ${WORKDIR}/sources-unpack/flash-to-emmc.service ${D}${sysconfdir}/systemd/system/
}
