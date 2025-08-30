SUMMARY = "Auto flasher to eMMC recipe"
LICENSE = "CLOSED"

FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI = "file://flash-to-emmc.sh \
           file://flash-to-emmc.service"

inherit systemd

do_install:append() {
    install -d ${D}/usr/sbin
    install -m 0755 ${WORKDIR}/sources-unpack/flash-to-emmc.sh ${D}/usr/sbin/flash-to-emmc

    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${WORKDIR}/sources-unpack/flash-to-emmc.service ${D}${systemd_system_unitdir}/
}

FILES:${PN} += " \
    /usr/sbin/flash-to-emmc \
    ${systemd_system_unitdir}/flash-to-emmc.service \
"

SYSTEMD_SERVICE:${PN} = "flash-to-emmc.service"
SYSTEMD_AUTO_ENABLE:${PN} = "enable"
