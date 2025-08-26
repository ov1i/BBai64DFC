SUMMARY = "Install Custom firmwares"
LICENSE = "CLOSED"
PR = "r1"

inherit systemd cmake externalsrc 

do_fetch[noexec] = "1"
INHIBIT_PACKAGE_ARCH = "1"
INSANE_SKIP:${PN} += "arch"

EXTERNALSRC = "${TOPDIR}/../core_a72"
EXTERNALSRC_BUILD = "${WORKDIR}/build"
EXTERNALSRC_SYMLINKS = " "

FIRMWARE_SRC_PATH := " \
${TOPDIR}/firmwares/dfc_app/bin/j721e_evm/dfc_app_mcu2_0_release.xer5f \
${TOPDIR}/firmwares/dfc_app/bin/j721e_evm/dfc_app_mcu2_0_release_strip.xer5f \
"

SRC_URI += " \
    file://dfc-wait-net.sh \
    file://dfc-isp.conf \
    file://dfc-img-udp.conf \
    file://dfc-tele.conf \
    file://dfc-isp.service \
    file://dfc-img-udp.service \
    file://dfc-tele-tx.service \
"

SYSTEMD_PACKAGES = "${PN}"
SYSTEMD_AUTO_ENABLE:${PN} = "enable"
SYSTEMD_SERVICE:${PN} = "dfc-isp.service dfc-img-udp.service dfc-tele-tx.service"

RDEPENDS:${PN} += "bash iproute2 coreutils grep sed iputils-ping"

do_install:append() {
    install -d ${D}${nonarch_base_libdir}/firmware/ti-eth/j721e

    install -m 0644 ${FIRMWARE_SRC_PATH} ${D}${nonarch_base_libdir}/firmware/ti-eth/j721e

    rm -rf ${D}${nonarch_base_libdir}/firmware/j7-main-r5f0_0-fw
    ln -sf ti-eth/j721e/dfc_app_mcu2_0_release.xer5f ${D}${nonarch_base_libdir}/firmware/j7-main-r5f0_0-fw

    install -d ${D}${sbindir}
    install -m 0755 ${WORKDIR}/dfc-wait-net.sh   ${D}${sbindir}/dfc-wait-net
    install -m 0755 ${WORKDIR}/dfc-wait-rpmsg.sh ${D}${sbindir}/dfc-wait-rpmsg

    install -d ${D}${sysconfdir}/default
    install -m 0644 ${WORKDIR}/dfc-isp.conf      ${D}${sysconfdir}/default/
    install -m 0644 ${WORKDIR}/dfc-img-udp.conf  ${D}${sysconfdir}/default/
    install -m 0644 ${WORKDIR}/dfc-tele.conf     ${D}${sysconfdir}/default/

    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${WORKDIR}/dfc-isp.service         ${D}${systemd_system_unitdir}/
    install -m 0644 ${WORKDIR}/dfc-img-udp.service     ${D}${systemd_system_unitdir}/
    install -m 0644 ${WORKDIR}/dfc-tele-tx.service     ${D}${systemd_system_unitdir}/
}

FILES:${PN} += " \
    ${nonarch_base_libdir}/firmware/ti-eth/j721e/dfc_app_mcu2_0_release.xer5f \
    ${nonarch_base_libdir}/firmware/ti-eth/j721e/dfc_app_mcu2_0_release_strip.xer5f \
    ${nonarch_base_libdir}/firmware/j7-main-r5f0_0-fw \
    ${sbindir}/dfc-wait-net \
    ${sysconfdir}/default/dfc-isp.conf \
    ${sysconfdir}/default/dfc-img-udp.conf \
    ${sysconfdir}/default/dfc-tele.conf \
    ${systemd_system_unitdir}/dfc-isp.service \
    ${systemd_system_unitdir}/dfc-img-udp.service \
    ${systemd_system_unitdir}/dfc-tele-tx.service \
"