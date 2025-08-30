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
${TOPDIR}/firmwares/dfc_c66_app/bin/j721e_evm/dfc_c66_app_c66xdsp_1_release.xe66 \
${TOPDIR}/firmwares/dfc_c66_app/bin/j721e_evm/dfc_c66_app_c66xdsp_1_release_strip.xe66 \
"

SRC_URI += " \
    file://dfc-wait-net.sh \
    file://dfc-isp.conf \
    file://dfc-isp.service \
    file://dfc-img-udp.conf \
    file://dfc-img-udp.service \
    file://dfc-tele-udp.conf \
    file://dfc-tele-udp.service \
    file://dfc-calib-udp.service \
    file://dfc-calib-udp.conf \
    file://blacklist-rpmsg.conf \
    file://virtio-rpmsg-after-rprocs.service \
"

SYSTEMD_PACKAGES = "${PN}"
SYSTEMD_AUTO_ENABLE:${PN} = "enable"
SYSTEMD_SERVICE:${PN} = "dfc-isp.service dfc-img-udp.service dfc-tele-udp.service dfc-calib-udp.service virtio-rpmsg-after-rprocs.service"

RDEPENDS:${PN} += "bash iproute2 coreutils grep sed iputils-ping"

do_install:append() {
    install -d ${D}${nonarch_base_libdir}/firmware/ti-eth/j721e

    install -m 0644 ${FIRMWARE_SRC_PATH} ${D}${nonarch_base_libdir}/firmware/ti-eth/j721e

    rm -rf ${D}${nonarch_base_libdir}/firmware/j7-main-r5f0_0-fw
    ln -sf ti-eth/j721e/dfc_app_mcu2_0_release.xer5f ${D}${nonarch_base_libdir}/firmware/j7-main-r5f0_0-fw
    
    ln -sf ti-eth/j721e/dfc_c66_app_c66xdsp_1_release.xe66 ${D}${nonarch_base_libdir}/firmware/j7-c66_0-fw

    install -d ${D}${sbindir}
    install -m 0755 ${WORKDIR}/sources-unpack/dfc-wait-net.sh           ${D}${sbindir}/dfc-wait-net

    install -d ${D}${sysconfdir}/default
    install -m 0644 ${WORKDIR}/sources-unpack/dfc-isp.conf                          ${D}${sysconfdir}/default/
    install -m 0644 ${WORKDIR}/sources-unpack/dfc-img-udp.conf                      ${D}${sysconfdir}/default/
    install -m 0644 ${WORKDIR}/sources-unpack/dfc-tele-udp.conf                     ${D}${sysconfdir}/default/
    install -m 0644 ${WORKDIR}/sources-unpack/dfc-calib-udp.conf                    ${D}${sysconfdir}/default/


    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${WORKDIR}/sources-unpack/dfc-isp.service                       ${D}${systemd_system_unitdir}/
    install -m 0644 ${WORKDIR}/sources-unpack/dfc-img-udp.service                   ${D}${systemd_system_unitdir}/
    install -m 0644 ${WORKDIR}/sources-unpack/dfc-tele-udp.service                  ${D}${systemd_system_unitdir}/
    install -m 0644 ${WORKDIR}/sources-unpack/dfc-calib-udp.service                 ${D}${systemd_system_unitdir}/
    install -m 0644 ${WORKDIR}/sources-unpack/virtio-rpmsg-after-rprocs.service     ${D}${systemd_system_unitdir}/

    install -d ${D}${sysconfdir}/modprobe.d
    install -m 0644 ${WORKDIR}/sources-unpack/blacklist-rpmsg.conf                  ${D}${sysconfdir}/modprobe.d/

}

FILES:${PN} += " \
    ${nonarch_base_libdir}/firmware/ti-eth/j721e/dfc_c66_app_c66xdsp_1_release.xe66 \
    ${nonarch_base_libdir}/firmware/ti-eth/j721e/dfc_c66_app_c66xdsp_1_release_strip.xe66 \
    ${nonarch_base_libdir}/firmware/ti-eth/j721e/dfc_app_mcu2_0_release.xer5f \
    ${nonarch_base_libdir}/firmware/ti-eth/j721e/dfc_app_mcu2_0_release_strip.xer5f \
    ${nonarch_base_libdir}/firmware/j7-main-r5f0_0-fw \
    ${nonarch_base_libdir}/firmware/j7-c66_0-fw \
    ${sbindir}/dfc-wait-net \
    ${sysconfdir}/default/dfc-isp.conf \
    ${sysconfdir}/default/dfc-img-udp.conf \
    ${sysconfdir}/default/dfc-tele-udp.conf \
    ${sysconfdir}/default/dfc-calib-udp.conf \
    ${sysconfdir}/modprob.d/blacklist-rpmsg.conf \
    ${systemd_system_unitdir}/dfc-isp.service \
    ${systemd_system_unitdir}/dfc-img-udp.service \
    ${systemd_system_unitdir}/dfc-tele-udp.service \
    ${systemd_system_unitdir}/dfc-calib-udp.service \
"