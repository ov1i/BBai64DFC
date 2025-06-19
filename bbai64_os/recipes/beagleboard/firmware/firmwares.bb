SUMMARY = "Install Custom firmwares"
LICENSE = "CLOSED"
PR = "r1"

do_fetch[noexec] = "1"
INHIBIT_PACKAGE_ARCH = "1"
INSANE_SKIP:${PN} += "arch"


FIRMWARE_SRC_PATH := " \
${TOPDIR}/firmwares/dfc_app/bin/j721e_evm/dfc_app_mcu2_0_release.xer5f \
${TOPDIR}/firmwares/dfc_app/bin/j721e_evm/dfc_app_mcu2_0_release_strip.xer5f \
"

do_install() {
    install -d ${D}${nonarch_base_libdir}/firmware/ti-eth/j721e

    install -m 0644 ${FIRMWARE_SRC_PATH} ${D}${nonarch_base_libdir}/firmware/ti-eth/j721e

    rm -rf ${D}${nonarch_base_libdir}/firmware/j7-main-r5f0_0-fw
    ln -sf ti-eth/j721e/dfc_app_mcu2_0_release.xer5f ${D}${nonarch_base_libdir}/firmware/j7-main-r5f0_0-fw
}

FILES:${PN} += "${nonarch_base_libdir}/firmware/ti-eth/j721e/dfc_app_mcu2_0_release.xer5f \ 
                ${nonarch_base_libdir}/firmware/ti-eth/j721e/dfc_app_mcu2_0_release_strip.xer5f \
                ${nonarch_base_libdir}/firmware/j7-main-r5f0_0-fw"
