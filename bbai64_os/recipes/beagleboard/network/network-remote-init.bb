SUMMARY = "Static IP config for usb0 and eth0"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

COMPATIBLE_MACHINE = "dfc"

S = "${WORKDIR}/network"
SRC_URI += "file://usb0.network \
           file://10_eth0.network"

do_install:append() {
    install -d ${D}${sysconfdir}/systemd/network
    install -m 0644 ${WORKDIR}/sources-unpack/usb0.network ${D}${sysconfdir}/systemd/network/
    install -m 0644 ${WORKDIR}/sources-unpack/10_eth0.network ${D}${sysconfdir}/systemd/network/

    install -d ${D}${sysconfdir}/modules-load.d
    echo "g_ether" > ${D}${sysconfdir}/modules-load.d/g_ether.conf
}

FILES:${PN} += "${sysconfdir}/systemd/network"
FILES:${PN} += "${sysconfdir}/modules-load.d/g_ether.conf"
