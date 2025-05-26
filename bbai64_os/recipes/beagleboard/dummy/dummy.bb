SUMMARY = "Dummy"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://dummy.sh \
           file://dummy.service"

do_install() {
    install -d ${D}${bindir}
    install -m 0755 ${WORKDIR}/dummy.sh ${D}${bindir}/dummy.sh

    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${WORKDIR}/dummy.service ${D}${systemd_system_unitdir}/
}

SYSTEMD_SERVICE:${PN} = "dummy.service"
inherit systemd
