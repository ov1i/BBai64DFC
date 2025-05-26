SUMMARY = "Enable dropbear SSH server on boot"
LICENSE = "MIT"
PR = "r0"

SRC_URI += ""

S = "${WORKDIR}/ssh_server"

inherit systemd

SYSTEMD_SERVICE:${PN} = "dropbear.service"

do_install:append() {
    install -d ${D}${systemd_system_unitdir}
    ln -sf /etc/init.d/dropbear ${D}${systemd_system_unitdir}/dropbear.service
}

FILES:${PN} += "${systemd_system_unitdir}/dropbear.service"