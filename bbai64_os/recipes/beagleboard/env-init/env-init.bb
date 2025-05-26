SUMMARY = "Install static extlinux.conf to boot partition"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI += "file://extlinux.conf"

inherit allarch

do_install:append() {
    install -d ${D}/boot/extlinux
    install -d ${DEPLOY_DIR_IMAGE}/extlinux

    install -m 0644 ${WORKDIR}/sources-unpack/extlinux.conf ${DEPLOY_DIR_IMAGE}/extlinux/extlinux.conf
    install -m 0644 ${WORKDIR}/sources-unpack/extlinux.conf ${D}/boot/extlinux/extlinux.conf
}

# do_deploy:append() {
#     cp ${WORKDIR}/sources-unpack/extlinux.conf ${DEPLOY_DIR_IMAGE}/extlinux.conf
# }

FILES:${PN} += "/boot/extlinux/extlinux.conf"