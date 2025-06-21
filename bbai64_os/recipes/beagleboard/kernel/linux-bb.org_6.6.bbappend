COMPATIBLE_MACHINE:append = "|dfc|dfc-k3r5"

FILESEXTRAPATHS:prepend := "${THISDIR}/files:"
SRC_URI += "file://extras.dtso \
            file://k3-j721e-beagleboneai64.dts"

do_configure:append() {
    cp ${WORKDIR}/sources-unpack/extras.dtso ${S}/arch/arm64/boot/dts/ti/extras.dtso
    grep -q 'dtb-$(CONFIG_ARCH_K3) += extras.dtbo' ${S}/arch/arm64/boot/dts/ti/Makefile \
        || echo 'dtb-$(CONFIG_ARCH_K3) += extras.dtbo' >> ${S}/arch/arm64/boot/dts/ti/Makefile


    # Should act as temporary workaround for parsing failure of the fdtoverlays paths
    # cp ${WORKDIR}/sources-unpack/k3-j721e-beagleboneai64.dts ${S}/arch/arm64/boot/dts/ti/k3-j721e-beagleboneai64.dts
    # Should act as temporary workaround for parsing failure of the fdtoverlays paths
}
