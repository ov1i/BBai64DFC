COMPATIBLE_MACHINE:append = "|dfc|dfc-k3r5"

FILESEXTRAPATHS:prepend := "${THISDIR}/files:"
SRC_URI += "file://extras.dtso \
            file://extras_v1.dtso \
            file://extras_v2.dtso \
            file://extras_v3.dtso \
            file://extras_v4.dtso \
            file://extras_v5.dtso \
            file://imx219_csi0.dtso \
            file://imx219_csi1.dtso \
            file://k3-j721e-beagleboneai64.dts"

do_configure:append() {
    cp ${WORKDIR}/sources-unpack/extras.dtso ${S}/arch/arm64/boot/dts/ti/extras.dtso
    cp ${WORKDIR}/sources-unpack/extras_v1.dtso ${S}/arch/arm64/boot/dts/ti/extras_v1.dtso
    cp ${WORKDIR}/sources-unpack/extras_v2.dtso ${S}/arch/arm64/boot/dts/ti/extras_v2.dtso
    cp ${WORKDIR}/sources-unpack/extras_v3.dtso ${S}/arch/arm64/boot/dts/ti/extras_v3.dtso
    cp ${WORKDIR}/sources-unpack/extras_v4.dtso ${S}/arch/arm64/boot/dts/ti/extras_v4.dtso
    cp ${WORKDIR}/sources-unpack/extras_v5.dtso ${S}/arch/arm64/boot/dts/ti/extras_v5.dtso
    cp ${WORKDIR}/sources-unpack/imx219_csi0.dtso ${S}/arch/arm64/boot/dts/ti/imx219_csi0.dtso
    cp ${WORKDIR}/sources-unpack/imx219_csi1.dtso ${S}/arch/arm64/boot/dts/ti/imx219_csi1.dtso

    grep -q 'dtb-$(CONFIG_ARCH_K3) += extras.dtbo' ${S}/arch/arm64/boot/dts/ti/Makefile \
        || echo 'dtb-$(CONFIG_ARCH_K3) += extras.dtbo' >> ${S}/arch/arm64/boot/dts/ti/Makefile
    grep -q 'dtb-$(CONFIG_ARCH_K3) += extras_v1.dtbo' ${S}/arch/arm64/boot/dts/ti/Makefile \
        || echo 'dtb-$(CONFIG_ARCH_K3) += extras_v1.dtbo' >> ${S}/arch/arm64/boot/dts/ti/Makefile
    grep -q 'dtb-$(CONFIG_ARCH_K3) += extras_v2.dtbo' ${S}/arch/arm64/boot/dts/ti/Makefile \
        || echo 'dtb-$(CONFIG_ARCH_K3) += extras_v2.dtbo' >> ${S}/arch/arm64/boot/dts/ti/Makefile
    grep -q 'dtb-$(CONFIG_ARCH_K3) += extras_v3.dtbo' ${S}/arch/arm64/boot/dts/ti/Makefile \
        || echo 'dtb-$(CONFIG_ARCH_K3) += extras_v3.dtbo' >> ${S}/arch/arm64/boot/dts/ti/Makefile
    grep -q 'dtb-$(CONFIG_ARCH_K3) += extras_v4.dtbo' ${S}/arch/arm64/boot/dts/ti/Makefile \
        || echo 'dtb-$(CONFIG_ARCH_K3) += extras_v4.dtbo' >> ${S}/arch/arm64/boot/dts/ti/Makefile
    grep -q 'dtb-$(CONFIG_ARCH_K3) += extras_v5.dtbo' ${S}/arch/arm64/boot/dts/ti/Makefile \
        || echo 'dtb-$(CONFIG_ARCH_K3) += extras_v5.dtbo' >> ${S}/arch/arm64/boot/dts/ti/Makefile
    grep -q 'dtb-$(CONFIG_ARCH_K3) += imx219_csi0.dtbo' ${S}/arch/arm64/boot/dts/ti/Makefile \
        || echo 'dtb-$(CONFIG_ARCH_K3) += imx219_csi0.dtbo' >> ${S}/arch/arm64/boot/dts/ti/Makefile
    grep -q 'dtb-$(CONFIG_ARCH_K3) += imx219_csi1.dtbo' ${S}/arch/arm64/boot/dts/ti/Makefile \
        || echo 'dtb-$(CONFIG_ARCH_K3) += imx219_csi1.dtbo' >> ${S}/arch/arm64/boot/dts/ti/Makefile


    # Should act as temporary workaround for parsing failure of the fdtoverlays paths
    cp ${WORKDIR}/sources-unpack/k3-j721e-beagleboneai64.dts ${S}/arch/arm64/boot/dts/ti/k3-j721e-beagleboneai64.dts
    # Should act as temporary workaround for parsing failure of the fdtoverlays paths
}
