SUMMARY = "Device Tree Overlay for DDR reserved memory (frame buffer) and r5f2_0 firmware"
LICENSE = "CLOSED"

SRC_URI = "file://ddr_new_map.dtso \
           file://r5f2_0.dtso"

inherit allarch
DEPENDS += "dtc-native"

S = "${WORKDIR}/dev_tree_update"

do_compile:append(){
    dtc -@ -I dts -O dtb -o ${S}/ddr_new_map.dtbo ${WORKDIR}/sources-unpack/ddr_new_map.dtso
    dtc -@ -I dts -O dtb -o ${S}/r5f2_0.dtbo ${WORKDIR}/sources-unpack/r5f2_0.dtso
}


do_install:append() {
    install -d ${D}/boot/overlays

    install -m 0644 ${S}/ddr_new_map.dtbo ${D}/boot/overlays/ddr_new_map.dtbo
    install -m 0644 ${S}/r5f2_0.dtbo ${D}/boot/overlays/r5f2_0.dtbo
}

FILES:${PN} += "/boot/overlays/*.dtbo"