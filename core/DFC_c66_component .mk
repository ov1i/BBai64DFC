dfc_c66_app_COMP_LIST = dfc_c66_app
dfc_c66_app_RELPATH = ti/drv/ipc/examples/DFC
dfc_c66_app_PATH = $(PDK_IPC_COMP_PATH)/examples/DFC
dfc_c66_app_BOARD_DEPENDENCY = yes
dfc_c66_app_CORE_DEPENDENCY = yes

dfc_c66_app_XDC_CONFIGURO = no
dfc_c66_app_MAKEFILE = -f makefile_c66 BUILD_OS_TYPE=baremetal
dfc_c66_app_PKG_LIST = dfc_c66_app
dfc_c66_app_INCLUDE = $(dfc_c66_app_PATH)
dfc_c66_app_BOARDLIST = j721e_evm
dfc_c66_app_$(SOC)_CORELIST = c66xdsp_1
ipc_EXAMPLE_LIST += dfc_c66_app

export dfc_c66_app_COMP_LIST
export dfc_c66_app_BOARD_DEPENDENCY
export dfc_c66_app_CORE_DEPENDENCY
export dfc_c66_app_XDC_CONFIGURO
export dfc_c66_app_BOARDLIST
export dfc_c66_app_$(SOC)_CORELIST
export dfc_c66_app_SBL_APPIMAGEGEN = yes
