dfc_app_COMP_LIST = dfc_app
dfc_app_RELPATH = ti/drv/ipc/examples/DFC
dfc_app_PATH = $(PDK_IPC_COMP_PATH)/examples/DFC
dfc_app_BOARD_DEPENDENCY = yes
dfc_app_CORE_DEPENDENCY = yes

dfc_app_XDC_CONFIGURO = no
dfc_app_MAKEFILE = -f makefile_r5 BUILD_OS_TYPE=freertos
dfc_app_PKG_LIST = dfc_app
dfc_app_INCLUDE = $(dfc_app_PATH)
dfc_app_BOARDLIST = j721e_evm
dfc_app_$(SOC)_CORELIST = mcu2_0
ipc_EXAMPLE_LIST += dfc_app

export dfc_app_COMP_LIST
export dfc_app_BOARD_DEPENDENCY
export dfc_app_CORE_DEPENDENCY
export dfc_app_XDC_CONFIGURO
export dfc_app_BOARDLIST
export dfc_app_$(SOC)_CORELIST
export dfc_app_SBL_APPIMAGEGEN = yes
