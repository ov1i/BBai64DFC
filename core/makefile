ifeq ($(RULES_MAKE), )
include $(PDK_INSTALL_PATH)/ti/build/Rules.make
else
include $(RULES_MAKE)
endif

APP_NAME = dfc_app

SRCDIR += . src src/common src/common/clusters src/common/clusters_configs src/common/clusters_configs/$(ISA) src/core_r5f2_0
INCDIR += interface interface/common
PACKAGE_SRCS_COMMON = .

CFLAGS_LOCAL_COMMON += -DFREERTOS $(PDK_CFLAGS)
INCLUDE_EXTERNAL_INTERFACES += pdk freertos

COMP_LIST_COMMON = $(PDK_COMMON_FREERTOS_COMP)

SRCS_COMMON += rtos_main.c rsc_table.c

ifeq ($(ISA), r5f)
	SRCS_COMMON += r5f_mpu_default.c
endif

ifeq ($(ISA), c66)
	SRCS_COMMON += c66_cache_mar.c
endif

ifeq ($(ISA), c7x)
    SRCS_COMMON += c7x_mmu.c
endif

EXTERNAL_LNKCMD_FILE_LOCAL = $(PDK_INSTALL_PATH)/ti/drv/ipc/examples/DFC/src/common/clusters/$(ISA)/lnk_$(ISA)_$(CORE).lds
APPEND_LNKCMD_FILE += $(PDK_INSTALL_PATH)/ti/drv/ipc/examples/DFC/src/common/clusters/memory_map_ddr.cmd

ifeq ($(MAKERULEDIR), )
  MAKERULEDIR := $(ROOTDIR)/ti/build/makerules
  export MAKERULEDIR
endif
include $(MAKERULEDIR)/common.mk
