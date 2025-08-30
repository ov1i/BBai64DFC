#!/bin/bash
clean_artifacts_full() {
    echo "[F]Build cleanup started.."
    if [ -d ".build" ]; then
        sudo rm -rf .build
    fi
    if [ -d "logs" ]; then
        sudo rm -rf logs
    fi
    echo "Build cleanup finished.."
}

clean_artifacts_lite() {
    echo "[L]Build cleanup started.."
    if [ -d ".build/tmp" ]; then
        sudo rm -rf .build/tmp
    fi

    if [ -d ".build/tmp-k3r5" ]; then
        sudo rm -rf .build/tmp-k3r5
    fi

    if [ -d ".build/deploy-ti" ]; then
        sudo rm -rf .build/deploy-ti
    fi
    echo "Build cleanup finished.."
}
sync() {
    SDK_URL="https://dr-download.ti.com/software-development/software-development-kit-sdk/MD-bA0wfI4X2g/11.00.00.06/ti-processor-sdk-rtos-j721e-evm-11_00_00_06.tar.gz"
    SDK_ARCHIVE="${SDK_URL##*/}"
    DOWNLOADS_DIR="$HOME/Downloads"
    EXTRACT_DIR="$HOME/ti"
    PATCH_VAR="TOOLS_INSTALL_PATH ?="
    PATCH_VAL="\$(HOME)/ti"

    mkdir -p "$DOWNLOADS_DIR"
    cd "$DOWNLOADS_DIR"
    if [ ! -f "$SDK_ARCHIVE" ]; then
        echo "Downloading SDK: $SDK_URL ..."
        if command -v wget >/dev/null 2>&1; then
            wget "$SDK_URL"
        elif command -v curl >/dev/null 2>&1; then
            curl -LO "$SDK_URL"
        else
            echo "Neither wget nor curl found!"
            gracefull_exit -1
        fi
    else
        echo "SDK archive already downloaded."
    fi

    mkdir -p "$EXTRACT_DIR"
    cd "$EXTRACT_DIR"
    if [[ "$SDK_ARCHIVE" == *.tar.xz ]]; then
        tar -xJf "$DOWNLOADS_DIR/$SDK_ARCHIVE"
    elif [[ "$SDK_ARCHIVE" == *.tar.gz ]]; then
        tar -xzf "$DOWNLOADS_DIR/$SDK_ARCHIVE"
    elif [[ "$SDK_ARCHIVE" == *.zip ]]; then
        unzip "$DOWNLOADS_DIR/$SDK_ARCHIVE"
    else
        echo "Unknown SDK archive format."
        gracefull_exit -1
    fi

    SDK_TOP=$(find "$EXTRACT_DIR" -maxdepth 1 -type d -name "ti-processor-sdk-rtos-*" | head -n1)
    if [ -z "$SDK_TOP" ]; then
        echo "Extracted SDK directory not found!"
        gracefull_exit -1
    fi
    echo "SDK extracted to: $SDK_TOP"

    cd "$SDK_TOP"
    if [ -f "./sdk_builder/scripts/setup_psdk_rtos.sh" ]; then
        echo "Running toolchain setup script..."
        bash ./sdk_builder/scripts/setup_psdk_rtos.sh --qnx_sbl --firmware_only --skip_pc_emulation --skip_atf_optee --skip_linux
    else
        echo "Toolchain setup script not found!"
        gracefull_exit -1
    fi

    PDK_BUILD_RULES="$SDK_TOP/pdk_jacinto_11_00_00_21/packages/ti/build/Rules.make"
    if [ -f "$PDK_BUILD_RULES" ]; then
        sed -i "s|^$PATCH_VAR.*|$PATCH_VAR $PATCH_VAL|" "$PDK_BUILD_RULES"
        echo "Patched $PDK_BUILD_RULES"
    else
        echo "Could not find $PDK_BUILD_RULES to patch!"
    fi

    echo "SDK ready in $SDK_TOP"
}



gracefull_exit() {
    echo "Logs generated succesfully, exiting with code $1."
    logger 0
    sleep 1
    exit $1
}
logger() {
    if [ $1 -eq 1 ]; then
        mkdir -p logs
        if [ $? -ne 0 ]; then
            echo "Failed to create logs. Exiting with code -4."
            exit -4
        fi
        LOG_FILE=logs/$( date '+%Y-%m-%d_%H-%M-%S' ).log

        # Redirect stdout and stderr, adding a timestamp to each log line
        exec > >(while IFS= read -r line; do echo "$(date '+%Y-%m-%d %H:%M:%S') $line"; done | tee $LOG_FILE) 2>&1
        if [ $? -ne 0 ]; then
            echo "Failed to redirect file descriptors. Exiting with code -4."
            exit -4
        fi
        echo -e "Logger set up & started"
    elif [ $1 -eq 0 ]; then
        exec &>/dev/tty
        if [ $? -ne 0 ]; then
            echo "Failed to restore file descriptors to their original state. Exiting with code -4."
            exit -4
        fi
    fi
}
submodule_check() {
    echo "Checking dependecies..."
    if [ ! -d ".git" ]; then
        echo "Error: This script must be run inside a Git repository."
        restore_filedescriptors
        gracefull_exit -2
    fi
    git submodule update --init --recursive

    echo -e "All submodules are up to date!\n"
}

buildFactory() {
    echo "Building started.."
    sudo bash -c 'echo 0 > /proc/sys/kernel/apparmor_restrict_unprivileged_userns'
    export TEMPLATECONF="${PWD}/bbai64_os/conf/templates/custom"
    source deps/poky/oe-init-build-env .build
    if [ $? -ne 0 ]; then
        echo "Failed to build the project."
        gracefull_exit -3
    fi

    bitbake factory-image
    if [ $? -ne 0 ]; then
        echo "Failed to build the project."
        gracefull_exit -3
    fi
}

buildFreeRTOSdep() {
    echo "Rebuilding FreeRTOS with custom config..."

    local BTYPE="${BTYPE:-release}"
    local CORE="${CORE:-mcu2_0}"
    local CORE_ALIAS="${CORE_ALIAS:-r5f}"
    local SOC="${SOC:-j721e}"
    local KEEP_PATCHED_conf="${KEEP_PATCHED_conf:-0}"   # set to 1 to keep header patched

    local PSDK_RTOS_PATH
    PSDK_RTOS_PATH=$(find "$HOME/ti" -maxdepth 1 -type d -name "ti-processor-sdk-rtos-*" | head -n1)
    if [ -z "$PSDK_RTOS_PATH" ]; then
        echo "ERROR: PSDK RTOS not found under \$HOME/ti"
        gracefull_exit -5
    fi
    local PDK_PATH="$PSDK_RTOS_PATH/pdk_jacinto_11_00_00_21/packages"

    local conf
    conf=$(find "$PDK_PATH" -path "*/kernel/freertos/config/j721e/r5f/FreeRTOSConfig.h" | head -n1)
    if [ -z "$conf" ]; then
        # fallback search if layout differs
        conf=$(grep -RIl "TI_FREERTOS_CONFIG_H" "$PDK_PATH" 2>/dev/null | head -n1)
    fi
    if [ -z "$conf" ] || [ ! -f "$conf" ]; then
        echo "ERROR: Could not locate FreeRTOSConfig.h aggregator in PDK."
        gracefull_exit -5
    fi

    echo "Patching..: $conf"
    local BAK="${conf}.bak_queuesets_$(date +%s)"
    cp "$conf" "$BAK" || { echo "ERROR: backup failed"; gracefull_exit -5; }

    # Flip configUSE_QUEUE_SETS = 1
    sed -Ei \
        -e 's@(^[[:space:]]*#\s*define\s+configUSE_QUEUE_SETS)[[:space:]]*\(([01])\)@\1 (1)@' \
        -e 's@(^[[:space:]]*#\s*define\s+configUSE_QUEUE_SETS)[[:space:]]+([01])@\1 1@' \
        "$conf"

    # Ensure INCLUDE_xTaskDelayUntil is enabled (add if missing)
    if grep -Eq '^[[:space:]]*#\s*define\s+INCLUDE_xTaskDelayUntil\b' "$conf"; then
        sed -Ei \
            -e 's@(^[[:space:]]*#\s*define\s+INCLUDE_xTaskDelayUntil)[[:space:]]*\(([01])\)@\1 (1)@' \
            -e 's@(^[[:space:]]*#\s*define\s+INCLUDE_xTaskDelayUntil)[[:space:]]+([01])@\1 1@' \
            "$conf"
    else
        printf "\n#define INCLUDE_xTaskDelayUntil (1)\n" >> "$conf"
    fi
    
    # Clean & rebuild FreeRTOS prebuilt lib for the target
    pushd "$PDK_PATH/ti/build" >/dev/null || { echo "ERROR: enter build dir"; mv -f "$BAK" "$conf"; gracefull_exit -5; }

    # CAREFUL FOR PATHING
    kernel_dir="$PDK_PATH/ti/binary/kernel"
    symbols_req="vTaskStartScheduler xTaskCreate vTaskSwitchContext vPortEnterCritical vPortExitCritical xQueueGenericSend xQueueReceive"
    
    echo "Attempt clear of the freertos comp…" 
    ok=1
    if [ -d "$kernel_dir" ]; then
        # we collect R5F objs/libs in the kernel
        readarray -d '' CAND < <(find "$kernel_dir/obj/$SOC/$CORE/$BTYPE" -type f \
        \( -name '*.oer5f' -o -name '*.aer5f' \) -print0 2>/dev/null)

        if ((${#CAND[@]} == 0)); then
            echo "No *.oer5f/*.aer5f found in: $kernel_dir"
            ok=0
        else
            echo "Found ${#CAND[@]} files under $kernel_dir"

            # we build a unique set of defined symbol names from those files
            readSymbols=$(nm -A --defined-only "${CAND[@]}" 2>/dev/null | awk '{print $NF}' | sort -u)

            # check for the required symbols
            missing=()
            for s in $SYMS; do
                grep -Fxq -- "$s" <<<"$readSymbols" || missing+=("$s")
            done

            if ((${#missing[@]} == 0)); then
                echo "All required symbols present."
            else
                echo "We found missing symbols:" "${missing[@]}"
                ok=0
            fi
        fi
    else
        echo "No dir present!";
    fi

    if [ "$ok" -eq 0 ]; then
        echo "No clear needed!"
    else
        echo "Clear started!"
        sudo rm -rf "$kernel_dir"
    fi
    
    echo "Building freertos…"
    gmake -s freertos CORE="$CORE" OS=linux || {
        echo "ERROR: freertos build failed"
        popd >/dev/null
        mv -f "$BAK" "$conf"
        gracefull_exit -5
    }
    popd >/dev/null

    # Restore header to keep SDK pristine
    if [ "$KEEP_PATCHED_conf" -ne 1 ]; then
        mv -f "$BAK" "$conf"
        echo "Config restored to original (library remains rebuilt with queue sets)."
    else
        echo "Config left patched. Backup at: $BAK"
    fi

    # Show where the library lives
    echo "Done. The rebuilt FreeRTOS lib is under:"
    find "$PDK_PATH/ti/kernel/freertos" -type f -name "freertos.*er5f" -o -name "freertos.*aer5f" 2>/dev/null | sed 's/^/  - /'
}

buildCustomSW() {
    echo "Building started.."
    sudo bash -c 'echo 0 > /proc/sys/kernel/apparmor_restrict_unprivileged_userns'
    export TEMPLATECONF="${PWD}/bbai64_os/conf/templates/custom"
    source deps/poky/oe-init-build-env .build
    if [ $? -ne 0 ]; then
        echo "Failed to build the project."
        gracefull_exit -3
    fi

    bitbake -ccleansstate linux-bb.org
    bitbake -ccleansstate firmwares
    bitbake -ccleansstate env-init
    bitbake linux-bb.org
    bitbake firmware-image

    if [ $? -ne 0 ]; then
        echo "Failed to build the project."
        gracefull_exit -3
    fi
}

buildFirmwares() {
    echo "Firmwares build started.."

    PSDK_RTOS_PATH=$(find "$HOME/ti" -maxdepth 1 -type d -name "ti-processor-sdk-rtos-*" | head -n1)
    PDK_PATH=$PSDK_RTOS_PATH/pdk_jacinto_11_00_00_21/packages
    REPO_ROOT=${PWD}
    APP_PATH=${PDK_PATH}/ti/drv/ipc/examples/DFC
    MK_FILE="${APP_PATH}/DFC_*.mk"
    BACKUP_SUFFIX=".bak"
    BUILD_ERR=0
    TARGET="${PDK_PATH}/ti/drv/ipc/ipc_component.mk"

    rm -rf $APP_PATH
    cp -r $REPO_ROOT/core $APP_PATH
    if [ $? -ne 0 ]; then
        echo "Failed to build firmwares."
        BUILD_ERR=-3
    fi

    if [ ! -f "${PDK_PATH}/ti/drv/ipc/ipc_component.mk${BACKUP_SUFFIX}" ]; then
        cp "${PDK_PATH}/ti/drv/ipc/ipc_component.mk" "${PDK_PATH}/ti/drv/ipc/ipc_component.mk${BACKUP_SUFFIX}"
    fi
    
    for f in "$APP_PATH"/DFC_*.mk; do
        grep -qF "$(head -n1 "$f")" "$TARGET" || cat "$f" >> "$TARGET"
    done

    cd $PDK_PATH/ti/build

    gmake -S CORE=mcu2_0 OS=linux pdk_libs
    if [ $? -ne 0 ]; then
        echo "Failed to build deps for the R5F firmware."
        BUILD_ERR=-3
    fi

    gmake -S CORE=c66xdsp_1 OS=linux pdk_libs
    if [ $? -ne 0 ]; then
        echo "Failed to build deps for the C66X firmware."
        BUILD_ERR=-3
    fi
    
    buildFreeRTOSdep

    echo ":::::::::NOTE: STARTED R5F MCU2_0 MAIN ISLAND CORE IDX 1 FIRMWARE BUILD..:::::::::"
    gmake -S CORE=mcu2_0 OS=linux dfc_app
    if [ $? -ne 0 ]; then
        echo "Failed to build firmwares."
        BUILD_ERR=-3
    fi

    echo ":::::::::NOTE: STARTED C66X CORE IDX 1 FIRMWARE BUILD..:::::::::"
    gmake -S CORE=c66xdsp_1 OS=linux dfc_c66_app
    if [ $? -ne 0 ]; then
        echo "Failed to build firmwares."
        BUILD_ERR=-3
    fi

    if [ ! -d "$REPO_ROOT/.build/firmwares" ]; then
        mkdir -p $REPO_ROOT/.build/firmwares
    fi

    cp -r "${PDK_PATH}/ti/binary/"dfc_* "${REPO_ROOT}/.build/firmwares"
    if [ $? -ne 0 ]; then
        echo "Failed to copy firmwares to the output dir please check the SDK/PDK binary dir."
        BUILD_ERR=-3
    fi

    mv "${PDK_PATH}/ti/drv/ipc/ipc_component.mk${BACKUP_SUFFIX}" "${PDK_PATH}/ti/drv/ipc/ipc_component.mk"
    if [ $? -ne 0 ]; then
        echo "Failed to restore the sdk buildables list to its original form."
        BUILD_ERR=-3
    fi

    rm -rf $APP_PATH
    if [ $? -ne 0 ]; then
        echo "Failed to restore sdk state to its original"
        BUILD_ERR=-3
    fi
    
    cd ${REPO_ROOT}

    if [ $BUILD_ERR -ne 0 ]; then
        gracefull_exit $BUILD_ERR
    fi
}


buildFactoryDocker() {
    echo "CHU..CHU...Build is on it's wayy.."
    sudo bash -c 'echo 0 > /proc/sys/kernel/apparmor_restrict_unprivileged_userns'
    docker build -t core_low_container .
    if [ $? -ne 0 ]; then
        echo "Oh noo! The train derailed...project build failed."
        gracefull_exit -3
    fi
    docker run -it --rm --privileged --security-opt apparmor=unconfined \
                -v "$(pwd)":/home/ThomasTheTankEngine/workspace \
                -w /home/ThomasTheTankEngine/workspace \
                --user 1000:1000 \
                core_low_container \
                bash -c 'export TEMPLATECONF=/home/ThomasTheTankEngine/workspace/core/conf && source deps/poky/oe-init-build-env .build && bitbake core-image-minimal
                '
    if [ $? -ne 0 ]; then
        echo "Oh noo! The train derailed...project build failed."
        gracefull_exit -3
    fi
}

menuWrapper() {
   if [ $# -ne 1 ]; then
        echo "Usage: $0 [option] ..."
        echo -e "OPTIONS:"
        echo -e "\t-build:\t\t Build factory image"
        echo -e "\t-buildi:\t Build custom sw image"
        echo -e "\t-buildf:\t Build custom firmwares"
        echo -e "\t-buildexp:\t Build express (CHUCHU) via docker container"
        echo -e "\t-sync:\t\t Sync the enviroment"
        echo -e "\t-btest:\t Build gtests for the sw"
        echo -e "\t-etest:\t Build and execute the gtests for the sw"
        echo -e "\t-liteclean:\t Clean build artifacts"
        echo -e "\t-fullclean:\t Clean build artifacts"
        echo -e "\t-rebuildfr:\t Rebuild FreeRTOS with custom config"
        
        exit -1
    fi

    if [ "$1" == "-build" ] || [ "$1" == "-b" ]; then
        logger 1
        buildFactory
        gracefull_exit 1
    elif [ "$1" == "-buildi" ] || [ "$1" == "-bi" ]; then
        logger 1
        buildFirmwares
        buildCustomSW
        gracefull_exit 1
    elif [ "$1" == "-buildf" ] || [ "$1" == "-bf" ]; then
        logger 1
        buildFirmwares
        gracefull_exit 1
    elif [ "$1" == "-buildexp" ] || [ "$1" == "-be" ]; then
        logger 1
        buildFactoryDocker
        gracefull_exit 1
    elif [ "$1" == "-sync" ] ||  [ "$1" == "-s" ]; then
        logger 1
        sync
        gracefull_exit 1
    elif [ "$1" == "-fullclean" ] || [ "$1" == "-fc" ]; then
        logger 1
        clean_artifacts_full
        gracefull_exit 1
    elif [ "$1" == "-liteclean" ] || [ "$1" == "-lc" ]; then
        logger 1
        clean_artifacts_lite
        gracefull_exit 1
    elif [ "$1" == "-rebuildfr" ] || [ "$1" == "-rfr" ]; then
        logger 1
        buildFreeRTOSdep
        gracefull_exit 1
    else 
        echo "Error: Invalid argument"
        echo "Usage: $0 [option] ..."
        echo -e "OPTIONS:"
        echo -e "\t-build:\t\t Build factory image"
        echo -e "\t-buildi:\t Build custom sw image"
        echo -e "\t-buildf:\t Build custom firmwares"
        echo -e "\t-buildexp:\t Build express (CHUCHU) via docker container"
        echo -e "\t-sync:\t\t Sync the enviroment"
        echo -e "\t-liteclean:\t Clean build artifacts"
        echo -e "\t-fullclean:\t Clean build artifacts"
        echo -e "\t-rebuildfr:\t Rebuild FreeRTOS with custom config"

        exit -1
    fi
}

main() {
    menuWrapper $*
}

main $*