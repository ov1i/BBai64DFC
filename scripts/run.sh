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

# flash_sw() {
#     echo "FLASH proccess started.."
#     st-flash --reset write ${PWD}/.build/DFC.bin 0x08000000
#     if [ $? -ne 0 ]; then
#         echo "FLASH process failed! Cleanup started"
#         st-flash --reset erase
#         if [ $? -ne 0 ]; then
#             echo "Cleanup failed please check the SW before reflashing!"
#         fi
#         gracefull_exit -7
#     fi

#     echo "Veryfing flash.."
#     size=$(stat --format=%s ${PWD}/.build/DFC.bin)
#     st-flash read ${PWD}/.build/verify.bin 0x08000000 $size
#     if [ $? -ne 0 ]; then
#         echo "Failed reading the flash from the board! Please check connectivity of the board!"
#         gracefull_exit -7
#     fi

#     if ! cmp -s ${PWD}/.build/verify.bin ${PWD}/.build/DFC.bin; then
#         echo "Found discrepancy between the local SW and the flash result. SW corrupted!"
#         echo "Cleanup started.."
#         st-flash --reset erase
#         if [ $? -ne 0 ]; then
#             echo "Cleanup failed!"
#         fi
#         gracefull_exit -7
#     fi
#     if [ $? -ne 0 ]; then
#         echo "Verifying failed..Cleanup started! (safety mechanism)"
#         st-flash --reset erase
#         if [ $? -ne 0 ]; then
#             echo "Cleanup failed!"
#         fi
#         gracefull_exit -7
#     fi
#     echo "SW flashed succesfully!"
# }

# check_board_connectivity() {
#     echo "Checking board connectivity.."

#     status="$(st-info --probe)"

#     if echo "$status" | grep -q "chipid:     0x451" && echo "$status" | grep -q "dev-type:   STM32F76x_F77x"; then
#         echo "STM32 board detected!"
#     else
#         echo "STM32 board not found!"
#         gracefull_exit -6
#     fi

# }
# check_sw_existance() {
#     echo "Checking build existance.."
#     if [ -d ".build" ]; then
#         echo "Build found continuing.."
#         if [ -f "${PWD}/.build/DFC.bin" ]; then
#             echo "SW found, continuing.."
#         else
#             echo "SW not found, please check the existance of the target or rebuild the project!"
#             gracefull_exit -5
#         fi
#     else
#         echo "Build not found, board flash failed!"
#         gracefull_exit -5
#     fi
# }   

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

# compileProject() {
#     echo "Compilation started.."
#     cmake . -B.build -GNinja -DCMAKE_VERBOSE_MAKEFILE=ON -DCMAKE_BUILD_TYPE=debug -DCMAKE_TOOLCHAIN_FILE=cmake/tc_utils.cmake -DDUMP_ASM=OFF
#     if [ $? -ne 0 ]; then
#         echo "Failed to compile the source code."
#         gracefull_exit -3
#     fi
#     echo "Compilation succesful.."
# }

buildProject() {
    echo "Building started.."
    sudo bash -c 'echo 0 > /proc/sys/kernel/apparmor_restrict_unprivileged_userns'
    # export TEMPLATECONF="${PWD}/core/conf"
    export TEMPLATECONF="${PWD}/bbai64_os/conf/templates/custom"
    source deps/poky/oe-init-build-env .build
    if [ $? -ne 0 ]; then
        echo "Failed to build the project."
        gracefull_exit -3
    fi

    # bitbake -c menuconfig u-boot-ti-staging
    bitbake -ccleansstate env-init
    bitbake dfc-factory-image
    if [ $? -ne 0 ]; then
        echo "Failed to build the project."
        gracefull_exit -3
    fi
}

buildProjectDocker() {
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
        echo -e "\t-build:\t Build and flash the board with the sw"
        echo -e "\t-buildexp:\t Build express (CHUCHU) via docker container"
        echo -e "\t-flash:\t Flash the sw on the board"
        echo -e "\t-btest:\t Build gtests for the sw"
        echo -e "\t-etest:\t Build and execute the gtests for the sw"
        echo -e "\t-liteclean:\t Clean build artifacts"
        echo -e "\t-fullclean:\t Clean build artifacts"
        
        exit -1
    fi

    if [ "$1" == "-build" ] || [ "$1" == "-b" ]; then
        logger 1
        # submodule_check
        # compileProject
        buildProject
        gracefull_exit 1
    elif [ "$1" == "-buildexp" ] || [ "$1" == "-be" ]; then
        logger 1
        # submodule_check
        # compileProject
        buildProjectDocker
        gracefull_exit 1
    elif [ "$1" == "-flash" ] ||  [ "$1" == "-f" ]; then
        logger 1
        # check_st_tools
        # check_board_connectivity
        # check_sw_existance
        # flash_sw
        gracefull_exit 1
    elif [ "$1" == "-bflsh" ] ||  [ "$1" == "-bf" ]; then
        logger 1
        # submodule_check
        # compileProject
        # buildProject
        # check_st_tools
        # check_board_connectivity
        # check_sw_existance
        # flash_sw
        gracefull_exit 1
    elif [ "$1" == "-fullclean" ] || [ "$1" == "-fc" ]; then
        logger 1
        clean_artifacts_full
        gracefull_exit 1
    elif [ "$1" == "-liteclean" ] || [ "$1" == "-lc" ]; then
        logger 1
        clean_artifacts_lite
        gracefull_exit 1
    else 
        echo "Error: Invalid argument"
        echo "Usage: $0 [option] ..."
        echo -e "OPTIONS:"
        echo -e "\t-build:\t Build and flash the board with the sw"
        echo -e "\t-buildexp:\t Build express (CHUCHU) via docker container"
        echo -e "\t-flash:\t Flash the sw on the board"
        echo -e "\t-btest:\t Build gtests for the sw"
        echo -e "\t-etest:\t Build and execute the gtests for the sw"
        echo -e "\t-liteclean:\t Clean build artifacts"
        echo -e "\t-fullclean:\t Clean build artifacts"

        exit -1
    fi
}

main() {
    menuWrapper $*
}

main $*