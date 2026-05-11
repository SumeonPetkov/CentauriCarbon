#!/usr/bin/env bash

pwd=`pwd`

# Function to display help information
show_help() {
    echo "Usage: $0 [options] [project_name]"
    echo
    echo "Options:"
    echo "  -h, --help        Display this help message and exit"
    echo "  -p, --project     name and directory of the project to be built, defualt projcet is e100"
    echo "  -n, --nfs         directory of your nfs server"
    echo
    echo "Examples:"
    echo "  $0"
    echo "  $0 -p e100"
}


# the default projcet 
proj_dir=${proj_dir:-e100}
nfs_dir=""


# Parse options
TEMP=$(getopt -o hp:n: --long help,project:,nfs: -n 'multi_options.sh' -- "$@")
if [ $? != 0 ]; then
    echo "Terminating..." >&2
    exit 1
fi

eval set -- "$TEMP"

while true; do
    case "$1" in
        -h|--help)
            show_help
            exit 0
            ;;
        -n|--nfs)
            nfs_dir="$2"
            shift 2
            ;;
        -p|--project)
            proj_dir="$2"
            shift 2
            ;;
        --)
            shift
            break
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done


echo "start to build ${proj_dir}"
# make & build the app
if [ $proj_dir = "e100" ]; then
    cp CMakeLists.txt.e100 CMakeLists.txt
    sed -i 's/#define CONFIG_BOARD_E100 .*/#define CONFIG_BOARD_E100 BOARD_E100/' jenkins.h
elif [ $proj_dir = "e100_lite" ]; then
    cp CMakeLists.txt.e100_lite CMakeLists.txt
    sed -i 's/#define CONFIG_BOARD_E100 .*/#define CONFIG_BOARD_E100 BOARD_E100_LITE/' jenkins.h
elif [ $proj_dir = "" ]; then
    cp CMakeLists.txt.e100 CMakeLists.txt
else
    echo "Unsupported project: ${proj_dir} "
    exit 1
fi

${pwd}/tools/tr/tr -g ${pwd}/resources/${proj_dir}/translation.csv ${pwd}/resources/${proj_dir}/translation.bin
${pwd}/tools/widget/widget -g ${pwd}/resources/${proj_dir}/widget.csv ${pwd}/resources/${proj_dir}/widget.bin

mkdir ${pwd}/build
cd ${pwd}/build
cmake ${pwd}
make -j19
