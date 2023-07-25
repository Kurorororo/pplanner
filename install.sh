#!/bin/bash

env=pplanner

(
    set -e

    trap "conda env remove -n $env" ERR

    (conda activate >/dev/null 2>/dev/null)  || {
        echo "This script must be sourced, not executed. Run it like: source $0"
        exit 1
    }

    conda env create -n $env -f environment.yml || {
        echo "installation failed; removing partially built environment $env"
        conda env remove -n $env
        exit 1
    }

    conda activate $env

    git submodule update --init --recursive

    ################################################################
    # reset my local variables so that they do not affect reproducibility

    echo "path variables that are set locally:"
    echo "-----------------"
    env | grep PATH | cut -d= -f1
    echo "-----------------"
    echo

    conda env config vars set LD_LIBRARY_PATH=
    conda env config vars set PYTHONPATH=
    conda env config vars set PKG_CONFIG_PATH=

    # these are required for building dependencies properly under conda
    conda env config vars set CPATH=${CONDA_PREFIX}/include:${CONDA_PREFIX}/targets/x86_64-linux/include:${CONDA_PREFIX}/x86_64-conda-linux-gnu/sysroot/usr/include:

    # note: "variables" field in yaml file introduced in conda 4.9 does not work because it does not expand shell variables

    # these are required for the variables to take effect
    conda deactivate ; conda activate $env

    echo $CPATH ; echo $LD_LIBRARY_PATH ; echo $PYTHONHASHSEED

    conda env config vars list

    ################################################################
    # build

    make -C lib/bliss/

    mkdir -p build
    (
        cd build
        cmake ..
        make
    )



) && {
    conda activate $env >/dev/null 2>/dev/null || echo "This script must be sourced, not executed. Run it like: source $0"
}
