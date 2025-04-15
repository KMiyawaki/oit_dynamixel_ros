#!/bin/bash

function copy_files(){
    local -r DST=${1}
    local -r SRCS=${@:2:($#-1)}
    for s in ${SRCS}
    do
        local cmd="sudo cp $s ${DST}/`basename $s`"
        echo $cmd
        eval $cmd
    done
}

function main(){
    cd $(dirname "$0")
    local -r SCRIPT_DIR=$(pwd)
    local -r PREFIX="DX2LIB"
    local -r TARGET="${PREFIX}_V3.0"
    rm -f ${TARGET}.zip
    local TMP=$(ls -d ${PREFIX}*/)
    if [ -n ${TMP}  ];then
        echo "Remove ${TMP}"
        rm -fr ${TMP}
    fi
    wget https://www.besttechnology.co.jp/download/${TARGET}.zip -O ${TARGET}.zip
    unzip -q ${TARGET}.zip
    local -r DX_ROOT="./$(ls -d ${PREFIX}*/)"
    local -r LIBS_DIR="${DX_ROOT}DX2LIB"
    
    cd ${LIBS_DIR}
    bash ./build_dx2lib.sh
    cd "${SCRIPT_DIR}"
    local -r HEADERS=$(find ${LIBS_DIR} -name '*.h')
    local -r LIBS="$(find ${LIBS_DIR} -name '*lib.a') $(find ${LIBS_DIR} -name '*.so.*')"
    local -r PYS=$(find ${LIBS_DIR} -name '*.py')
    sed -i "s/.\/dx2lib./\/usr\/local\/lib\/dx2lib./g" ${PYS}

    local -r HEADERS_DST="/usr/local/include"
    local -r LIBS_DST="/usr/local/lib"
    local -r PYS_DST=`python -c "import site; print(*site.getsitepackages(), sep='\n')"|grep /usr/local`

    local -r INSTALL_LOG="${SCRIPT_DIR}/install_dynamixel_rotocol_2_library_log.txt"
    local -r UNINSTALLER="${SCRIPT_DIR}/uninstall_dynamixel_rotocol_2_library.sh"

    touch ${INSTALL_LOG}
    echo "# `date`" > ${INSTALL_LOG}
    copy_files ${HEADERS_DST} ${HEADERS} >> ${INSTALL_LOG}
    copy_files ${LIBS_DST} ${LIBS} >> ${INSTALL_LOG}
    copy_files ${PYS_DST} ${PYS} >> ${INSTALL_LOG}

    echo "#!/bin/bash -x" > ${UNINSTALLER}
    awk '{print "sudo rm -f " $4}' ${INSTALL_LOG}|grep '/usr' >> ${UNINSTALLER}
    chmod u+x ${UNINSTALLER}

    cd "${SCRIPT_DIR}"
    rm -fr ./${TARGET}.zip
    rm -fr ${DX_ROOT}
    cat ${INSTALL_LOG}
}

main "$@"
