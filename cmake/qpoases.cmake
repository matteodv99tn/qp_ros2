
set (QPOASES_INSTALL_DIR ${CMAKE_BINARY_DIR}/qpoases_install)
include(ExternalProject)
ExternalProject_Add(
    external_qpoases
    URL https://github.com/coin-or/qpOASES/archive/refs/tags/releases/3.2.1.tar.gz
    CMAKE_ARGS 
        -DCMAKE_INSTALL_PREFIX=${QPOASES_INSTALL_DIR}
        -DCMAKE_BUILD_TYPE=Release
        -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
        -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
        -DBUILD_SHARED_LIBS=OFF
        -DQPOASES_BUILD_EXAMPLES=OFF
)
add_library(qpoases_helper INTERFACE)
add_dependencies(qpoases_helper external_qpoases)
target_include_directories(qpoases_helper INTERFACE ${QPOASES_INSTALL_DIR}/include)
target_link_libraries(qpoases_helper INTERFACE ${QPOASES_INSTALL_DIR}/lib/libqpOASES.a)
add_library(qpoases::qpoases ALIAS qpoases_helper)
