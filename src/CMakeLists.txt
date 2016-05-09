# This file is in /src

# SET(CMAKE_C_COMPILER g++)

SET(CMAKE_CXX_COMPILER arm-linux-gnueabi-g++)
SET(CMAKE_C_COMPILER arm-linux-gnueabi-g++)

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -Wno-unused-variable -pthread")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -Wno-unused-variable -pthread")
# SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++0x -g -I/home/mynfs/usr/xenomai/include -L/home/mynfs/usr/xenomai/lib")

link_directories(/home/mynfs/usr/xenomai/lib)
SET(LIBRARIES libnative.so libnative.a libxenomai.so libxenomai.a libpthread_rt.so libpthread_rt.a)

AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR} MAIN_SRC)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/data_absyntax DATA_ABSYNTAX_SRC)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/data_stage1_2 DATA_STAGE1_2_SRC)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/data_stage4 DATA_STAGE4_SRC)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/data_stage4/delete_tree DATA_STAGE4_DELTREE_SRC)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/data_stage4/generate_symtable DATA_STAGE4_GENSYM_SRC)
#AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/data_util DATA_UTIL_SRC)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/program_absyntax PROG_ABSYNTAX_SRC)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/program_stage1_2 PROG_STAGE1_2_SRC)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/program_stage4 PROG_STAGE4_SRC)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/program_stage4/delete_tree PROG_STAGE4_DELTREE_SRC)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/program_stage4/generate_stmt_node PROG_STAGE4_GENSTM_SRC)
#AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/program_util PROG_UTIL_SRC)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/stage5 STAGE5_SRC)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/stage5/statement_interpreter STAGE5_INTER_SRC)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/stage6_interpolator STAGE6_SRC)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/stage6_interpolator/stage6_test STAGE6_TEST_SRC)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/stage5_to_6 STAGE5_to_6_SRC)
ADD_SUBDIRECTORY(data_stage1_2)
ADD_SUBDIRECTORY(program_stage1_2)

ADD_EXECUTABLE(testinterpolator.exe ${STAGE6_TEST_SRC} ${STAGE6_SRC})
ADD_EXECUTABLE(rc-runtime.exe   ${MAIN_SRC}
                                ${DATA_ABSYNTAX_SRC}
                                ${DATA_STAGE1_2_SRC}
                                ${DATA_STAGE4_SRC}
                                ${DATA_STAGE4_DELTREE_SRC}
                                ${DATA_STAGE4_GENSYM_SRC}
                                ${PROG_ABSYNTAX_SRC}
                                ${PROG_STAGE1_2_SRC}
                                ${PROG_STAGE4_SRC}
                                ${PROG_STAGE4_DELTREE_SRC}
                                ${PROG_STAGE4_GENSTM_SRC}
                                ${STAGE5_SRC}
                                ${STAGE5_INTER_SRC}
                                ${STAGE6_SRC}
                                ${STAGE5_to_6_SRC} )
target_link_libraries(rc-runtime.exe ${LIBRARIES})

