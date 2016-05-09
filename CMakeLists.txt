# This file is in root dir to project
# NOTE: DO NOT use ${PROJECT_BINARY_DIR}
#       DO NOT compile symtable.cc alone!
PROJECT(RC_RUNTIME)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/data_absyntax
        ${PROJECT_SOURCE_DIR}/src/data_stage1_2
        ${PROJECT_SOURCE_DIR}/src/data_stage4
        ${PROJECT_SOURCE_DIR}/src/data_stage4/delete_tree
        ${PROJECT_SOURCE_DIR}/src/data_stage4/generate_symtable
        ${PROJECT_SOURCE_DIR}/src/data_util
        ${PROJECT_SOURCE_DIR}/src/program_absynatax
        ${PROJECT_SOURCE_DIR}/src/program_stage1_2
        ${PROJECT_SOURCE_DIR}/src/program_stage4
        ${PROJECT_SOURCE_DIR}/src/program_stage4/delete_tree
        ${PROJECT_SOURCE_DIR}/src/program_stage4/generate_stmt_node
        ${PROJECT_SOURCE_DIR}/src/program_util
        ${PROJECT_SOURCE_DIR}/src/stage5
        ${PROJECT_SOURCE_DIR}/src/statement_interpreter
        ${PROJECT_SOURCE_DIR}/src/stage6_interpolator
        ${PROJECT_SOURCE_DIR}/src/stage5_to_6
        /home/mynfs/usr/xenomai/include)
ADD_SUBDIRECTORY(src)
