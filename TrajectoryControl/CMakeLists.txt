add_executable(TrajectoryControl "")

include_directories("${INCLUDE_DIR}")

IF (WIN32)
    add_definitions(-D_WIN32_WINNT=0x0501)
ENDIF()

add_definitions(-DNOMINMAX)
add_definitions(-DUNICODE)
add_definitions(-D_UNICODE)

include_directories("${INCLUDE_DIR}")
include_directories("${CMAKE_CURRENT_LIST_DIR}/../util")

IF (MSVC)
    string(REPLACE "/MDd" "/MD" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
ENDIF()

file(GLOB_RECURSE srcs *.h *.hh *i *.hpp *.c *.cc *.cpp)
target_sources(TrajectoryControl PRIVATE ${srcs})
source_group(TREE ${CMAKE_CURRENT_LIST_DIR} FILES ${srcs})

SetDefaultTargetProperties(TrajectoryControl)

IF (WIN32)
    target_link_libraries(TrajectoryControl  ${LIB_DIR}/SimOneAPI.lib ${LIB_DIR}/SSD.lib ${LIB_DIR}/HDMapModule.lib  ${LIB_DIR}/ProtobufModule.lib)

    add_custom_command(TARGET TrajectoryControl
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/SimOneAPI.dll ${BINARY_OUT_PATH}/SimOneAPI.dll
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/SSD.dll ${BINARY_OUT_PATH}/SSD.dll
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/HDMapModule.dll ${BINARY_OUT_PATH}/HDMapModule.dll
        )
ELSEIF (UNIX)
        target_link_libraries(TrajectoryControl debug ${LIB_DIR}/libSimOneAPI.so)
        target_link_libraries(TrajectoryControl optimized ${LIB_DIR}/libSimOneAPI.so -lpthread)
        target_link_libraries(TrajectoryControl debug ${LIB_DIR}/libSSD.so)
        target_link_libraries(TrajectoryControl optimized ${LIB_DIR}/libSSD.so -lpthread)
        target_link_libraries(TrajectoryControl debug ${LIB_DIR}/libHDMapModule.so)
        target_link_libraries(TrajectoryControl optimized ${LIB_DIR}/libHDMapModule.so)

        add_custom_command(TARGET TrajectoryControl
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libSimOneAPI.so ${BINARY_OUT_PATH}/libSimOneAPI.so
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libSSD.so ${BINARY_OUT_PATH}/libSSD.so
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libHDMapModule.so ${BINARY_OUT_PATH}/libHDMapModule.so
        )
ENDIF()
