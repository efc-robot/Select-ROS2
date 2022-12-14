find_package(ament_cmake_gtest REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

find_package(mimick_vendor REQUIRED)
find_package(test_msgs REQUIRED)

find_package(mimick_vendor REQUIRED)

find_package(rcpputils REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)

find_package(osrf_testing_tools_cpp REQUIRED)

get_target_property(memory_tools_ld_preload_env_var
  osrf_testing_tools_cpp::memory_tools LIBRARY_PRELOAD_ENVIRONMENT_VARIABLE)

include(cmake/rcl_add_custom_executable.cmake)
include(cmake/rcl_add_custom_gtest.cmake)
include(cmake/rcl_add_custom_launch_test.cmake)

set(extra_lib_dirs "${rcl_lib_dir}")
set(test_resources_dir_name "resources")
add_definitions(-DTEST_RESOURCES_DIRECTORY="${CMAKE_CURRENT_BINARY_DIR}/${test_resources_dir_name}")

# finding gtest once in the highest scope
# prevents finding it repeatedly in each local scope
ament_find_gtest()

macro(test_target)
  find_package(${rmw_implementation} REQUIRED)
  test_target_function()
endmacro()

function(test_target_function)
  message(STATUS "Creating tests for '${rmw_implementation}'")
  set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

  # Gtests

  rcl_add_custom_gtest(test_client${target_suffix}
    SRCS rcl/test_client.cpp
    ENV ${rmw_implementation_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    LIBRARIES ${PROJECT_NAME}
    AMENT_DEPENDENCIES ${rmw_implementation} "osrf_testing_tools_cpp" "test_msgs"
  )

  rcl_add_custom_gtest(test_time${target_suffix}
    SRCS rcl/test_time.cpp
    ENV ${rmw_implementation_env_var} ${memory_tools_ld_preload_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    LIBRARIES ${PROJECT_NAME} osrf_testing_tools_cpp::memory_tools
    AMENT_DEPENDENCIES ${rmw_implementation}
  )

  # TODO(hidmic): re-enable timer tests against RTI Connext once
  #               https://github.com/ros2/rcl/issues/687 is resolved
  set(AMENT_GTEST_ARGS "")
  if(rmw_implementation STREQUAL "rmw_connext_cpp")
    message(STATUS "Skipping test_timer${target_suffix} test.")
    set(AMENT_GTEST_ARGS "SKIP_TEST")
  endif()

  rcl_add_custom_gtest(test_timer${target_suffix}
    SRCS rcl/test_timer.cpp
    ENV ${rmw_implementation_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    LIBRARIES ${PROJECT_NAME} osrf_testing_tools_cpp::memory_tools
    AMENT_DEPENDENCIES ${rmw_implementation}
    ${AMENT_GTEST_ARGS}
  )

  rcl_add_custom_gtest(test_context${target_suffix}
    SRCS rcl/test_context.cpp
    ENV ${rmw_implementation_env_var} ${memory_tools_ld_preload_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    LIBRARIES ${PROJECT_NAME} mimick osrf_testing_tools_cpp::memory_tools
    AMENT_DEPENDENCIES ${rmw_implementation}
  )

  rcl_add_custom_gtest(test_get_node_names${target_suffix}
    SRCS rcl/test_get_node_names.cpp
    ENV ${rmw_implementation_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    LIBRARIES ${PROJECT_NAME}
    AMENT_DEPENDENCIES ${rmw_implementation} "osrf_testing_tools_cpp"
  )

  rcl_add_custom_gtest(test_lexer${target_suffix}
    SRCS rcl/test_lexer.cpp
    ENV ${rmw_implementation_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    LIBRARIES ${PROJECT_NAME}
    AMENT_DEPENDENCIES ${rmw_implementation}
  )

  rcl_add_custom_gtest(test_lexer_lookahead${target_suffix}
    SRCS rcl/test_lexer_lookahead.cpp
    ENV ${rmw_implementation_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    LIBRARIES ${PROJECT_NAME}
    AMENT_DEPENDENCIES ${rmw_implementation} "osrf_testing_tools_cpp"
  )

  set(AMENT_GTEST_ARGS "")
  # TODO(wjwwood): remove this when the graph API works properly for connext dynamic
  if(rmw_implementation STREQUAL "rmw_connext_dynamic_cpp")
    message(STATUS "Skipping test_graph${target_suffix} test.")
    set(AMENT_GTEST_ARGS "SKIP_TEST")
  # TODO(mm318): why rmw_connext tests run much slower than rmw_fastrtps and rmw_opensplice tests
  elseif(rmw_implementation STREQUAL "rmw_connext_cpp")
    message(STATUS "Increasing test_graph${target_suffix} test timeout.")
    set(AMENT_GTEST_ARGS TIMEOUT 180)
  endif()
  rcl_add_custom_gtest(test_graph${target_suffix}
    SRCS rcl/test_graph.cpp
    ENV ${rmw_implementation_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    LIBRARIES ${PROJECT_NAME}
    AMENT_DEPENDENCIES ${rmw_implementation} "osrf_testing_tools_cpp" "test_msgs"
    TIMEOUT 120
    ${AMENT_GTEST_ARGS}
  )

  set(AMENT_GTEST_ARGS "")
  # TODO(mm318): why rmw_connext tests run much slower than rmw_fastrtps and rmw_opensplice tests
  if(rmw_implementation STREQUAL "rmw_connext_cpp")
    message(STATUS "Increasing test_info_by_topic${target_suffix} test timeout.")
    set(AMENT_GTEST_ARGS TIMEOUT 120)
  endif()
  rcl_add_custom_gtest(test_info_by_topic${target_suffix}
    SRCS rcl/test_info_by_topic.cpp rcl/wait_for_entity_helpers.cpp
    ENV ${rmw_implementation_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    LIBRARIES ${PROJECT_NAME}
    AMENT_DEPENDENCIES ${rmw_implementation} "osrf_testing_tools_cpp" "test_msgs"
    ${AMENT_GTEST_ARGS}
  )

  rcl_add_custom_gtest(test_count_matched${target_suffix}
    SRCS rcl/test_count_matched.cpp
    ENV ${rmw_implementation_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    LIBRARIES ${PROJECT_NAME}
    AMENT_DEPENDENCIES ${rmw_implementation} "osrf_testing_tools_cpp" "test_msgs"
  )

  rcl_add_custom_gtest(test_get_actual_qos${target_suffix}
    SRCS rcl/test_get_actual_qos.cpp
    ENV ${rmw_implementation_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    LIBRARIES ${PROJECT_NAME}
    AMENT_DEPENDENCIES ${rmw_implementation} "test_msgs" "osrf_testing_tools_cpp"
  )

  rcl_add_custom_gtest(test_init${target_suffix}
    SRCS rcl/test_init.cpp
    ENV ${rmw_implementation_env_var} ${memory_tools_ld_preload_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    LIBRARIES ${PROJECT_NAME} osrf_testing_tools_cpp::memory_tools mimick
    AMENT_DEPENDENCIES ${rmw_implementation}
  )

  rcl_add_custom_gtest(test_node${target_suffix}
    SRCS rcl/test_node.cpp
    ENV ${rmw_implementation_env_var} ${memory_tools_ld_preload_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    LIBRARIES ${PROJECT_NAME} mimick osrf_testing_tools_cpp::memory_tools
    AMENT_DEPENDENCIES ${rmw_implementation} "osrf_testing_tools_cpp"
    TIMEOUT 240  # Large timeout to wait for fault injection tests
  )

  rcl_add_custom_gtest(test_arguments${target_suffix}
    SRCS rcl/test_arguments.cpp
    ENV ${rmw_implementation_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../src/rcl/
    LIBRARIES ${PROJECT_NAME} osrf_testing_tools_cpp::memory_tools
    AMENT_DEPENDENCIES "osrf_testing_tools_cpp" "rcpputils"
  )

  rcl_add_custom_gtest(test_remap${target_suffix}
    SRCS rcl/test_remap.cpp
    ENV ${rmw_implementation_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../src/rcl/
    LIBRARIES ${PROJECT_NAME}
    AMENT_DEPENDENCIES "osrf_testing_tools_cpp"
  )

  rcl_add_custom_gtest(test_remap_integration${target_suffix}
    SRCS rcl/test_remap_integration.cpp
    TIMEOUT 200
    ENV ${rmw_implementation_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    LIBRARIES ${PROJECT_NAME}
    AMENT_DEPENDENCIES "osrf_testing_tools_cpp" "test_msgs"
  )

  rcl_add_custom_gtest(test_guard_condition${target_suffix}
    SRCS rcl/test_guard_condition.cpp
    ENV ${rmw_implementation_env_var} ${memory_tools_ld_preload_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    LIBRARIES ${PROJECT_NAME} mimick osrf_testing_tools_cpp::memory_tools
    AMENT_DEPENDENCIES ${rmw_implementation} "osrf_testing_tools_cpp"
  )

  rcl_add_custom_gtest(test_publisher${target_suffix}
    SRCS rcl/test_publisher.cpp
    ENV ${rmw_implementation_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../src/rcl/
    LIBRARIES ${PROJECT_NAME} mimick
    AMENT_DEPENDENCIES ${rmw_implementation} "osrf_testing_tools_cpp" "test_msgs"
  )

  rcl_add_custom_gtest(test_service${target_suffix}
    SRCS rcl/test_service.cpp rcl/wait_for_entity_helpers.cpp
    ENV ${rmw_implementation_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    LIBRARIES ${PROJECT_NAME} mimick
    AMENT_DEPENDENCIES ${rmw_implementation} "osrf_testing_tools_cpp" "test_msgs"
  )

  rcl_add_custom_gtest(test_subscription${target_suffix}
    SRCS rcl/test_subscription.cpp rcl/wait_for_entity_helpers.cpp
    ENV ${rmw_implementation_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    LIBRARIES ${PROJECT_NAME} mimick
    AMENT_DEPENDENCIES ${rmw_implementation} "osrf_testing_tools_cpp" "test_msgs"
  )
  if(rmw_implementation STREQUAL "rmw_fastrtps_cpp" OR
    rmw_implementation STREQUAL "rmw_fastrtps_dynamic_cpp")
    message(STATUS "Enabling message timestamp test for ${rmw_implementation}")
    target_compile_definitions(test_subscription${target_suffix}
      PUBLIC "RMW_TIMESTAMPS_SUPPORTED=1" "RMW_RECEIVED_TIMESTAMP_SUPPORTED=1")
    target_compile_definitions(test_service${target_suffix}
      PUBLIC "RMW_TIMESTAMPS_SUPPORTED=1" "RMW_RECEIVED_TIMESTAMP_SUPPORTED=1")
  else()
    if(rmw_implementation STREQUAL "rmw_cyclonedds_cpp")
      message(STATUS "Enabling only source timestamp test for ${rmw_implementation}")
      target_compile_definitions(test_subscription${target_suffix}
        PUBLIC "RMW_TIMESTAMPS_SUPPORTED=1")
      target_compile_definitions(test_service${target_suffix}
        PUBLIC "RMW_TIMESTAMPS_SUPPORTED=1")
      # TODO(tfoote) Disable this tests on CI as it's being flakey
      # This should be removed when https://github.com/ros2/rmw_cyclonedds/issues/185 is resolved.
      ament_add_test_label(test_service${target_suffix} xfail)
    else()
      message(STATUS "Disabling message timestamp test for ${rmw_implementation}")
    endif()
  endif()

  rcl_add_custom_gtest(test_events${target_suffix}
    SRCS rcl/test_events.cpp
    ENV ${rmw_implementation_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    LIBRARIES ${PROJECT_NAME}
    AMENT_DEPENDENCIES ${rmw_implementation} "osrf_testing_tools_cpp" "test_msgs"
  )

  rcl_add_custom_gtest(test_wait${target_suffix}
    SRCS rcl/test_wait.cpp
    ENV ${rmw_implementation_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    LIBRARIES ${PROJECT_NAME} mimick
    AMENT_DEPENDENCIES ${rmw_implementation} "osrf_testing_tools_cpp"
  )

  rcl_add_custom_gtest(test_logging_rosout${target_suffix}
    SRCS rcl/test_logging_rosout.cpp
    ENV ${rmw_implementation_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    LIBRARIES ${PROJECT_NAME}
    AMENT_DEPENDENCIES ${rmw_implementation} "osrf_testing_tools_cpp" "rcl_interfaces"
  )

  rcl_add_custom_gtest(test_namespace${target_suffix}
    SRCS test_namespace.cpp
    ENV ${rmw_implementation_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    LIBRARIES ${PROJECT_NAME}
    AMENT_DEPENDENCIES ${rmw_implementation} "osrf_testing_tools_cpp" "test_msgs"
  )

  rcl_add_custom_gtest(test_rmw_impl_id_check_func${target_suffix}
    SRCS rcl/test_rmw_impl_id_check_func.cpp
    ENV ${rmw_implementation_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    LIBRARIES ${PROJECT_NAME}
    AMENT_DEPENDENCIES ${rmw_implementation}
  )

  # Launch tests

  rcl_add_custom_executable(service_fixture${target_suffix}
    SRCS rcl/service_fixture.cpp rcl/wait_for_entity_helpers.cpp
    LIBRARIES ${PROJECT_NAME}
    AMENT_DEPENDENCIES ${rmw_implementation} "osrf_testing_tools_cpp" "test_msgs"
  )

  rcl_add_custom_executable(client_fixture${target_suffix}
    SRCS rcl/client_fixture.cpp rcl/wait_for_entity_helpers.cpp
    LIBRARIES ${PROJECT_NAME}
    AMENT_DEPENDENCIES ${rmw_implementation} "osrf_testing_tools_cpp" "test_msgs"
  )

  rcl_add_custom_launch_test(test_services
    service_fixture
    client_fixture
    ENV ${rmw_implementation_env_var}
    APPEND_LIBRARY_DIRS ${extra_lib_dirs}
    TIMEOUT 15
  )

  set(SKIP_TEST "")
  if(WIN32)
    # TODO(dhood): launch does not set the return code correctly for these tests on Windows.
    # See https://github.com/ros2/launch/issues/66
    set(SKIP_TEST "SKIP_TEST")
  endif()
  set(TEST_RMW_IMPL_ID_CHECK_EXECUTABLE_NAME "$<TARGET_FILE:test_rmw_impl_id_check_exe>")
  configure_file(
    rcl/test_rmw_impl_id_check.py.in
    ${CMAKE_CURRENT_BINARY_DIR}/test_rmw_impl_id_check${target_suffix}.py.configure
    @ONLY
  )
  file(GENERATE
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test/test_rmw_impl_id_check${target_suffix}_$<CONFIG>.py"
    INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_rmw_impl_id_check${target_suffix}.py.configure"
  )
  add_launch_test(
    "${CMAKE_CURRENT_BINARY_DIR}/test/test_rmw_impl_id_check${target_suffix}_$<CONFIG>.py"
    TARGET test_rmw_impl_id_check${target_suffix}
    APPEND_LIBRARY_DIRS "${extra_lib_dirs}"
    ${SKIP_TEST}
  )
  if(TEST test_rmw_impl_id_check${target_suffix})
    set_tests_properties(
      test_rmw_impl_id_check${target_suffix}
      PROPERTIES DEPENDS test_rmw_impl_id_check_exe
    )
  endif()

endfunction()

# Build simple executable for using in the test_rmw_impl_id_check
add_executable(test_rmw_impl_id_check_exe
  rcl/test_rmw_impl_id_check_exe.cpp)
target_link_libraries(test_rmw_impl_id_check_exe ${PROJECT_NAME})

call_for_each_rmw_implementation(test_target)

rcl_add_custom_gtest(test_validate_enclave_name
  SRCS rcl/test_validate_enclave_name.cpp
  APPEND_LIBRARY_DIRS ${extra_lib_dirs}
  LIBRARIES ${PROJECT_NAME} mimick
)

rcl_add_custom_gtest(test_domain_id
  SRCS rcl/test_domain_id.cpp
  APPEND_LIBRARY_DIRS ${extra_lib_dirs}
  LIBRARIES ${PROJECT_NAME}
)

rcl_add_custom_gtest(test_localhost
  SRCS rcl/test_localhost.cpp
  APPEND_LIBRARY_DIRS ${extra_lib_dirs}
  LIBRARIES ${PROJECT_NAME}
)

rcl_add_custom_gtest(test_logging
  SRCS rcl/test_logging.cpp
  APPEND_LIBRARY_DIRS ${extra_lib_dirs}
  LIBRARIES ${PROJECT_NAME} mimick
  AMENT_DEPENDENCIES "osrf_testing_tools_cpp"
)

rcl_add_custom_gtest(test_validate_topic_name
  SRCS rcl/test_validate_topic_name.cpp
  APPEND_LIBRARY_DIRS ${extra_lib_dirs}
  LIBRARIES ${PROJECT_NAME}
)

rcl_add_custom_gtest(test_expand_topic_name
  SRCS rcl/test_expand_topic_name.cpp
  APPEND_LIBRARY_DIRS ${extra_lib_dirs}
  LIBRARIES ${PROJECT_NAME} mimick
)

rcl_add_custom_gtest(test_security
  SRCS rcl/test_security.cpp
  APPEND_LIBRARY_DIRS ${extra_lib_dirs}
  LIBRARIES ${PROJECT_NAME} mimick
  AMENT_DEPENDENCIES "osrf_testing_tools_cpp"
)

rcl_add_custom_gtest(test_common
  SRCS rcl/test_common.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../src/rcl/common.c
  APPEND_LIBRARY_DIRS ${extra_lib_dirs}
  INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../src/rcl/
  LIBRARIES ${PROJECT_NAME}
)

# Install test resources
install(DIRECTORY ${test_resources_dir_name}
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
