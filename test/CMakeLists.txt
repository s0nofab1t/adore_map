# Tests for adore_map
# Expects:
#   - ADORE_TEST_LIB_TARGET set by parent CMakeLists.txt
#   - ament_cmake_gtest already found

file(GLOB ADORE_MAP_TEST_SOURCES CONFIGURE_DEPENDS
  "*.cpp"
)

foreach(test_src ${ADORE_MAP_TEST_SOURCES})
  get_filename_component(test_name ${test_src} NAME_WE)

  ament_add_gtest(${test_name} ${test_src})

  if(TARGET ${test_name})
    # ament_add_gtest uses the plain target_link_libraries signature,
    # so we must also use the plain signature here.
    target_link_libraries(${test_name}
      ${ADORE_TEST_LIB_TARGET}
    )

    target_include_directories(${test_name}
      PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
    )
  endif()
endforeach()
