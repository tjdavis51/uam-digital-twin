cmake_minimum_required(VERSION 3.16)
project(uam_core VERSION 0.1.0 LANGUAGES CXX)
include(CTest)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
option(UAM_REQUIRE_ALGORITHMS "Treat unfinished user algorithms as test failures" OFF)
add_library(uam_core src/core.cpp)
add_library(uam_core::uam_core ALIAS uam_core)
target_compile_features(uam_core PUBLIC cxx_std_17)
target_include_directories(uam_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(uam_core PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
if(BUILD_TESTING)
  add_executable(uam_core_tests test/core_test.cpp)
  target_link_libraries(uam_core_tests PRIVATE uam_core)
  add_test(NAME core_contract COMMAND uam_core_tests contract)
  foreach(algorithm pd hold smooth)
    add_test(NAME acceptance_${algorithm} COMMAND uam_core_tests ${algorithm})
    if(NOT UAM_REQUIRE_ALGORITHMS)
      set_tests_properties(acceptance_${algorithm} PROPERTIES SKIP_RETURN_CODE 77)
    endif()
  endforeach()
endif()
install(TARGETS uam_core EXPORT uam_coreTargets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT uam_coreTargets NAMESPACE uam_core:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/uam_core)
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/uam_coreConfig.cmake"
  "include(\"\${CMAKE_CURRENT_LIST_DIR}/uam_coreTargets.cmake\")\n")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/uam_coreConfig.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/uam_core)
write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/uam_coreConfigVersion.cmake"
  COMPATIBILITY SameMajorVersion)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/uam_coreConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/uam_core)
