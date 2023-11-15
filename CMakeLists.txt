# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
#project(main)
project(sample_project)
idf_build_get_property(python PYTHON)
idf_build_get_property(project_dir PROJECT_DIR)
idf_build_get_property(project_name PROJECT_NAME)
message(STATUS "The Python intepreter is: ${python}")
message(STATUS "The PROJECT_DIR is: ${project_dir}")
message(STATUS "The PROJECT_NAME is: ${project_name}")

