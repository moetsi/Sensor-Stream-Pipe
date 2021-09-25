# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.16.3)

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(ssp_dep-populate NONE)

include(ExternalProject)
ExternalProject_Add(ssp_dep-populate
                     "UPDATE_DISCONNECTED" "False" "URL" "https://moetsiblob.blob.core.windows.net/binaries-ssp/202109242040__ssp_lindep.tar.gz" "URL_HASH" "MD5=5ef969687a6fdd37d258d5f84ff1e240"
                    SOURCE_DIR          "/home/adam/dev/Sensor-Stream-Pipe/build/_deps/ssp_dep-src"
                    BINARY_DIR          "/home/adam/dev/Sensor-Stream-Pipe/build/_deps/ssp_dep-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
)
