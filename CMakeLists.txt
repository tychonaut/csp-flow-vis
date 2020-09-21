# ------------------------------------------------------------------------------------------------ #
#                                This file is part of CosmoScout VR                                #
#       and may be used under the terms of the MIT license. See the LICENSE file for details.      #
#                         Copyright: (c) 2019 German Aerospace Center (DLR)                        #
# ------------------------------------------------------------------------------------------------ #

option(CSP_FLOW_VIS "Enable compilation of this plugin" ON)

if (NOT CSP_FLOW_VIS)
  return()
endif()


find_package(GLUT REQUIRED)

# build plugin -------------------------------------------------------------------------------------

file(GLOB SOURCE_FILES src/*.cpp)

# Resource files and header files are only added in order to make them available in your IDE.
file(GLOB HEADER_FILES src/*.hpp)

add_library(csp-flow-vis SHARED
  ${SOURCE_FILES}
  ${HEADER_FILES}
)

target_link_libraries(csp-flow-vis
  PUBLIC
    cs-core
	glu32
)

# Add this Plugin to a "plugins" folder in your IDE.
set_property(TARGET csp-flow-vis PROPERTY FOLDER "plugins")

# Make directory structure available in your IDE.
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES 
  ${SOURCE_FILES} ${HEADER_FILES}
)

# install plugin -----------------------------------------------------------------------------------

install(TARGETS   csp-flow-vis DESTINATION "share/plugins")
