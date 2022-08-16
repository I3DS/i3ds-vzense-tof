find_package (Boost COMPONENTS program_options log REQUIRED)
find_package (OpenCV 4.0.0)

set(TARGET i3ds_vzense_tof)

set (SRCS
  vzense_camera.cpp
  )

find_library(VZENSE_LIBS vzense_api HINTS "../thirdparty/Vzense_SDK_Linux/Ubuntu18.04/Lib")
if (NOT VZENSE_LIBS)
  message(FATAL_ERROR "Could not find vzense library.")
endif(NOT VZENSE_LIBS)

find_path(VZENSE_INCLUDE "Vzense_api2.h" HINTS "../thirdparty/Vzense_SDK_Linux/Ubuntu18.04/Include")
if (NOT VZENSE_INCLUDE)
  message(FATAL_ERROR "Could not find vzense includes")
endif (NOT VZENSE_INCLUDE)

set(VZENSE_LDLIBS "-lvzense_api")

set (LIBS
  ${I3DS_LIBS}
  zmq
  pthread
  ${VZENSE_LIBS}
  ${Boost_LIBRARIES}
  )


include_directories ("../include/")
include_directories (SYSTEM ${I3DS_INCLUDE})
include_directories (${VZENSE_INCLUDE})


add_executable(${TARGET} i3ds_vzense_tof.cpp ${SRCS})
target_compile_options(${TARGET} PRIVATE ${VZENSE_CXXFLAGS} -Wno-unknown-pragmas)
target_link_libraries (${TARGET} ${VZENSE_LDFLAGS} ${LIBS})
install(TARGETS ${TARGET} DESTINATION bin)

add_executable(application_demo application_demo.cpp)
target_link_libraries (application_demo ${VZENSE_LDFLAGS} ${LIBS})