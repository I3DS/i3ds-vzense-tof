find_package (Boost COMPONENTS program_options log REQUIRED)
find_package (OpenCV 4.0.0)

set(TARGET i3ds_vzense_tof)

set (SRCS
  vzense_camera.cpp
  )

find_library(VZENSE_LIBS vzense_api HINTS "../thirdparty/vzense/libs")

if (NOT VZENSE_LIBS)
  message(FATAL_ERROR "Could not find vzense library.")
endif(NOT VZENSE_LIBS)

set(VZENSE_LDLIBS "-lvzense_api")

set (LIBS
  i3ds
  i3ds_asn1
  i3ds_clients
  zmq
  pthread
  ${VZENSE_LIBS}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  )


include_directories ("../include/")
include_directories ("../thirdparty/vzense/include/")


add_executable(${TARGET} i3ds_vzense_tof.cpp ${SRCS})
target_compile_options(${TARGET} PRIVATE ${VZENSE_CXXFLAGS} -Wno-unknown-pragmas)
target_link_libraries (${TARGET} ${VZENSE_LDFLAGS} ${LIBS})
install(TARGETS ${TARGET} DESTINATION bin)
