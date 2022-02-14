find_package (Boost COMPONENTS program_options log REQUIRED)

set(TARGET i3ds_vzense_tof)

set (SRCS
  vzense_camera.cpp
  )

set (LIBS
  i3ds
  i3ds_asn1
  i3ds_clients
  zmq
  pthread
  ${VZENSE_LDLIBS}
  ${Boost_LIBRARIES}
  )

include_directories ("../include/")


add_executable(${TARGET} i3ds_vzense_tof.cpp ${SRCS})
target_compile_options(${TARGET} PRIVATE ${VZENSE_CXXFLAGS} -Wno-unknown-pragmas)
target_link_libraries (${TARGET} ${VZENSE_LDFLAGS} ${LIBS})
install(TARGETS ${TARGET} DESTINATION bin)
