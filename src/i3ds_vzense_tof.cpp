///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2022 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
////////////////////////////////////////////////////////////////////////////////

#include <boost/program_options/value_semantic.hpp>
#include <unistd.h>

#include <csignal>
#include <cstddef>
#include <iostream>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#ifndef BOOST_LOG_DYN_LINK
#define BOOST_LOG_DYN_LINK
#endif  // BOOST_LOG_DYN_LINK

#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include <boost/program_options.hpp>

#include <i3ds/communication.hpp>
#include <i3ds/configurator.hpp>
#include <i3ds/exception.hpp>

#include "vzense_camera.hpp"
#include "vzense_helpers.hpp"

namespace po = boost::program_options;

volatile bool running;

void signal_handler(int signum) {
  BOOST_LOG_TRIVIAL(info) << "do_deactivate()";
  running = false;
}

struct counter {
  int count = 0;
};
void validate(boost::any& v, std::vector<std::string> const& xs, counter*, long) {
  if (v.empty()) {
    counter num;
    num.count = 1;
    v = num;
  } else {
    ++boost::any_cast<counter&>(v).count;
  }
}

int main(int argc, char** argv) {
  unsigned int node_id;
  bool no_ir = false;

  i3ds::VzenseCamera::Parameters param;
  i3ds::Configurator configurator;

  po::options_description desc("Allowed camera control options");
  configurator.add_common_options(desc);

  desc.add_options()
  ("node,n", po::value<unsigned int>(&node_id)->default_value(10), "Node ID of camera")
  ("camera-name,c", po::value<std::string>(&param.camera_name), "Connect via (UserDefinedName) of Camera")
  ("no-ir", po::bool_switch(&no_ir), "Disable IR output")
  ("print,p", "Print the camera configuration")
  ;

  po::variables_map vm = configurator.parse_common_options(desc, argc, argv);

  if (vm.count("print")) {
    print_available_cameras();
    return 0;
  }

  param.ir_output = !no_ir;

  BOOST_LOG_TRIVIAL(info) << "Node ID:     " << node_id;
  BOOST_LOG_TRIVIAL(info) << "ToF name: " << param.camera_name;
  BOOST_LOG_TRIVIAL(info) << "ToF type: Vzense";

  i3ds::Context::Ptr context = i3ds::Context::Create();

  i3ds::Server server(context);

  i3ds::VzenseCamera camera(context, node_id, param);

  camera.Attach(server);

  running = true;
  signal(SIGINT, signal_handler);

  server.Start();

  while (running) {
    sleep(1);
  }

  server.Stop();
  return 0;
}
