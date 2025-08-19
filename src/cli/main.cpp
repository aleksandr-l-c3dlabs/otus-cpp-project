#include <boost/program_options.hpp>
#include <format>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

#include "camera.h"
#include "model.h"
#include "ppm.h"
#include "renderer.h"

using namespace std::string_view_literals;
using namespace rtr;

namespace po = boost::program_options;

struct Vec3 {
  float x;
  float y;
  float z;
};

namespace boost {

template <>
std::string lexical_cast(const Vec3& arg) {
  return std::format("{},{},{}", arg.x, arg.y, arg.z);
}

template <>
std::string lexical_cast(const std::optional<Vec3>& arg) {
  return arg.has_value() ? lexical_cast<std::string>(arg.value()) : "";
}

}  // namespace boost

void validate(boost::any& v, const std::vector<std::string>& values, Vec3*,
              int) {
  po::validators::check_first_occurrence(v);
  const std::string& s = po::validators::get_single_string(values);
  Vec3 vec;
  char delim1, delim2;

  std::stringstream ss(s);
  if (!(ss >> vec.x >> delim1 >> vec.y >> delim2 >> vec.z) || delim1 != ',' ||
      delim2 != ',') {
    throw po::validation_error(po::validation_error::invalid_option_value);
  }

  v = vec;
}

template <typename T>
void validate(boost::any& v, const std::vector<std::string>& values,
              std::optional<T>*, int) {
  if (values.size() == 1 && values[0].empty()) {
    v = boost::any(std::optional<T>());
    return;
  }

  boost::any temp;
  validate(temp, values, (T*)nullptr, 0);

  if (temp.empty()) {
    throw po::validation_error(po::validation_error::invalid_option_value);
  }

  v = std::optional<T>(boost::any_cast<T>(temp));
}

int main(const int argc, const char* argv[]) {
  po::variables_map vm;
  po::options_description desc("Available options");
  desc.add_options()("help,h", "Produce help message")(
      "model,m", po::value<std::string>()->required(), "Model file path")(
      "output,o", po::value<std::string>()->required(), "Output ppm file path")(
      "position,p", po::value<std::optional<Vec3>>()->default_value({}),
      "Camera position")("up,u", po::value<Vec3>()->default_value({0, 1, 0}),
                         "Camera up vector")(
      "direction,d", po::value<Vec3>()->default_value({0, 0, -1}),
      "Camera direction vector")(
      "width,w", po::value<size_t>()->default_value(400), "Viewport width")(
      "height,h", po::value<size_t>()->default_value(300), "Viewport height")(
      "threads,t", po::value<size_t>()->default_value(4), "Used thread count");

  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
  } catch (const std::exception& er) {
    std::cout << "ERR: Parse options error: " << er.what() << std::endl;
    return 1;
  }

  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return 0;
  }

  auto m = vm["model"].as<std::string>();
  auto o = vm["output"].as<std::string>();
  auto p = vm["position"].as<std::optional<Vec3>>();
  Vec3 pos = p.has_value() ? p.value() : Vec3{0, 0, 0};
  auto dir = vm["direction"].as<Vec3>();
  auto up = vm["up"].as<Vec3>();
  auto w = vm["width"].as<size_t>();
  auto h = vm["height"].as<size_t>();
  auto t = vm["threads"].as<size_t>();

  auto camera = std::make_shared<Camera>(Camera{{pos.x, pos.y, pos.z},
                                                {dir.x, dir.y, dir.z},
                                                {up.x, up.y, up.z},
                                                60.f,
                                                float(w) / h});
  auto model = Model::import(m);
  if (!model.has_value()) {
    std::cout << "ERR: Can't open model file " << m << std::endl;
    return 2;
  }
  Renderer renderer(std::make_shared<const Model>(model.value()), camera, w, h);
  if (!p.has_value()) {
    camera->zoom_to_fit();
  }

  auto progress_callback = [](float progress) {
    std::cout << "Progress: " << int(progress * 100) << "%\r" << std::flush;
  };
  renderer.render(t, progress_callback);

  const auto& frame_buffer = renderer.get_frame_buffer();
  std::ofstream ofs(o.data(), std::ios::binary);
  ppm_export(ofs, frame_buffer);

  return 0;
}