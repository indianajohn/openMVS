#include "../../libs/MVS/Common.h"
#define _USE_OPENCV
#include "../../libs/MVS/Interface.h"
#include "json.hpp"
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

using namespace MVS;
using namespace boost::program_options;
typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> EMat33d;
typedef Eigen::Matrix<double, 3, 1> EVec3d;

Interface parse(const std::string &path) {
  Interface scene;
  nlohmann::json json;
  std::ifstream stream(path);
  boost::filesystem::path parent_dir(path);
  boost::filesystem::path cameras = parent_dir / "cameras";
  boost::filesystem::path images = parent_dir / "images";
  for (boost::filesystem::directory_iterator itr(cameras);
       itr != boost::filesystem::directory_iterator(); itr++) {
    // Load JSON file
    nlohmann::json json;
    std::ifstream stream(itr->path().string());
    stream >> json;

    // Read image so we know the size
    boost::filesystem::path image_path =
        images / itr->path().leaf().replace_extension("png");
    cv::Mat image = cv::imread(image_path.string());

    Interface::Platform::Camera camera;
    camera.name = image_path.string();
    camera.K = camera.K.eye();
    camera.K(0, 0) = json["fx"];
    camera.K(0, 2) = json["cx"];
    camera.K(1, 1) = json["fy"];
    camera.K(1, 2) = json["cy"];
    const REAL fScale(REAL(1) /
                      Interface::Platform::Camera::GetNormalizationScale(
                          image.cols, image.rows));
    camera.K(0, 0) *= fScale;
    camera.K(1, 1) *= fScale;
    camera.K(0, 2) *= fScale;
    camera.K(1, 2) *= fScale;
    std::cout << camera.K << std::endl;
    camera.R = Interface::Mat33d::eye();
    camera.C = Interface::Pos3d(0, 0, 0);

    // Platform & image
    Interface::Platform platform;
    Interface::Platform::Pose pose;
    pose.R << json["t_00"].get<double>(), json["t_01"].get<double>(),
        json["t_02"].get<double>(), json["t_10"].get<double>(),
        json["t_11"].get<double>(), json["t_12"].get<double>(),
        json["t_20"].get<double>(), json["t_21"].get<double>(),
        json["t_22"].get<double>();
    Eigen::Map<EMat33d> R(pose.R.val);
    R = R * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
    pose.C = Interface::Pos3d(json["t_03"], json["t_13"], json["t_23"]);
    EnsureRotationMatrix((Matrix3x3d &)pose.R);
    Interface::Image mvs_image;
    mvs_image.name = MAKE_PATH_SAFE(image_path.string().c_str());
    mvs_image.platformID = scene.platforms.size();
    mvs_image.cameraID = 0;
    mvs_image.ID = scene.images.size();
    mvs_image.poseID = (uint32_t)platform.poses.size();
    std::cout << "platform=" << mvs_image.platformID
              << "/camera=" << mvs_image.cameraID << "/image=" << mvs_image.ID
              << "/pose=" << mvs_image.poseID << pose.R << pose.C << std::endl;

    // Add everything
    platform.poses.push_back(pose);
    platform.cameras.push_back(camera);
    scene.platforms.push_back(platform);
    scene.images.push_back(mvs_image);
  }
  return scene;
}
int main(int argc, LPCTSTR *argv) {
  try {
    options_description desc{"Options"};
    desc.add_options()("help,h", "Help screen")(
        "in", value<std::string>()->required(), "Input directory")(
        "out", value<std::string>()->required(), "Output directory");

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if (vm.count("help")) {
      std::cout << desc << '\n';
      return EXIT_SUCCESS;
    }
    std::ofstream stream(vm["out"].as<std::string>());
    Interface scene = parse(vm["in"].as<std::string>());
    if (!ARCHIVE::SerializeSave(scene, vm["out"].as<std::string>(), 0)) {
      return EXIT_FAILURE;
    }
  } catch (const error &ex) {
    std::cerr << ex.what() << '\n';
  }
  return EXIT_SUCCESS;
}
