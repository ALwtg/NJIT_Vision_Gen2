#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
// #include "io/ros2/publish2nav.hpp"
// #include "io/ros2/ros2.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);

  // io::ROS2 ros2;
  io::CBoard cboard(config_path);
  io::Camera camera(config_path);
  io::USBCamera back_camera("video4", config_path);
  io::USBCamera usbcam1("video0", config_path);
  io::USBCamera usbcam2("video2", config_path);

  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  omniperception::Decider decider(config_path);

  cv::Mat img;

  std::chrono::steady_clock::time_point timestamp;
  io::Command last_command;
  io::Command command;

  while (!exiter.exit()) {
    camera.read(img, timestamp);
    Eigen::Quaterniond q = cboard.imu_at(timestamp - 1ms);
    // recorder.record(img, q, timestamp);//数据录制

    /// 自瞄核心逻辑
    solver.set_R_gimbal2world(q);
    //把旋转矩阵转换成按照 Z-Y-X 顺序表示的欧拉角
    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);//2 1 0表示z y x

    auto armors = yolo.detect(img);

    //通过 ROS2 获取敌方状态，识别无敌装甲板
    // decider.get_invincible_armor(ros2.subscribe_enemy_status());

    decider.armor_filter(armors);

    //通过 ROS2 接收指定自瞄目标
    // decider.get_auto_aim_target(armors, ros2.subscribe_autoaim_target());

    decider.set_priority(armors);

    auto targets = tracker.track(armors, timestamp);

    // io::Command command{false, false, 0, 0};
    //#####################################################################################
    /// 全向感知逻辑（此处逻辑似乎有问题）##########################################################3
    if (tracker.state() == "lost")//如果目标丢失
    {
      command = decider.decide(yolo, gimbal_pos, usbcam1, usbcam2, back_camera); //全向感知轮训搜索目标
    }
    else                          //如果发现目标
    {
      command = aimer.aim(targets, timestamp, cboard.bullet_speed, cboard.shoot_mode);//自瞄，计算云台瞄准角
    }
    //#####################################################################################
    
    
    
    /// 发射逻辑
    command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);

    cboard.send(command);

    // /// ROS2通信
    // Eigen::Vector4d target_info = decider.get_target_info(armors, targets);

    // ros2.publish(target_info);
  }
  return 0;
}