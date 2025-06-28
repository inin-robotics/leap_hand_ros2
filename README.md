# LEAP Hand ROS2 description and driver packages

## 安装

1. 通过插拔确定串口号，然后使用以下命令确定`ATTRS{serial}`：
  ```shell
  udevadm info -a -n /dev/ttyUSB0 | grep serial
  # 示例输出，FTA2W8CB需要的内容（另一个我不知道可不可以）
  #  SUBSYSTEMS=="usb-serial"
  #  ATTRS{serial}=="FTA2W8CB"
  #  ATTRS{serial}=="0000:00:14.0"
  ```
  将`ATTRS{serial}`填入`scripts/99-leap-hand.rules`，另外可以根据需要修改symlink名称。
2. 安装udev rules：
  ```shell
  sudo scripts/install_udev_rules.sh
  ```
3. 启动docker容器：
  ```shell
  cd docker
  docker compose up -d
  ```
4. 进入容器，通过`ros2 launch`启动单手或双手脚本，具体参数见launch文件。

## TODO

- [ ] 当前的驱动和urdf实现功能少且不符合ROS2惯例，考虑使用[ros2control dynamixel](https://github.com/dynamixel-community/dynamixel_hardware)改写

## Notes

- ININ实验室：右手安装有问题，代码中有fixme注释