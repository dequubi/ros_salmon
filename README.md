# ros_salmon (SLNAM)

Проект по реализации одновременной локализации, навигации и создания карты при помощи ROS.

Демонстрация производится при помощи Gazebo и Turtlebot3.

Глобальные зависимости: `scipy`

---

## Создание проекта

```sh
cd ~/mag_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git -b noetic_devel
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git -b noetic_devel
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git -b noetic_devel
git clone https://github.com/dequubi/ros_salmon.git
cd ~/mag_ws && catkin_make
```

Затем добавить `devel/setup.bash` в `~/.bashrc`

## Запуск модели

### Запуск виртуального окружения

```sh
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### Запуск окна для решения SLAM и навигации

```sh
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_slam turtlebot3_slam.launch
```

### Запуск пакета навигации

```sh
rosrun dqb_navigation dqb_path.py
```

Цель устанавливается сообщением в topic `/move_base_simple/goal`.

Путь публикуется в topic `/dqb/path`.