# Four-wheel_omni
オムニ４輪シミュレーション\
※製作途中\
![robotmodel](/image/robotmodel.png)
## 環境構築
```bash
sudo apt-get install ros-melodic-effort-controllers
sudo apt-get install ros-melodic-joint-state-controller
sudo apt-get install ros-melodic-position-controllers
```
```bash
sudo apt-get install ros-melodic-teleop-twist-keyboard
```
## キーボード操作
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
## Gazebo
```bash
roslaunch Four-wheel_omni gazebo.launch
```
## オムニ四輪コントローラー
```bash
rosrun Four-wheel_omni omuni4controller.py
```
## モデルの表示
xacroディレクトリに移動し以下のコマンドを実行
```bash
roslaunch urdf_tutorial display.launch model:=OmuniRobot.xacro
```
