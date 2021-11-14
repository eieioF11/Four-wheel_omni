# Four-wheel_omni
オムニ４輪シミュレーション\
※製作途中\
![robotmodel](/image/robotmodel.png)
## キーボード操作
```bash
sudo apt-get install ros-melodic-teleop-twist-keyboard
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
## navigation
```bash
roslaunch Four-wheel_omni navi.launch
```