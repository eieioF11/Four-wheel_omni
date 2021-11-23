# Four-wheel_omni
オムニ４輪シミュレーション\
※製作途中\
![robotmodel](/image/robotmodel.png)
## 環境構築
```bash
sudo apt-get install ros-melodic-effort-controllers
sudo apt-get install ros-melodic-joint-state-controller
sudo apt-get install ros-melodic-position-controllers
sudo apt install ros-melodic-ros-control
sudo apt install ros-melodic-ros-controllers
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
### フィールドを指定する場合
スタートゾーンの色が赤の場合
```bash
roslaunch Four-wheel_omni gazebo.launch Field:=Red
```
スタートゾーンの色が青の場合
```bash
roslaunch Four-wheel_omni gazebo.launch Field:=Blue
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

## 問題対処法メモ
フィールドを指定した場合にworldに何も表示されないときは以下のコマンドを入力
```bash
export GAZEBO_RESOURCE_PATH=`pwd`
```