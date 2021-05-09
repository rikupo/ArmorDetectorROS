インターンシップ応募にあたり一時的に公開許可をもらい公開中  
主に装甲板認識部を担当(armor_detector,yolo)

# 装甲板認識プログラム
### 環境構築
- Ubuntu18.04 or later
- ROS Noetic
- Python3.8 or later
- trollius rosdep rospkg rosinstall_generator rosinstall wstool vcstools catkin_pkg
- python3-catkin-tools
- opencv4
Dockerが楽

## ArmorDetector
装甲の幾何学的性質より認識
参考(深セン大学:)

## Yolo
CNN(Yolov5)による認識

# ToDo
- 軽量化: ros-topicの画像圧縮
- 軽量化: pytorchモデルをONNXに変換
