インターンシップ応募にあたり一時的に公開許可をもらい公開中  
主に装甲板認識部を担当(armor_detector,yolo)

# Overview
RoboMasterでの自動照準用の装甲版認識プログラム
### 担当箇所
- armor_detector :装甲版認識1(opencv)
- yolo :装甲版認識2(Yolov5)
- img_proc :デバッグ用映像表示
- movie_publisher :デバッグ用映像入力
### チームメイト作成
- serial_com :TypeAシリアル通信to機体制御マイコン
- 
- industrial_camera_publisher :産業用カメラから映像取得

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
参考: 深セン大学(https://github.com/yarkable/RP_Infantry_Plus)

## Yolo
CNN(Yolov5)による認識
Yolov5: https://github.com/ultralytics/yolov5
データセット:https://terra-1-g.djicdn.com/b2a076471c6c4b72b574a977334d3e05/resources/DJI%20ROCO.zip
データセット変換:https://qiita.com/harmegiddo/items/d4387ae5aba8f9bcd354 （


# ToDo
- 軽量化: ros-topicの画像圧縮
- 軽量化: pytorchモデルをONNXに変換
