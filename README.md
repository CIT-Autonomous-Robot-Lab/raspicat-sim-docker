# raspicat-sim-docker
ROS 2（humble）でRaspberry Pi Catのシミュレータ使うためのDockerfileです。

sshkeyの設定は[こちらのサイト](https://qiita.com/shizuma/items/2b2f873a0034839e47ce)を参照してください。

## dockerの環境構築
### 1. dockerのインストール
```
sudo apt install docker.io
```

### 2. dockerのコマンドをsudoなしで実行するための設定
* dockerグループに現在のユーザを追加

```
sudo gpasswd -a $USER docker
sudo reboot
```

PCを再起動すると  
sudoなしでdockerコマンドを使用できるようになります。

## コンテナを起動

### 1. イメージの作成

* GPUあり
```
git clone https://github.com/CIT-Autonomous-Robot-Lab/raspicat-sim-docker
cd raspicat-sim-docker/ros2-gpu 
docker build --build-arg USERNAME=$USER -t raspicat-sim:humble -f Dockerfile .
```

* GPUなし
```
git clone https://github.com/CIT-Autonomous-Robot-Lab/raspicat-sim-docker
cd raspicat-sim-docker/ros2-cpu
docker build --build-arg USERNAME=$USER -t raspicat-sim:humble -f Dockerfile .
```

* 備考
  * raspicat: 作成するdockerイメージの名前
  * humble: 作成するdockerイメージのTAG名

### 2. GUIを使用するためにXサーバへのアクセス許可
```
xhost +local:docker
```

### 3. コンテナ起動
* GPUあり
```
docker run --rm -it \
           -u $(id -u):$(id -g) \
           --gpus all \
           --privileged \
           --net=host \
           --ipc=host \
           --env="DISPLAY=$DISPLAY" \
           --mount type=bind,source=/home/$USER/.ssh,target=/home/$USER/.ssh \
           --mount type=bind,source=/home/$USER/.gitconfig,target=/home/$USER/.gitconfig \
           --mount type=bind,source=/usr/share/zoneinfo/Asia/Tokyo,target=/etc/localtime \
           --name raspicat-sim \
           raspicat-sim:humble
```

* GPUなし
```
docker run --rm -it \
           -u $(id -u):$(id -g) \
           --privileged \
           --net=host \
           --ipc=host \
           --env="DISPLAY=$DISPLAY" \
           --mount type=bind,source=/home/$USER/.ssh,target=/home/$USER/.ssh \
           --mount type=bind,source=/home/$USER/.gitconfig,target=/home/$USER/.gitconfig \
           --mount type=bind,source=/usr/share/zoneinfo/Asia/Tokyo,target=/etc/localtime \
           --name raspicat-sim \
           raspicat-sim:humble
```

* 備考
  * --rmを使用して、コンテナが終了した後に自動的にコンテナを削除することを指定
  * --gpu allを使用して、コンテナがホストシステムに接続されてGPUを利用可能
  * --runtime=nvidiaを使用して、Dockerのコンテナ内でNVIDIA GPUを使用するためのランタイムを設定
  * --mountを使用して、ローカルの.sshディレクトリをコンテナの起動時にマウント

## Raspberry Pi Catのシミュレータ

### 1. terminatorの起動
```
terminator
```
terminatorを起動すると新たなターミナルが起動します。

次の操作のためにターミナルを分割して3つにしてください。

* terminatorでよく使うコマンド
  * Ctrl+Shift+Eで縦に分割
  * Ctrl+Shift+Oで横に分割
  * Ctrl+Dで指定しているターミナルを削除

### 2. Raspberry Pi Catのシミュレータ
各コマンドをそれぞれのターミナルで実行してください。

```
ros2 launch raspicat_gazebo raspicat_with_iscas_museum.launch.py
ros2 service call /motor_power std_srvs/SetBool '{data: true}'
ros2 launch raspicat_navigation raspicat_nav2.launch.py
```
全てのコマンドを実行後、  
GazeboとRVizが立ち上がっていると思います。

RVizで初期位置を設定すると、ナビゲーションを開始できます。

## 注意
`docker commit`を実行する前に、  
sshディレクトリのアンマウントを行いましょう。（セキュリティー対策）

### .sshディレクトリのアンマウント
```
sudo umount /home/$USER/.ssh 
```
