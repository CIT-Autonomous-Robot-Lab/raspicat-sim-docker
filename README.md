# ros2-humble-docker
ROS 2のhumbleを使うためのDockerfileです。

sshkeyの設定は[こちらのサイト](https://qiita.com/shizuma/items/2b2f873a0034839e47ce)を参照してください。

## dockerのインストール
```
sudo apt install docker.io
```

## リポジトリのクローン
```
git clone git@github.com:CIT-Autonomous-Robot-Lab/raspicat-docker.git
```

## 実行

### 1. イメージの作成
```
cd raspicat-docker/ros2-humble-gpu 
docker build --build-arg DEFAULT_USER=<user> -t <image_name>:<TAG> -f Dockerfile .
```
GPUを搭載していない場合はraspicat/ros2-humbleに移動して上記のbuildコマンドを入力してください。

・user: ローカルのユーザ名

・image_name: 作成するdockerイメージの名前

・TAG: 作成するdockerイメージのTAG名

ローカルのユーザ名は以下のコマンドで確認してください。
```
whoami
```
### 2. 使用するイメージのIMAGE IDを取得
```
docker images <image_name>:<TAG>
```
### 3. イメージでGUIを使用すつために以下のコマンドを実行
```
xhost local:<IMAGE ID>
```
### 4. コンテナ起動
```
docker run --rm -it --privileged --gpus all --net=host --ipc=host --env="DISPLAY=$DISPLAY" --runtime=nvidia --mount type=bind,source=/home/<user>/.ssh,target=/home/<user>/.ssh <image_name>:<TAG>
```
・--gpu allを使用して、コンテナがホストシステムに接続されてGPUを利用可能にしています。

・--runtime=nvidiaを使用して、Dockerのコンテナ内でNVIDIA GPUを使用するためのランタイムを設定しています。

（GPUを搭載していない場合は、--gpu allと--runtime=nvidiaオプションは付けずに実行してください。）

・--mountを使用して、ローカルの.sshディレクトリをコンテナの起動時にマウントするようにしています。
### 5. コンテナ内でユーザを変更
```
su <user>
```
変更後にユーザをrootに戻すときは以下のコマンドを実行
```
exit
```

### 6. 実行中のコンテナを新しいイメージにコミット
以下のコマンドはローカルで実行してください。
起動中のコンテナのコンテナIDを取得
```
docker ps
```
コンテナを新しいイメージにコミット
```
docker container commit <container_id> <image_name>:<TAG> 
```
### 7. .sshディレクトリのアンマウント
sshディレクトリのアンマウントはユーザをrootに変更して行ってください
```
umount /home/<user>/.ssh 
```