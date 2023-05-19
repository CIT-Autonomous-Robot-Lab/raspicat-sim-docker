# raspicat-ros2-humble-docker
ROS 2のhumbledeでRaspberry Pi Catのシミュレータ使うためのDockerfileです。

sshkeyの設定は[こちらのサイト](https://qiita.com/shizuma/items/2b2f873a0034839e47ce)を参照してください。

## dockerの環境構築
### 1. dockerのインストール
```
sudo apt install docker.io
```
### 2. dockerのコマンドをsudoなしで実行するための設定
dockerグループに所属しているユーザが確認できる。
```
getent group docker
```
設定前だと以下のように出力される。

例: `docker:x:138:`

以下のコマンドでユーザ名を確認できます。
```
whoami
```
例: `leon`

dockerグループにPCのユーザを追加します。

```
sudo gpasswd -a <user> docker
```
追加後に`getent group docker`を実行すると以下のように出力されます。

例: `docker:x:138:leon`

dockerグループに所属してから一度ログアウトしてから再ログインするとsudoなしでdockerコマンドを使用できるようになります。

## リポジトリのクローン
```
git clone git@github.com:CIT-Autonomous-Robot-Lab/raspicat-docker.git
```

## コンテナを起動

### 1. イメージの作成
```
cd raspicat-docker/ros2-humble-gpu 
docker build --build-arg DEFAULT_USER=$USER -t raspicat:humble -f Dockerfile .
```
GPUを搭載していない場合は`raspicat/ros2-humble`に移動して上記のbuildコマンドを入力してください。

・raspicat: 作成するdockerイメージの名前

・humble: 作成するdockerイメージのTAG名

(任意で変更可能)

使用するDockerfileはbuildの際に以下のアプリケーションをインストールします。

・vim

・git 

・terminator

・wget

### 2. 使用するイメージのIMAGE IDを取得
```
docker images raspicat:humble
```
指定したdockerイメージのIMAGE IDが出力されます。

例: `test2    colcon    77bf3192b4db   2 days ago   5.52GB`

左からREPOSITORY、TAG、IMAGE ID、CREATED、SIZEを表しています。

### 3. イメージでGUIを使用するために以下のコマンドを実行
```
xhost local:<IMAGE ID>
```
### 4. コンテナ起動
```
docker run --rm -it --privileged --gpus all --net=host --ipc=host --env="DISPLAY=$DISPLAY" --runtime=nvidia --mount type=bind,source=/home/$USER/.ssh,target=/home/$USER/.ssh raspicat:humble
```
・--rmを使用して、コンテナが終了した後に自動的にコンテナを削除することを指定します。

・--gpu allを使用して、コンテナがホストシステムに接続されてGPUを利用可能にしています。

・--runtime=nvidiaを使用して、Dockerのコンテナ内でNVIDIA GPUを使用するためのランタイムを設定しています。

（GPUを搭載していない場合は、--gpu allと--runtime=nvidiaオプションは付けずに実行してください。）

・--mountを使用して、ローカルの.sshディレクトリをコンテナの起動時にマウントするようにしています。

## Raspberry Pi Catのシミュレータ

### 1. terminatorの起動
```
terminator
```
terminatorを起動すると新たなターミナルが起動します。

次の操作のためにターミナルを分割して3つにしてください。

terminatorでよく使うコマンド

・Ctrl+Shift+Eで縦に分割

・Ctrl+Shift+Oで横に分割

・Ctrl+Dで指定しているターミナルを削除

### 2. 各ターミナルでsource
以下のコマンドを作成した３つターミナルすべてで行ってをください。
```
. install/setup.bash
```

### 3. Raspberry Pi Catのシミュレータ
１つ目のターミナルで以下のコマンドを実行してください。
```
ros2 launch raspicat_gazebo raspicat_with_iscas_museum.launch.py
```
２つ目のターミナルで以下のコマンドを実行してください。
```
ros2 service call /motor_power std_srvs/SetBool '{data: true}'
```
３つ目のターミナルで以下のコマンドを実行してください。
```
ros2 launch raspicat_navigation raspicat_nav2.launch.py
```
３つのコマンドを実行したら、GazeboとRvizが立ち上がっていると思います。

Rvizで初期位置を設定すればナビゲーションを開始できます。

## コンテナ内でgit clone

git clone を行う場合はコンテナ内でユーザをrootからローカルのユーザに変更してください。

コンテナ内で以下のコマンドを実行することでユーザをローカルのユーザに変更できます。
```
su <user>
```
変更後にユーザをrootに戻すときは以下のコマンドを実行してください。
```
exit
```
## .sshディレクトリのアンマウント
sshディレクトリのアンマウントはコンテナ内でユーザをrootに変更して行ってください。
```
umount /home/<user>/.ssh 
```

### 7. 実行中のコンテナを新しいイメージにコミット
以下のコマンドはローカルで実行してください。

起動中のコンテナのコンテナIDを取得
```
docker ps
```
コンテナを新しいイメージにコミット
```
docker container commit <container_id> <image_name>:<TAG> 
```
`<image_name>:<TAG>`は起動しているコンテナと同じにすればイメージを上書きできます。

新しいイメージとして保存したい場合は、任意のイメージ名とTAG名をつけてください。

### 8.コンテナから出る
コンテナ内で以下のコマンドを実行してください。（rootユーザで実行）
```
exit
```