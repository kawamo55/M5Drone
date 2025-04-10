# M5Drone
このソースは、M5Stamp FlyをM5StampS3でコントローラをシミュレーションするためのソースです。  
なので、M5Stamp Flyとは別のM5Stamp本体にこのプログラムを載せて、PCからコントローラーをシミュレートするM5Stampにシリアル通信でコマンドを送ってM5Stamp Flyをコントロールするものです。
まだまだ未完成ですがとりあえず飛びます。

# オリジナルプロジェクト
オリジナルプロジェクトは  
https://github.com/m5stack/Atom-JoyStick/  
このプロジェクトで、M5AtomS3を使ったコントローラー部分のソースを参考にしております。  

# コンパイル環境
コンパイル環境は、Arduino IEDでボードマネージャに下記URLを追加してM5StampS3を選択します。  
https://static-cdn.m5stack.com/resource/arduino/package_m5stack_index.json  

そのあと、StampCTLフォルダにあるStampCTL.inoをArduino IDEで開いてコンパイルおよび書き込みします。

# 使い方
シリアルポートからコマンドを送ることで、M5Stamp Flyに制御コードを送ります。  
prset ペアリングリセットコマンド  
ペアリングした機体と別の期待にペアリングする場合に、M5Stamp Fly本体の電源を入れた状態でこのコマンドを送るとペアリングを行います。  
  
takeoff テイクオフコマンド  
200mm程浮上します。  
浮上した後行動を保とうとしていますが、うまく動いていません。  
改行コードの身を送ると、とりあえずステータスを返すのでそのステータスを見ながら浮上させたり降下制御が必要だと考えられます。  
  
fend 着地コマンド  
強制着地します。  
  
SampleフォルダにPythonで書いたサンプルプログラムを置いてあります。  
