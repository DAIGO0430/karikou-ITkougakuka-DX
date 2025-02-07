# 🌈 **図書館DX** 🌈

![Banner Image](https://tyoudoii-illust.com/wp-content/uploads/2024/07/oksign_businessman_color-300x282.png)　　　　　　　　　　　　　　　　![Banner Image](https://tyoudoii-illust.com/wp-content/uploads/2024/07/oksign_businessman_color-300x282.png)
# 図書館DX


**図書館DX** プロジェクトでは、物体検出を用いて蔵書管理を効率化するためのシステムを構築しています。現時点では、物体検出により本の位置を特定することは可能ですが、本の題名や著者名の認識にはまだ成功していません。その実現には、検出された本の位置情報をもとに画像処理を行い、さらにGoogle Booksのデータベースを利用して本を自動的に識別するシステムを作成する必要があります。

---

### 現在の課題と次のステップ

1. **物体検出の改善**: 現在、物体検出で本を認識できるようになっています。しかし、次のステップとして、物体検出の結果から本の位置を特定し、その部分の画像を切り取る必要があります。

2. **文字認識**: 切り取った画像に対して文字認識（OCR）を行い、題名や著者名を抽出します。OpenCVやTesseractを使ってこの処理を行うことができます。

3. **Google Booksデータベースとの連携**: 取得した題名や著者名を基に、Google Books APIを使って本の情報を取得し、蔵書の判別ができるようにします。

4. **本のタイトル表示**: 物体検出と文字認識を連携させ、最終的に物体検出結果の画像上に本の題名を表示できるようにすることが目標です。

---

### 物体検出結果から本のタイトルを表示するイメージ

イメージとしては、物体検出で本の位置が特定された後、その位置に基づいて画像を切り取り、タイトルを抽出して検出画像に表示する形です。以下はその例となります：

---

## **実装方法**

1. **物体検出後の座標取得**  
   `object_detection_node.py` で得られた座標をもとに、対象となる本を切り取るコードを実装します。

2. **画像切り取りとOCR**  
   OpenCVを使用して、物体検出で得た座標情報を基に画像を切り取り、TesseractなどのOCRツールを使ってタイトルや著者名を抽出します。

3. **データベース連携**  
   抽出した文字データを使って、Google Books APIを利用して本の詳細情報を取得します。

4. **結果の表示**  
   最終的に、本の画像上に題名を表示できるようにします。

---

これらのステップを実装することで、物体検出に加えて本の題名や著者名を正確に認識し、蔵書管理がよりスマートに行えるようになります。

---

## 📂 **object_packageの場所**

- **ワークスペース/src/object_package/object_package/object_detection_node.py**  
  上記のように、`src` ディレクトリで `git clone` を行ってください。

## 🖼 **pic.pyについて**

- `pic.py` は `object_detection_node.py` の確認用のプログラムです。  
  `pic.py` は接続されたカメラ映像をトピックとして配信するプログラムです。

##dokcerfileについて
下のサイトで配布されているdokcerfileは情報が欠損している箇所があるのでこのgithubのものを使用してください。
dokcerfileのビルドはsrcに配置してから行ってください
ビルドコマンド
```bash

```
daemon.jsonについて
以下のサイトを参考にしてください
ーhttps://docs.docker.jp/config/daemon/daemon.html
ーhttps://docs.docker.com/engine/daemon/

##参考サイト
https://note.com/m_shirae/n/n7e38cb8c10b7　←大本のサイト仮想環境上で物体検出を行っている
https://qiita.com/Yarimizu14/items/52f4859027165a805630　←dockerのマウントの際に参考にしたサイト


---

## **セットアップ手順**

以下のコマンドを使用して、プロジェクトをセットアップします。

```bash
cd src
git clone https://github.com/DAIGO0430/karikou-ITkougakuka-DX.git
colcon build
cd ..
source /install/setup.bash
```
![Banner Image](https://tyoudoii-illust.com/wp-content/uploads/2024/07/oksign_businessman_color-300x282.png)![Banner Image](https://tyoudoii-illust.com/wp-content/uploads/2024/07/oksign_businessman_color-300x282.png)!
