<!DOCTYPE html>
<html lang="ja">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>図書館DX</title>
</head>
<body style="font-family: Arial, sans-serif; line-height: 1.6; background-color: #f9f9f9; padding: 20px;">

    <h1 style="text-align: center;">
        <span style="color: #FF0000;">🌈</span>
        <span style="color: #FF6347;">図</span><span style="color: #FF8C00;">書</span><span style="color: #FFD700;">館</span>
        <span style="color: #008000;">D</span><span style="color: #0000FF;">X</span>
        <span style="color: #4B0082;">🌈</span>
    </h1>

    <div style="text-align: center;">
        <img src="https://tyoudoii-illust.com/wp-content/uploads/2024/07/oksign_businessman_color-300x282.png" alt="Banner Image" style="max-width: 100%; height: auto;">
    </div>

    <hr>

    <h2 style="color: #FF9800;">🌟 <span style="color: #3F51B5;">Overview</span> 🌟</h2>
    <p>Welcome to the <span style="color: #2196F3;"><strong>図書館DX</strong></span>!</p>

    <hr>

    <h3 style="color: #9C27B0;">📁 <span style="color: #009688;">セットアップ手順</span></h3>

    <ol style="font-size: 16px;">
        <li>
            <span style="color: #8BC34A;">リポジトリのクローン:</span><br>
            以下のコマンドを実行して、リポジトリをクローンしてください：
            <pre style="background-color: #f0f0f0; padding: 10px; border-radius: 5px;">
cd src
git clone https://github.com/DAIGO0430/karikou-ITkougakuka-DX.git
            </pre>
        </li>
        <li>
            <span style="color: #8BC34A;">ビルドの実行:</span><br>
            次に、ビルドを行います：
            <pre style="background-color: #f0f0f0; padding: 10px; border-radius: 5px;">
colcon build
            </pre>
        </li>
        <li>
            <span style="color: #8BC34A;">セットアップの完了:</span><br>
            最後に、セットアップを完了するために以下のコマンドを実行します：
            <pre style="background-color: #f0f0f0; padding: 10px; border-radius: 5px;">
cd ..
source /install/setup.bash
            </pre>
        </li>
    </ol>

    <hr>

    <h3 style="color: #FF5722;">📑 <span style="color: #00BCD4;">ファイルについて</span></h3>

    <h4 style="color: #4CAF50;">1. <strong>object_detection_node.py</strong></h4>
    <p>
        - <span style="color: #9C27B0;">場所</span>: <code>ワークスペース/src/object_package/object_package/object_detection_node.py</code><br>
        こちらのファイルを参照するために、<code>src</code>ディレクトリで<code>git clone</code>を実行してください。
    </p>

    <h4 style="color: #4CAF50;">2. <strong>pic.py</strong></h4>
    <p>
        - <span style="color: #9C27B0;">用途</span>: <code>pic.py</code>は、<code>object_detection_node.py</code>の動作確認用のプログラムです。<br>
        このプログラムは、接続されたカメラ映像をトピックとして配信する機能を持っています。
    </p>

    <hr>

    <h3 style="color: #FF5722;">🔧 <span style="color: #2196F3;">必要な環境</span></h3>
    <ul>
        <li><span style="color: #4CAF50;">ROS 2 (適切なバージョン)</span></li>
        <li><span style="color: #4CAF50;">Python 3.x</span></li>
        <li><span style="color: #4CAF50;">colcon (ビルドツール)</span></li>
    </ul>

    <hr>

    <h3 style="color: #9C27B0;">📚 <span style="color: #FF9800;">ライセンス</span></h3>
    <p>このプロジェクトは、<a href="LICENSE" style="color: #2196F3;">MIT License</a>の下で公開されています。</p>

</body>
</html>
