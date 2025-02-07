import cv2
import easyocr
import numpy as np
from PIL import Image, ImageDraw, ImageFont

# EasyOCRリーダーの初期化（日本語を含む）
reader = easyocr.Reader(['ja', 'en'])  # 日本語と英語を認識対象に設定

# 日本語フォントの指定（Pillow用）
font_path = "/usr/share/fonts/truetype/fonts-japanese-gothic.ttf"  # 日本語フォントのパス
font = ImageFont.truetype(font_path, 32)

# カメラを開く
cap = cv2.VideoCapture(0)  # 4番カメラを使用

if not cap.isOpened():
    print("カメラが開けませんでした")
    exit()

while True:
    ret, frame = cap.read()  # フレームをキャプチャ
    if not ret:
        print("フレームが取得できませんでした")
        break

    # OCRを実行
    results = reader.readtext(frame)

    # OCR結果があれば、フレームに描画
    for (bbox, text, prob) in results:
        # bboxの座標を取り出し、整数にキャスト
        (x1, y1), (x2, y2), (x3, y3), (x4, y4) = bbox
        x1, y1 = int(x1), int(y1)  # x1, y1を整数にキャスト
        x2, y2 = int(x2), int(y2)  # x2, y2を整数にキャスト
        x3, y3 = int(x3), int(y3)  # x3, y3を整数にキャスト
        x4, y4 = int(x4), int(y4)  # x4, y4を整数にキャスト

        # OCRのテキストをボックス内に描画
        cv2.polylines(frame, [np.array([bbox], dtype=np.int32)], isClosed=True, color=(0, 255, 0), thickness=2)

        # Pillowを使って日本語を描画するために画像をPillowに変換
        pil_image = Image.fromarray(frame)
        draw = ImageDraw.Draw(pil_image)

        # 日本語テキストを描画（フォント指定）
        draw.text((x1, y1 - 10), text, font=font, fill=(255, 0, 0))

        # OpenCV形式に戻して表示
        frame = np.array(pil_image)

    # カメラ映像を表示
    cv2.imshow('Camera Feed with EasyOCR', frame)

    # 'q'キーで終了
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# カメラの解放とウィンドウの終了処理
cap.release()
cv2.destroyAllWindows()

