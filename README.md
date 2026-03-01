# 糸を用いた指向性ボリュームディスプレイのための投影画像自動補正システム
本プロジェクトは，糸を用いたボリュームディスプレイにおいて，プロジェクタとディスプレイの3次元位置関係に基づき投影画像を自動補正するシステムです．投影位置調整作業の大幅な簡易化と糸配置の柔軟性向上を実現しました．

## 1. 概要（Overview）
**糸を用いた指向性ボリュームディスプレイ**（図1）は，観察方向に応じて異なる画像を表示できる特殊なディスプレイです．鉛直方向に張られた複数の糸に対し，プロジェクタで特殊な画像を投影することで指向性画像を表示します．高品質な表示には，各糸に対して対応する矩形画像を正確に投影する必要があります．

<div align="center">
  <img src="https://github.com/user-attachments/assets/2676090f-660b-4da5-916a-2122b8b2f7bc" width=75%/>
  <p><b>
    図1：糸を用いたボリュームディスプレイ<br>
    （左：投影の様子，中央：正面から見た画像"G"と表示，右：側面から見た画像"7"と表示）
  </b></p>
</div>

従来は，**投影位置の調整を手作業**で行う必要がありましたが，要求精度が高いうえ，多大な時間を要するという問題がありました．さらに，プロジェクタとディスプレイの位置関係によっては，各糸に投影される矩形画像同士が重なり，ノイズが発生します．そのため，**糸の配置が制限**され，表示に必要な本数の糸を全て配置できない場合がありました．

これらの問題を解決するために，**ArUcoマーカー**を用いた3次元姿勢推定によりプロジェクタとディスプレイの相対位置を自動計測し，それに基づいて**投影画像を自動補正するシステム**を開発しました（図2）．これにより，従来手動で行っていた投影位置調整を自動化し，ノイズが発生しやすい糸配置においても正確な画像投影を可能にしました．

<div align="center">
  <img src="https://github.com/user-attachments/assets/6b452d17-6d06-445b-bf6b-6e6a00585126" width="50%">
  <p><b>図2：投影画像自動補正システムの概要図</b></p>
</div>

## 2. 主な機能（Key Features）
+ **3次元位置関係の自動計測**：プロジェクタとディスプレイの上部に貼り付けたArUcoマーカーをカメラで撮影し，プロジェクタとディスプレイ間の3次元位置関係を推定
+ **3次元位置関係の微調整UI**（図3）：トラックバーを用いて、各糸に対応する矩形画像の位置・形状をリアルタイムに微調整し、マーカー計測の誤差を解消
+ **投影画像の自動補正**：微調整後のパラメータに基づき，従来手法で作成した投影画像を自動補正
<div align="center">
  <img src="https://github.com/user-attachments/assets/7a6d4e23-0727-41bf-8a32-d979efaf6b55" width=70% />
  <p><b>図3：微調整UIのイメージ画像</b></p>
</div>

## 3. ディレクトリ構造（Directory Structure）
```
.
├── src/                
│   ├── automatic_projected_image_correction.cpp  # ソースコード（マーカー検出・補正計算）
│   └── json.hpp            # カメラの内部パラメータ入力用ファイル (nlohmann/json)
├── config/             
│   └── inParams.json       # カメラの内部パラメータを保存したファイル
├── data/               
│   └── testImg.bmp         # 動作検証用のArUcoマーカー撮影画像
│   └── projImg_threads.bmp # 従来手法で作成した，補正前の投影画像
├── results/            
│   └── projImg_threads_tuned.bmp                # 微調整後の矩形画像の描画位置を示す画像
│   └── projImg_threads_directional_filtered.bmp # 補正後の投影画像
└── README.md               # 本ファイル
```

## 4. 技術スタック（Tech Stack）
+ Language: C++
+ Libraries:
  +  OpenCV 4.11.0
  +  nlohmann/json
+  Environment: Windows (Visual Studio 2022)

## 5. 実行結果（Experimental Results）
本システムを実行した結果，従来手法で作成した元の投影画像（図4）に対し，ディスプレイの糸配置に合わせた自動補正（図5）が可能になりました．
これらの投影画像は，正面からは「×」，側面からは「〇」が視認される指向性表示を目的としたものです．

<div align="center">
  <table>
    <tr>
      <td align="center">
        <img src="https://github.com/user-attachments/assets/86c4057d-4a24-4610-9e7d-67237f6c6307" width=90% /><br>
        <b>図4：元の投影画像（従来手法）</b>
      </td>
      <td align="center">
        <img src="https://github.com/user-attachments/assets/50b1406d-ca12-474e-8733-47f9360196c5" width=90% /><br>
        <b>図5：自動補正後の投影画像</b>
      </td>
    </tr>
  </table>
</div>

また，補正後の投影画像を実際にディスプレイへ投影した結果（図6），ノイズの発生しやすい糸配置においても高品質な指向性画像を表示することに成功しました．さらに，糸とプロジェクタの距離に応じて光線長を最適化したことで，側面観察時の投影画像拡散による歪みを大幅に軽減できました．

<div align="center">
  <table>
    <tr>
      <td align="center">
        <img src="https://github.com/user-attachments/assets/044319c7-0efc-4f5f-8793-8fe62cdc4d3e" width="300px"/><br>
        <b>正面：理想の表示画像</b>
      </td>
      <td align="center">
        <img src="https://github.com/user-attachments/assets/aaac6c1c-1c26-4216-8cfe-eea09b5b3b3b" width="300px"/><br>
        <b>正面：実際の観察結果</b>
      </td>
    </tr>
    <tr>
      <td align="center">
        <img src="https://github.com/user-attachments/assets/9439cbda-416b-4a19-9625-9a3e65051388" width="300px"/><br>
        <b>側面：理想の表示画像</b>
      </td>
      <td align="center">
        <img src="https://github.com/user-attachments/assets/d70ddf25-a7b9-4105-afa9-b476f15ee016" width="300px"/><br>
        <b>側面：実際の観察結果</b>
      </td>
    </tr>
  </table>
  <p><b>図6：投影画像自動補正システムによる表示結果の比較</b></p>
</div>

## 6. 今後の展望 (Future Work)
本システムのさらなる利便性と表示品質の向上に向け，以下の課題に取り組む予定です．

+ **微調整工程のさらなる自動化**：カメラフィードバックを用いた「自動最適化アルゴリズム」を導入予定です．投影された画像をカメラで解析し，矩形の位置や形状を動的に変更することで，人の手を介さず最適な投影位置を自動探索する仕組みを構築します
+ **3次元位置計測の高精度化**：プロジェクタとディスプレイに配置するマーカーの数を増やし，多点観測による誤差補正を行うことで，さらなる誤差軽減を実現します




