# neo-mv

[English](README.md) | [简体中文](README.zh-CN.md) | **日本語**

neo-mv は VapourSynth と AviSynth 用の動き処理プラグインです。ブロック動き推定、動き補償、時間方向の混合、動きマスク、フレーム補間、グローバル動き推定と手ぶれ補正を提供します。API は MVUtensils を基にしています。

実装は C++17 を使用し、スカラー計算、Google Highway によるクロスプラットフォーム SIMD、周波数領域での動き推定に使う PocketFFT を備えています。DualSynth2 が計算コアを両ホストに接続します。VapourSynth では `core.neomv`、AviSynth では `neo_mv_` 接頭辞の関数を使用します。

## 設計

neo-mv は動きの計算とホストのフレーム管理を分離しています。コアは画像プレーン、動き場、明示的な幾何情報を扱い、ホスト層はクリップ、フレーム要求、プロパティ、出力の割り当てを管理します。コアは単独でビルドおよびテストできます。

実装は動作仕様に基づいて開発され、探索順序、丸め、境界、動き情報を利用できない場合の処理を明確に定めています。スカラー実装を計算の基準とし、SIMD 経路をそれぞれの数値規則に従って検証します。同じ関数名でも、過去のすべての MVTools や MVUtensils ビルドと同一の出力を保証するものではありません。

## 対応する処理

| 分類 | 関数 |
|---|---|
| 作業用画像 | `Super`：画像ピラミッド、境界拡張、サブピクセル位相。 |
| ブロック動き | `Analyse`、`AnalyseMany`、`Recalculate`、`SCDetection`：動き探索、複数の時間距離での解析、ベクトルの再推定、シーンフラグ。 |
| ブロック描画 | `Compensate`、`Degrain`、`Degrain1`–`Degrain25`：動き補償サンプリングと時間方向の重み付き合成。 |
| 動きマスク | `VectorLengthMask`、`SADMask`、`OcclusionMask`：ベクトル長、ブロック誤差、オクルージョンのマップ。 |
| 画素単位の動き | `Flow`、`FlowInter`、`FlowFPS`、`FlowBlur`：密な変位場、補間、フレームレート変換、軌道上の平均。 |
| グローバル動き | `DepanAnalyse`、`DepanEstimate`、`DepanCompensate`、`DepanStabilise`：動きの当てはめ、FFT 相関、幾何補償、手ぶれ補正。 |
| 診断 | `KernelInfo`：選択された計算バックエンドと FFT バックエンド。 |

ブロック動きと Flow 系の処理は、プレーナー GRAY/YUV の 8–16 ビット整数および 32 ビット浮動小数点サンプルに対応します。形式とサブサンプリングの制約は関数によって異なります。`DepanAnalyse`、`DepanCompensate`、`DepanStabilise` は整数画像を使用し、`DepanEstimate` は float32 にも対応します。RGB には対応していません。

動きデータはフレームプロパティに格納されます。既定のプロパティ接頭辞は `MVUtensils` で、プラグインの名前空間 `neomv` とは独立しています。Super の補助画像は生成した実装に属するため、neo-mv の関数で使用する Super は neo-mv で生成してください。

## ドキュメントと使用方法

計算ナレッジベースでは、各関数が入力から出力を求める過程を解説します。データ表現、数式、演算順序、パラメーターの役割、数値例、境界、精度を扱います。英語版を参照してください。

- [English knowledge base](docs/knowledge/en/README.md)

ビルドしたプラグインを明示的に読み込むか、VapourSynth のプラグイン自動読み込みディレクトリに配置してください。以下は Windows のファイル名を使用しています。他のプラットフォームでは実際のプラグインパスに置き換えてください。

```python
import vapoursynth as vs

core = vs.core
core.std.LoadPlugin(path="/path/to/neo-mv.dll")

print(core.neomv.KernelInfo())
clip = core.std.BlankClip(width=640, height=360, format=vs.YUV420P8, length=24)
super_clip = core.neomv.Super(clip, blksize=16, overlap=8, pad=32, pel=2)
vectors = core.neomv.Analyse(super_clip, delta=1)
output = core.neomv.Compensate(clip, super_clip, vectors)
output.set_output()
```

この最小例は合成クリップで呼び出しの流れを示します。`Super` ではブロックサイズとオーバーラップを明示する必要があります。正の `delta` は後のフレーム、負の `delta` は前のフレームを参照します。パラメーターと計算の詳細は各関数の記事を参照してください。

同じプラグインファイルに AviSynth C インターフェースも含まれます。AviSynth+ 3.7.4 以降（インターフェース 11）、または互換性のある AviSynthMinus ランタイムで `LoadCPlugin` を使用してください。

```avs
LoadCPlugin("/path/to/neo-mv.dll")
clip = BlankClip(width=640, height=360, length=24, pixel_type="YV12")
super_clip = neo_mv_Super(clip, blksize=16, overlap=8, pad=32, pel=2)
vectors = neo_mv_AnalyseMany(super_clip, radius=1)
return neo_mv_Degrain1(clip, super_clip, vectors)
```

関数名とパラメーター順はナレッジベースに対応し、関数名には `neo_mv_` を付けます。配列パラメーターは `[16, 8]` などのネイティブ配列を受け取り、単一の値も 1 要素の配列として扱います。`AnalyseMany` と `Recalculate` はクリップ配列を返し、Recalculate の入力が 1 つでも 1 要素の配列になります。`neo_mv_KernelInfo()` は `[backend, target, fft, fft_lanes]` を返します。真偽値には `true`/`false` を使用します。音声とフィールドパリティは最初の入力クリップから引き継ぎ、フィールド計算は `_Field` プロパティまたは明示した `tff` を使用します。Depan の `info=true` は AviSynth の `propShow` で診断プロパティを描画します。

## SIMD と CPU 選択

SIMD を有効にしたビルドは、実行中の CPU が対応し、かつビルドに含まれる Highway ターゲットを選択します。スカラーへのフォールバックも利用できます。最初のバックエンド初期化より前に `NEO_MV_KERNEL=scalar` を設定すると、スカラーカーネルとスカラー FFT を使用します。`NEO_MV_KERNEL=highway` は SIMD 有効ビルドを明示的に要求します。未設定の場合はビルドの既定バックエンドを選択します。

最初に成功した初期化の結果はキャッシュされます。その後に環境変数を変更しても、既存または新規のフィルターのバックエンドは切り替わりません。`core.neomv.KernelInfo()` は `backend`、`target`、`fft`、`fft_lanes` を返します。FFT ターゲットは一般カーネルのターゲットと異なる場合があります。レーン数はスレッド数や高速化率ではありません。

FFT の設定と許容される浮動小数点差分は結果に影響することがあります。広い SIMD が常に高いスループットを保証するわけではありません。[KernelInfo](docs/knowledge/en/kernel-info.md) と各関数の精度の節を参照してください。

## ビルドとテスト

CMake 3.24 以降、Git、C++17 対応コンパイラーが必要です。CMake は固定バージョンの DualSynth2 と PocketFFT を取得し、SIMD 有効時には Highway 1.4.0 も取得します。両ホストの SDK はローカルで検出するか、自動取得します。VapourSynth テストにはアーキテクチャが一致するランタイムと `vspipe`、AviSynth テストにはアーキテクチャが一致するランタイムライブラリが必要です。

```sh
cmake -S . -B build/release -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build/release --config Release --parallel 4
ctest --test-dir build/release -C Release --output-on-failure
```

| オプション | 用途 |
|---|---|
| `NEO_MV_BUILD_VAPOURSYNTH=OFF` | VapourSynth のエントリーポイントを無効化。 |
| `NEO_MV_BUILD_AVISYNTH=OFF` | AviSynth のエントリーポイントを無効化。両ホストを OFF にするとコアのみをビルド。 |
| `NEO_MV_ENABLE_SIMD=OFF` | SIMD カーネルとベクトル化 FFT を無効化。 |
| `BUILD_TESTING=OFF` | テストをビルドしない。 |
| `NEO_MV_TEST_VAPOURSYNTH=OFF` | コアテストとプラグインを残し、ホストテストを省略。 |
| `NEO_MV_VSPIPE_EXECUTABLE=/path/to/vspipe` | ホストテスト用ランタイムを指定。 |
| `NEO_MV_VS_SDK=/path/to/sdk` | ローカルの VapourSynth SDK を指定。 |
| `NEO_MV_AVS_SDK=/path/to/sdk` | ローカルの AviSynth SDK を指定。 |
| `NEO_MV_TEST_AVISYNTH=ON` | AviSynth ホストテストを有効化。既定では無効。 |
| `NEO_MV_AVISYNTH_RUNTIME=/path/to/avisynth.dll` | AviSynth ホストテスト用のランタイムライブラリを指定。 |
| `FETCHCONTENT_SOURCE_DIR_DUALSYNTH2=/path/to/dualsynth2` | 固定バージョンの取得に代えてローカルの DualSynth2 ソースを使用。 |
| `NEO_MV_BUILD_BENCHMARKS=ON` | 手動実行するカーネルベンチマークをビルド。SIMD が必要。 |

プラグインターゲット名は引き続き `neo_mv_vs`、出力ファイルの基本名は `neo-mv` です。既定で両ホストのエントリーポイントを含みます。テストは算術、動き探索、画像サンプリング、プロパティ検証、スカラー/SIMD 比較、境界、ホスト動作を扱います。MVUtensils と公開動作を比較するブラックボックステストもあり、参照実装の別途インストールが必要です。

CI は Windows x64、Linux x64、macOS ARM64、Linux ASan/UBSan の検査を設定しています。リリースワークフローは Windows、Linux、macOS の x64/ARM64 をビルドし、VapourSynth のホストテストは現在 Windows x64 で実行します。AviSynth のランタイムテストは上記のオプションで別途有効にします。

## 性能

既存の計測では、計測対象のフィルター経路で **MVUtensils R9 の約 0.62–1.83 倍のスループット**を得ています。比率は **neo-mv のスループット / MVUtensils のスループット**、すなわち MVUtensils の時間 / neo-mv の時間です。**1 より大きい場合は neo-mv が高速**です。以下の範囲は 8/16 ビットと AVX2/AVX-512 の各設定をまとめたもので、信頼区間ではありません。

| 関数 | 相対スループット |
|---|---:|
| Super | 1.11–1.44× |
| Analyse | 0.71–0.81× |
| Recalculate | 0.75–0.84× |
| Compensate | 0.80–1.03× |
| Degrain1 | 0.88–1.19× |
| Degrain2 | 0.94–1.38× |
| VectorLengthMask | 1.17–1.83× |
| SADMask | 0.84–1.56× |
| OcclusionMask | 0.81–1.08× |
| Flow | 1.26–1.59× |
| FlowInter | 1.07–1.58× |
| FlowFPS | 1.03–1.67× |
| FlowBlur | 1.30–1.35× |
| DepanAnalyse | 0.73–0.93× |
| DepanCompensate （バイリニア） | 0.79–0.94× |
| DepanCompensate （バイキュービック） | 1.15–1.49× |
| DepanEstimate | 0.62–0.81× |
| DepanStabilise | 1.06–1.42× |

この範囲は、VapourSynth 上で固定した 1080p YUV420P8/P16 入力で、個々のフィルターを単一スレッドで計測した結果です。上流の画像を事前生成しているため、処理チェーン全体のスループットを示すものではありません。結果は入力、パラメーター、ハードウェア、スレッド数によって変わります。

## 開発と貢献

メンテナーが技術方針、変更のレビュー、リリースを担当します。不具合報告、提案、貢献を歓迎します。数値動作、公開インターフェース、重要な設計変更については、実装前に目的と方針を相談してください。

本プロジェクトでは実装、テスト、レビューに AI を活用します。貢献には問題、方法、検証内容、AI の関与を記載してください。報告にはバージョン、OS、CPU、コンパイラー、ビルド設定、入出力形式、最小再現例を含め、性能報告には画像サイズ、バックエンド選択、計測方法も記載してください。

## 謝辞とライセンス

動き処理に取り組んできた MVTools、MVUtensils の作者と貢献者、そしてテスト、報告、改善に協力する開発者とユーザーの皆様に感謝します。neo-mv はホスト接続に DualSynth2、SIMD に Google Highway、FFT 計算に PocketFFT を使用します。

開発に使用する LLM サブスクリプションをご支援いただいた [SB.SB](https://sb.sb) に感謝します。

neo-mv は GNU General Public License バージョン 2 以降（`GPL-2.0-or-later`）で提供します。全文は [LICENSE](LICENSE) を参照してください。第三者コンポーネントはそれぞれの著作権表示とライセンス条項を維持します。
