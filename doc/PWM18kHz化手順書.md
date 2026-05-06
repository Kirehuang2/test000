# PWM 18kHz 化 手順書

作成日: 2026-04-22
対象コミット: `e2735fc` (doc訂正済)

## 1. 目的

現行PWM 10kHz → 18kHz に変更し、以下を達成する：
- 可聴域外（10kHz可聴域ど真ん中 → 18kHzは閾値付近）で**騒音低減**
- 高速域（≥2000RPM）の電流波形品質改善（サンプル数/電気周期を1.8倍に）
- 将来のFOC帯域拡大余地の確保

## 2. 前提・段階計画

本変更は**Step 3（最終段階）**。順序厳守：

| Step | 内容 | 独立性 | 現状 |
|------|------|--------|------|
| Step 1 | doc訂正（20kHz→10kHz） | 独立 | **完了** (e2735fc) |
| Step 2 | DTC補正値修正 (0.02→0.01) + コードコメント更新 | 独立 | 未着手 |
| Step 3 | PWM 10kHz→18kHz + 関連パラメータ再スケーリング | Step 2完了必須 | 本書 |

Step 2 を先に完了させ、10kHz で DTC=0.01 の動作を確認しておくこと。これにより 18kHz 化後の不具合が「PWM変更由来」か「DTC由来」かの切り分けが可能になる。

## 3. 事前確認（18kHz化前に必須）

### 3.1 FOC計算時間の実測

**目的**: 18kHz (周期55.56μs) に FOC 1サイクル分の計算が収まるか確認。
現行は10kHz (100μs) で余裕あり、angle_advance_us=200μs=2周期相当の計算遅延。

**方法A（推奨、侵襲低）**: GPIO トグル
1. [hal_entry.c:1330付近](src/hal_entry.c) の `adc_callback` 先頭に `R_IOPORT_PinWrite(DEBUG_PIN, HIGH)`
2. 同関数の末尾に `R_IOPORT_PinWrite(DEBUG_PIN, LOW)`
3. E1 + M2000 状態でスコープ観測、HIGH期間を測定

**方法B（コードのみ）**: サイクルカウンタ
- ISR内で DWT cycle counter 読み、最大値を変数に保存 → Expressions で確認

**判定**:
| 計算時間 | 判定 | 対応 |
|---------|------|------|
| < 30μs | 余裕あり | そのまま進める |
| 30~45μs | ギリギリOK | 進めてよいが ISR最適化を後続で検討 |
| 45~55μs | 危険 | FOC最適化必須（浮動小数→固定小数、sincosテーブル化等） |
| > 55μs | 不可 | 18kHz化断念、コード最適化優先 |

### 3.2 DRV8302 / FET 熱余裕

スイッチング頻度1.8倍 → スイッチング損失1.8倍。無負荷で長時間回して温度チェック：
- `Tinv_C` (V3) と `Treg_C` (V4) を BLE モニタで観察
- 10kHz時との差分が +10°C 以内なら許容
- DRV8302 のゲート駆動電流が足りているかデータシートで確認（60ns GTDVU/DVD の立上り/立下り要件）

### 3.3 Step 2 (DTC=0.01) 動作確認済み

Step 2 で `dtc_comp=0.01` に修正して M500/M2000/M3000 で |I| が悪化していないこと。

---

## 4. 変更箇所一覧（Step 3 全量）

### 4.1 FSP 設定（configuration.xml）

| ファイル | 現在値 | 新値 | 備考 |
|---------|--------|------|------|
| `configuration.xml:259` | `period = "10"` | `period = "18"` | 単位 kHz（line 260 unit で指定済み） |

**注意**: `hal_data.c` (`period_counts = 0x1770 = 6000`) は FSP 自動生成ファイル。直接編集不可、configuration.xml から再生成する。18kHz設定後、`period_counts = 3333` (≈120MHz/(18kHz×2)) になる。

### 4.2 C ソースコード

| ファイル:行 | 変数/値 | 現在値 | 新値 | スケール根拠 |
|------------|--------|--------|------|-------------|
| [hal_entry.c:1469](src/hal_entry.c#L1469) | Id積分器の Ts ハードコード | `0.0001f` | `5.556e-5f` | Ts = 1/18000 |
| [hal_entry.c:1474](src/hal_entry.c#L1474) | Iq積分器の Ts ハードコード | `0.0001f` | `5.556e-5f` | 同上 |
| [hal_entry.c:160](src/hal_entry.c#L160) | `OC_GRACE_CYCLES` | `5000` (500ms) | `9000` (500ms) | 500ms ÷ Ts |
| [hal_entry.c:161](src/hal_entry.c#L161) | `OC_SUSTAIN_CYCLES` | `50` (5ms) | `90` (5ms) | 同上 |
| [hal_entry.c:44](src/hal_entry.c#L44) | `SPEED_EST_WINDOW` | `200` (20ms) | `360` (20ms) | 同上 |
| [hal_entry.c:211](src/hal_entry.c#L211) | `decouple_lpf_alpha` デフォルト | `0.006f` (~10Hz) | `0.00333f` (~10Hz) | α × fs/(2π) が一定 |
| [hal_entry.c:309](src/hal_entry.c#L309) | `speed_ramp_rate` デフォルト | `0.00004f` | `0.00002222f` | PU/call × call_rate が一定 |
| [hal_entry.c:218](src/hal_entry.c#L218) | DTC計算式コメント | `2×500ns/50us=0.02` | `2×500ns/55.56us=0.018` | T_pwm更新 |
| [hal_entry.c:218付近](src/hal_entry.c#L218) | `dtc_comp` デフォルト | `0.01` (Step 2後) | `0.018` | 実T_pwm/実t_dead |
| [ConfigParameters.c:44](src/ConfigParameters.c#L44) | `T_pwm` | `5.0E-5` (旧docの誤値) | `5.556E-5` | 18kHz周期 |

### 4.3 コメント・ドキュメント

| ファイル | 修正内容 |
|---------|---------|
| [hal_entry.c](src/hal_entry.c) | "10kHz" 出現箇所全て "18kHz" に置換（コメントのみ、検索: `10kHz\|10 kHz\|100us\|100μs\|0.0001`） |
| [doc/ソフト管理表.md](doc/ソフト管理表.md) | 周波数記載を全て 18kHz/55.56μs へ、パラメータ一覧新値反映 |
| [doc/モーター設計書.md](doc/モーター設計書.md) | 周波数・DTC計算式を18kHz準拠に |
| [doc/ソフトウェア設計書.md](doc/ソフトウェア設計書.md) | タイミングパラメータ表更新 |
| [doc/既存実装分析.md](doc/既存実装分析.md) | 周波数記載更新 |
| [doc/自作モータードライバ基板_仕様書.md](doc/自作モータードライバ基板_仕様書.md) | 同上 |
| [doc/設計書_1軸サーボバランサ制御システム.md](doc/設計書_1軸サーボバランサ制御システム.md) | `inverter.f_sw` を 18kHz に |
| [doc/基板管理表.md](doc/基板管理表.md) | 影響なし |
| [doc/使い方ガイド.md](doc/使い方ガイド.md) | 影響なし（周波数記載なし） |

### 4.4 angle_advance_us の再最適化（テスト時）

**現状**: 10kHz で `angle_advance_us=200` (2サンプル遅延相当) が最適
**18kHz後**: 物理計算遅延は変わらない可能性が高いが、FOC周期変化で実効遅延は変わる。**再スイープ必須**。
- M2000 で 0/50/100/150/200/250 μs をスイープ、|I| が最小の値を採用

---

## 5. 実施手順

### Step 3-A: コード変更とビルド

1. **ブランチ作成（推奨）**
   ```bash
   git checkout -b pwm-18khz
   ```

2. **e2 studio で FSP Configurator を開く**
   - プロジェクト `000` → `configuration.xml` をダブルクリック
   - `Stacks` タブ → `g_three_phase0 Three-Phase PWM` を選択
   - Properties → `Period` を **18** に変更（単位 kHz のまま）
   - **Generate Project Content** ボタンをクリック
   - `hal_data.c` が再生成される

3. **ソースコード変更（4.2節の全項目）**
   - Ts ハードコード 0.0001f → 5.556e-5f（2箇所）
   - OC/SPEED_EST 時定数（3箇所）
   - LPF/ramp デフォルト（2箇所）
   - DTC コメントと値（Step 2 の0.01 → 0.018）
   - ConfigParameters.c の T_pwm

4. **ビルド**
   - Debug Configuration でビルド、Warning/Error ないこと確認
   - Map ファイルで stack overflow などに注意

5. **ドキュメント更新（4.3節）**
   - ソフト管理表.md に「セッション6：18kHz化」ログ追加
   - パラメータ一覧表の値を新値に更新

6. **コミット前テスト開始前**: ここでまず `git commit` しない。テスト後まとめてコミット。

### Step 3-B: 実機テスト（#2 or #3 基板）

**注意**: 先に基板#2のモーター回転問題を解決しておくこと。回転しない状態で新コードを書き込むと切り分け困難。

1. **無負荷・低速テスト**
   ```
   E0
   # 書き込み・デバッグ開始
   C1 L60 T30 E1
   ```
   - モーター回るか？
   - `D` で enable_off_src, oc_fault, i2t_ratio 正常確認
   - 騒音の変化を聴覚確認（10kHz→18kHzの聴感上の差）
   - 回らなければ **即 E0、ロールバック**

2. **中速テスト（M500, M1000）**
   - 各速度で `LOG START 100` → 20秒記録 → `S`
   - CSV解析で RPM std、|I|、IqRef 確認
   - 10kHz時との比較：RPM精度、電流リップル

3. **高速テスト（M2000, M3000）**
   - **angle_advance_us スイープ**: `SET angle_advance_us N` で 0~250 を50刻み
   - M2000 各設定で20秒ログ → |I| 最小の値を採用
   - その値をコードデフォルトに焼き込み

4. **DTC 再調整（必要なら）**
   - 理論値 `dtc_comp=0.018` で開始
   - M2000/M3000 で |I| が改善しない場合、0.015~0.025 でスイープ

5. **保護機能テスト**
   - 手動ストール → 実電流リミッター発動確認（|I|<2×I_rated維持）
   - i2t 閾値未変更のため、トリップ時間は同じ（秒単位は実時間ベース）

### Step 3-C: 温度・負荷試験

1. **長時間無負荷**: M1000 で10分回してインバーター温度記録
2. **負荷試験**: Phase 3-1再開（1kg→5kg→10kg→13kg）

### Step 3-D: コミット

テスト全てOKなら：
```bash
git add src/hal_entry.c src/ConfigParameters.c configuration.xml ra_gen/hal_data.c doc/
git commit -m "Change PWM from 10kHz to 18kHz for noise reduction"
```

ラベル（コミットメッセージ）には以下を含める：
- 変更理由（騒音低減、可聴域外化）
- 再スケーリングしたパラメータ一覧
- angle_advance_us の新最適値
- テスト結果サマリ（M500/M1000/M2000/M3000 の |I|, RPM std）

---

## 6. ロールバック手順

### A. 物理的ロールバック（コミット前）

```bash
git checkout configuration.xml src/ ra_gen/hal_data.c
# FSP Configurator を再度開いて Generate Project Content（念のため）
```

### B. コミット後ロールバック

```bash
git revert <commit_hash>
```
または `git checkout` で安全な戻り先：
- `e2735fc` — doc訂正のみ、10kHz動作状態
- `32b815d` — 電流リミッター修正、#1基板で負荷試験5.5kgOK

### C. ホットフィックス（実機動作中に問題発覚）

多くのパラメータは BLE `SET` で可変：
- `SET dtc_comp 0.01` → DTC無効化相当
- `SET angle_advance_us 200` → 旧値で試す
- `SET decouple_lpf_alpha 0.006` → 旧値

`integrator_limit`, `Kp_iq`, `Ki_iq` も BLEで可変なので、暫定対応可能。

---

## 7. リスクと対策

| リスク | 発生確率 | 影響度 | 対策 |
|--------|---------|-------|------|
| FOC計算が55μs超過 → 制御破綻 | 中 | 大 | §3.1 で事前計測。超過なら18kHz断念 |
| 積分器Tsスケール漏れ → 不安定 | 中 | 大 | 本書§4.2の全箇所をチェックリスト化 |
| FET 熱上昇 | 低 | 中 | §3.2 で事前温度確認、本番でも継続監視 |
| angle_advance 再最適化失敗 | 低 | 中 | スイープ必須、10kHz時の200μsは非最適 |
| FSP再生成で未知のファイル変更 | 低 | 中 | `git diff` で hal_data.c 以外の変更を精査 |
| DTC 0.018 が過補償/不足 | 中 | 小 | BLEで0.01~0.025スイープ、最適値を選定 |
| bootstrap cap 不足で高側FETが飽和 | 低 | 大 | DRV8302データシートで確認、異音/電流異常を即検知 |

---

## 8. チェックリスト（実施時に印刷して使用）

### 事前
- [ ] Step 2 (DTC=0.01) が完了し、10kHz動作が安定
- [ ] 基板 #2 のモーター回転問題が解決済、または健全な #3/#4 基板使用
- [ ] FOC計算時間 < 45μs を実測確認
- [ ] 無負荷長時間運転で温度正常
- [ ] `git status` クリーン、ブランチ作成済

### 実施
- [ ] configuration.xml: `three_phase.period = "18"` に変更
- [ ] FSP Generate Project Content 実行
- [ ] hal_data.c の `period_counts` が 3333 付近になったことを確認
- [ ] hal_entry.c:1469,1474 の Ts (0.0001f → 5.556e-5f)
- [ ] hal_entry.c:160,161 の OC_GRACE/SUSTAIN
- [ ] hal_entry.c:44 の SPEED_EST_WINDOW
- [ ] hal_entry.c:211 の decouple_lpf_alpha デフォルト
- [ ] hal_entry.c:309 の speed_ramp_rate デフォルト
- [ ] hal_entry.c:218付近 の dtc_comp デフォルト (0.018)
- [ ] hal_entry.c コメント中の "10kHz" → "18kHz" 全置換
- [ ] ConfigParameters.c の T_pwm
- [ ] ビルド成功 (Warning 0、Error 0)

### テスト
- [ ] E1 + T30 でモーター回転
- [ ] 騒音が聴感上低下
- [ ] M500/M1000 無負荷 |I|<0.3A
- [ ] M2000 angle_advance スイープ → 最適値採用
- [ ] M3000 無負荷 |I|<0.4A
- [ ] ストール検出・実電流リミッター動作
- [ ] 長時間運転（10分）で温度異常なし

### 事後
- [ ] doc/ソフト管理表.md 更新（パラメータ表、セッション6ログ）
- [ ] doc/モーター設計書.md の周波数・DTC式更新
- [ ] `git commit` 実施、message に結果サマリ
- [ ] コミットハッシュを「安全な戻り先」に追加

---

## 9. 参考情報

### 周波数・Ts 換算表
| 項目 | 10kHz | 18kHz | 20kHz |
|------|-------|-------|-------|
| T_pwm | 100 μs | 55.56 μs | 50 μs |
| Ts (FOC) | 100 μs | 55.56 μs | 50 μs |
| period_counts (120MHz) | 6000 | 3333 | 3000 |
| DTC (500ns dead time) | 0.01 | 0.018 | 0.02 |
| スケール係数 (10kHz比) | 1.0 | 0.5556 | 0.5 |

### 可聴性
- 10kHz: ほぼ全年齢で聞こえる、騒音感強
- 18kHz: 30代以降で聞こえにくくなる、若年層や女性は聞こえることあり
- 20kHz: 成人の大半は聞こえない、但しMCU/FET余裕ギリギリ

18kHz は「聴こえにくさ」と「ハードウェア余裕」のバランス点。
