# モーター制御プロジェクト - 冒険の書

## プロジェクト概要
- **プロジェクトコード**: pj:26-001
- **IDE**: e2 studio (Renesas)
- **MCU**: RA6T2 (R7FA6T2AAD3CFP, ARM Cortex-M33)
- **モーター**: maxon EC45 flat 75W (651614)
- **目的**: PMSMモーターのFOC制御（ホールセンサー使用）
- **用途**: 電動アシスト（マキタ18Vバッテリー駆動）
- **ワークスペース**: `C:\Users\ikeha\e2_studio\workspace\000\`

## 重要なファイル
- `src/hal_entry.c` - メイン制御ループ、ADCコールバック、状態管理、保護機能、I/O制御
- `src/FOCCurrentControl.c` - 電流制御（Simulink自動生成、手動修正あり）
- `src/FOCSpeedControl.c` - 速度制御、状態マシン（Simulink自動生成、手動修正あり）
- `src/ConfigParameters.c/.h` - モーターパラメータ、PIゲイン

## 現在の状態（2026-03-02 夜 更新）

### ADCモニタリング デバッグ記録

#### 問題: モニタリングADC（温度・電圧）が読めない
- 症状: temp_inv_adc_raw, temp_regen_adc_raw, battery_adc_raw が全て0

#### 原因1: Scan Group 1 の FIFO が無効
- FSPコンフィグレータで **Scan Group 1 の FIFO Enable = Disable** だった
- ADFIFOCR レジスタの FIFOEN1 = 0 → 変換結果がFIFOに格納されない
- R_ADC_B_FifoRead() は常に count=0 を返す
- **追加問題**: Group 1 は ScanGroupStart も呼ばれておらず、トリガー未有効化
- **追加問題**: Group 1 と Group 2 が同じ ADC Unit 1 + GPT_TRIGGER_A6 でトリガー競合
- **対策**: VIRT_CH_3-6 を Group 1 (group=2) → Group 2 (group=3) に移動
  - hal_data.c を手動編集（FSP再生成で上書きされる→要FSP設定変更）
  - → debug_fifo2_count = 6 になった（6チャネル分のFIFOエントリ確認）

#### 原因2: FIFOエントリ順序が想定と違った
- 想定: [0]=AN012, [1]=AN013, [2]=AN014...
- 実際: [0]=AN006(ch6), [1]=AN018(ch18), [2]=AN012(ch12), [3]=AN013(ch13), [4]=AN014(ch14), [5]=AN016(ch16)
- VIRT_CH_7(AN006) と VIRT_CH_8(AN018) が先にFIFOに入っていた
- → temp_inv_adc_raw = fifo[0].data → ch6 のデータ(0) を読んでいた
- **対策**: physical_channel フィールドで動的マッチング（switchでch番号判定）

#### 原因3: NTC温度計算式が反転していた
- コード: `R_ntc = R_pullup * ADC / (4095 - ADC)` ← VCC→R固定→ADC→NTC→GND の式
- 実回路: `3.3V → NTC(プルアップ) → ADCノード → 10kΩ(プルダウン) → GND`
- 正しい式: `R_ntc = R_pulldown * (4095 - ADC) / ADC`
- **対策**: 式を修正、NTC_PULLUP → NTC_PULLDOWN にリネーム

#### 原因4（真因）: ADCチャネル-ユニット物理ミスマッチ ★解決済み
- **症状**: AN012-AN016 の ADC値が実電圧に対して異常に小さく、チャネル間で補正比率が不均一
  - ch12(AN012): テスター1.05V → 期待ADC=1303, 実測FIFO=81, 比率=16.1
  - ch13(AN013): テスター1.067V → 期待ADC=1324, 実測FIFO=111, 比率=11.9
  - 比率が 16 と 12 で不均一 → ビットシフトではなくゴミデータ
- **追加証拠**: FIFO生レジスタ確認
  - debug_fifo2_raw[2] = 0x0C000051 (ch12, data=81, err=0)
  - debug_fifo2_raw[3] = 0x0D00006F (ch13, data=111, err=0)
  - Group 0 (Unit 0) の電流センサは正常な12ビット値（376, 338 等）
- **真因**: RA6T2 の BSP 定義により物理チャネルとADCユニットの対応が決まっている
  ```
  BSP_FEATURE_ADC_B_UNIT_0_CHANNELS = 0x000001E71FF3F03F
  BSP_FEATURE_ADC_B_UNIT_1_CHANNELS = 0x000001E71FFC0FC0
  ```
  - AN012, AN013, AN014, AN016 → **Unit 0 専用**（Unit 1 のマスクにビットなし）
  - AN006, AN018 → **Unit 1 専用**（Unit 0 のマスクにビットなし）
  - FSPコンフィグレータで Group 1 = Unit 1 に設定されていたため、
    Unit 0 専用チャネルを Unit 1 で変換しようとしていた
  - Unit 1 のアナログMUXに AN012-AN016 は接続されていない → 浮遊電圧を読む → ゴミデータ
  - ch6/ch18 の err=1 も整合（Unit 1 専用チャネルが Unit 0 グループでスキャンされた可能性）
- **修正（最終）**: VCH 3-6 を全て **Group 0 (Unit 0)** に統合
  - hal_data.c: VCH 3-6 の group = (1) (= Scan Group 0)
  - hal_entry.c: Group 0 FIFO から physical_channel で動的マッチング（7ch）
  - hal_entry.c: `<<4` 暫定補正を完全削除（正しいユニットなら不要）
  - Group 1: 無効化（チャネルなし）
- **FSPコンフィグレータ正式反映 完了** ✓
  - configuration.xml: VCH 3-6 → Scan Group 0、Group 1 → Disable
  - Generate Project Content 実行 → hal_data.c 再生成済み
  - 再生成後も動作確認済み（temp_inverter ≈ 24.4°C, temp_regen ≈ 24.6°C）

### ADC確定マッピング（修正後・実機確認済み）
```
Scan Group 0 (Unit 0, GPT_TRIGGER_A6|B6) - 電流計測 + モニタリング統合:
  7チャネル (FIFO深さ8に収まる), physical_channelで動的マッチング
  ch0  (AN000/PA00) = W相電流         → Iab[0]
  ch2  (AN002/PA02) = (予備)
  ch4  (AN004/PA04) = V相電流         → Iab[1]
  ch12 (AN012/PC00) = インバータ温度   → temp_inv_adc_raw (実測: 1231 → 24.4°C ✓)
  ch13 (AN013/PC01) = 回生回路温度     → temp_regen_adc_raw (実測: 24.6°C ✓)
  ch14 (AN014/PC02) = バッテリー電圧   → battery_adc_raw
  ch16 (AN016/P000) = 予備
  ※Unit 0 で正しく変換 → 12ビット値そのまま使用、<<4補正不要

Scan Group 2 (Unit 1, GPT_TRIGGER_A6) - 電圧モニタリング:
  ch6  (AN006/PA06) = VU_AD (U相電圧)
  ch18 (AN018/PB02) = VV_AD (V相電圧)
```

---


### 現在のPIゲイン（ConfigParameters.c）
```c
Kp_id = 0.3F    // 電流d軸
Ki_id = 0.0F    // 無効化（Ki>0で発振するため）
Kp_iq = 0.3F    // 電流q軸
Ki_iq = 0.0F    // 無効化
Kp_speed = 1.0F // 速度P（2.0→0.5→1.0: 起動摩擦>0.07閾値のため）
Ki_speed = 0.1F // 速度I（定常偏差解消。角度微分速度推定により発振せず使用可能に）
```

### 現在のパラメータ
```c
inverter.V_dc = (動的)   // battery_voltageからリアルタイム更新（公称18V、実測18.7V前後）
pmsm.N_base = 1559.0F   // ベース速度 [rpm]（PU正規化基準、モーター速度定数×V_dcから算出）
pmsm.V_boost = 0.05F    // オープンループ最小電圧比（0.15→0.05に低減済み）
V_DC_NOMINAL = 18.0F    // PWM電圧補償の基準電圧（FOCがこの電圧を前提にduty計算）
```

### 動作状況
- **速度制御（control_mode=0, Kp=1.0, Ki=0.1）** → 130〜170RPMで安定追従 ✓（目標156RPM）
- **トルク制御（control_mode=1, Iq=0.2）** → 231RPMで力強く滑らか ✓
- **トルク制御（Iq=0.07）** → 0〜120RPMで滑らか ✓（起動閾値≈0.07 PU）
- **PWM電圧補償** → 有効時のほうが振動音が小さい ✓（V_battery≈18.7V）
- **速度推定整合** → debug_f_get_speed_raw/8 ≈ debug_speed_from_angle ✓（10kHz確定後）
- TEST_BYPASS_FOC=1（直接電圧出力） → スムーズに回転（正常）

### 2026-03-02 実施した変更

#### 変更1: DRV8302 fault自動停止処理を追加
- nFAULT検出時にEnable=0、PWM=50%（ゼロ電圧）にラッチ停止
- 電源再投入まで復帰しない安全設計
```c
// hal_entry.c
if(drv8302_fault) { drv8302_fault_latched = 1; }
if(drv8302_fault_latched) { Enable = 0; /* PWM=50% */ }
```

#### 変更2: inverter.V_dc を 24V → 18V に修正
- 実バッテリー電圧（マキタBL1820B公称18V）に合わせた
- FOC内部の電圧正規化が正確になる

#### 変更3: バッテリー電圧・温度監視を実装
- AN1: バッテリー電圧監視（分圧回路 180k/15k、比率13倍）
- AN2: インバータ部温度（NTCサーミスタ 10kΩ/β=3435K）
- AN3: 回生回路温度（NTCサーミスタ）
- 保護ロジック: 低電圧警告(14V)→停止(12V)、高温警告(70°C)→ディレーティング(80°C)→停止(90°C)

#### 変更4: スイッチ入力を実装
- IN1 (DI_IN1): アシストレベル切替（L→M→H循環、デバウンス30ms）
- IN2 (DI_IN2): ターボモードトグル（デバウンス30ms）

#### 変更5: LED出力制御を実装
- OUT1: Power LED（緑）- 正常時点灯
- OUT2: Power LED（赤）- 異常時点灯、警告時点滅(2Hz)
- OUT3: Turbo LED - ターボモード時点灯
- OUT4-6: Assist Level LED（H/M/L）- 現在レベルのみ点灯

#### 変更6: ドキュメント整備
- 回生回路設計書: トリガ電圧22V→39Vに矛盾を統一修正
- 基板仕様書: [要確認]項目を可能な範囲で埋めた
- 既知の問題・注意事項セクションを記入

### 2026-03-03 実施した変更（V_dc補償・パラメータ調整・BLE通信）

#### 変更13: BLE UART通信実装（hal_entry.c） — FSP設定待ち
- DA14531MOD（CodeLess DataPump）を UART-BLE 透過ブリッジとして使用
- SCI3 UART (57600 8N1): PE05=RXD3, PE06=TXD3
- DA14531 リセット制御: PE02 (BTM_GPIO_RESET) でアクティブLowパルス
- テキストベースコマンドプロトコル実装:
  - `GET <var>` / `SET <var> <value>`: 変数の読み書き
  - `LOG START [hz]` / `LOG STOP`: ストリーミングログ（1〜100Hz）
  - `LIST`: 利用可能な変数一覧
- 変数レジストリ: 13変数（読み取り専用8 + 読み書き5）
- UART受信: `sci3_uart_callback` で1バイトずつ受信、CR/LFで行完了
- コマンド処理: `ble_uart_tick()` を One_ms_Int 内で毎ms実行
- LIST送信: 1ms毎に1変数ずつ送信（TX完了待ち）
- **前提**: FSP Configurator で r_sci_b_uart (ch3) を追加 → Generate が必要

#### 変更11: PWM電圧フィードフォワード補償（hal_entry.c）
- FOC出力（duty比）はV_DC_NOMINAL=18Vを前提としている
- バッテリー電圧が変動すると同じdutyで印加電圧が変わる問題を補償
- `compensated = 0.5 + (duty - 0.5) × 18.0 / battery_voltage`
- PWM出力直前に挿入、`vdc_compensation_en`でデバッガから有効/無効切替可能
- `inverter.V_dc`もリアルタイムの`battery_voltage`で更新

#### 変更12: torque_ref_iq_max 0.194 → 0.3（hal_entry.c）
- トルク制御モードの電流上限を引き上げ
- 定格電流 3.29A / ISenseMax 8.25A ≈ 0.4 PU に対し、0.3 PU は安全域

### 2026-03-02 深夜2 実施した変更（低速滑らか化・制御モード切替）

#### 変更7: FSP Hall モジュール設定修正（hal_data.c 手編集 3箇所）
- `u2_maximum_period`: 48 → **2000**
  - 原因: 低速でcarrier countが48でサチり、角度補間が途中で停止→コギング
  - 修正後: 6 RPMまで正しく角度補間可能
- `f_pwm_carrier_freq`: 20.0 → **10.0** ✓確定
  - `adc_rate_result = 10000` で ADCコールバック = 10kHz と確認
  - 旧値20.0では `f_get_speed` が2倍に膨張していた
  - 修正後: `debug_f_get_speed_raw / 8 ≈ debug_speed_from_angle` で整合確認済み
- `f4_start_speed_rad`: 250.0 → **0.0**
  - 原因: `f4_reach_time_msec=0` → `f4_add_pseudo_speed_rad=NaN` → `f_get_speed=NaN`
  - 修正後: pseudo speed 無効化、常に実測 `f_calculated_speed` を返す

#### 変更8: 角度微分による速度推定（hal_entry.c）
- 旧方式: `f_get_speed`（ホールエッジ時のみ更新、離散ジャンプ）
- 新方式: `f_get_angle` のスライディングウィンドウ微分（10ms窓、10kHz更新）
  - アンラップ→リングバッファ→ `(angle_now - angle_100ago) / 10ms`
  - IIRフィルタ係数 `speed_filter_alpha` でデバッガ調整可能（デフォルト0.10）
- 旧 stale-speed 検出は削除（停止時はangle_diff=0で自然にゼロ）

#### 変更9: 制御モード切替（hal_entry.c）
- `control_mode = 0`: 速度制御（SpeedControl_stepがIdqRefを管理、既存動作）
- `control_mode = 1`: トルク制御（速度ループバイパス、直接Iq指令）
  - `torque_ref_iq`: 直接Iq指令 [PU]（デバッガで変更）
  - `torque_ref_iq_max`: Iqクランプ上限（デフォルト0.194 PU）
- 両モードとも FOCCurrentControl_step（電流制御）は常時動作

#### 変更10: Kp_speed 2.0 → 0.5（ConfigParameters.c）
- 速度ノイズのトルク脈動への増幅を1/4に低減
- 線形制御範囲を9.7%→38.8%に拡大

---

## 過去の修正履歴（2026-02-02）

### 問題1: FOCSpeedControlの状態マシンがOpenLoopのまま
**原因:** SpeedRef < 0.15 なので遷移条件を満たさない
**修正:** FOCSpeedControl.c でOpenLoop状態でも常に `EnClOut = true` を出力

### 問題2: 速度PIの積分器が正の値で初期化
**原因:** `rtb_Add`（IqFb含む値）で初期化されていた
**修正:** 0で初期化

### 問題3: 電流PIの積分器も正の値で初期化
**原因:** Iq積分器が `rtb_Sum_ox`（オープンループの電圧）で初期化
**修正:** Id/Iq両方の積分器を0で初期化

### 問題4: hal_entry.cでIdqRefを上書きしていた
**修正:** IdqRef上書きコードを削除、SpeedControlに完全委譲

### 問題5: 電流Kpが低すぎてVqが小さい
**試行:** Kp_iq = 1.0 → 発振して電源落ち
**現在:** Kp_iq = 0.3（次回テスト）

---

## 次回やること

1. ~~**ADCモニタリング修正の検証** — 完了 ✓~~
2. ~~**FSPコンフィグレータで正式に反映** — 完了 ✓~~

3. ~~**低速滑らか化テスト** — 完了 ✓~~
   - トルク制御: Iq=0.07で0〜120RPM滑らか、Iq=0.194で230RPM滑らか
   - 速度制御: Kp=1.0, Ki=0.1 で130〜170RPM安定（目標156RPM）

4. ~~**f_pwm_carrier_freq の検証** — 完了 ✓ → 10kHz確定、10.0に変更済み~~

5. ~~**トルク制御モードテスト** — 完了 ✓~~
   - 起動摩擦閾値 ≈ 0.07 PU（これ以下では回転しない）

6. ~~**Ki_speed 再試行** — 完了 ✓ → Ki_speed=0.1 で安定動作~~

7. ~~**BLE UART通信を有効化** — 完了 ✓~~
   - SPS ファームウェア (921600 baud) で双方向通信確立
   - GET/SET/LIST/LOG コマンドプロトコル実装済み
   - MATLAB BLE モニターアプリ (`matlab/ble_monitor.m`) 作成済み

8. **LEDピン割り当て確認**（実機必要）

9. **パラメータ調整**（動作安定後）
   - ~~Iq制限: 0.194 → 0.3 — 完了 ✓~~
   - Iq制限: 0.3 → 0.5 PU（段階的に）
   - 速度域拡大テスト（SpeedRefIn_PU を上げる）

9. **デバッグ変数の整理**（全機能安定後）

---

## 確認済みの事実

### ホールセンサー角度
- `f_get_angle` は**電気角**を出力（機械角ではない）
- f_get_speedは**電気角速度** → 機械角速度に変換時に`/pmsm.p`が必要
- TEST_BYPASS_FOC=1 で hall_angle_offset = 0.0 が最適と確認済み

### 電流PIのKiは使えない
- Ki > 0 はすべて発振またはワインドアップを起こす
- P制御のみ（Ki=0）で安定動作

### 制御構造
```
速度指令 → [速度PI] → IdqRef[1] → [電流PI] → Vdq → [逆パーク] → Vabc → PWM
              ↑                       ↑
         SpeedFb_Hall_PU           IqFb
         (フィルタ済み)            (パーク変換後)
```

### 割り込みタイミング
```
ADCコールバック (rm_motor_driver_cyclic): 10kHz
  → 電流制御ステップ、ADC読み取り、ホールセンサ、PWM更新

1msタイマー (One_ms_Int): 1kHz
  → 速度制御ステップ、保護監視、スイッチ入力、LED制御、電源スイッチ、BLE UART処理
```

---

## デバッグ用変数
| 変数名 | 説明 |
|--------|------|
| SpeedRefIn_PU | 速度指令 [PU]（現在0.1、ランプ生成後） |
| SpeedFb_Hall_PU | フィルタ済み速度FB [PU] |
| SpeedFb_RPM | 速度FB [RPM] |
| IdqRef[1] | q軸電流指令（トルク）[PU] |
| IqFb | q軸電流FB [PU] |
| EnCl | クローズドループフラグ（SpeedControlから） |
| debug_speed_error | SpeedRef - SpeedFb |
| drv8302_fault | ゲートドライバフォルト（0=正常） |
| drv8302_fault_latched | ラッチ済みフォルト（電源再投入で解除） |
| battery_voltage | バッテリー電圧 [V] |
| temp_inverter | インバータ温度 [°C] |
| temp_regen | 回生回路温度 [°C] |
| protection_state | 保護状態（0=正常, 1=警告, 2=制限, 3=停止） |
| output_derating | 出力制限率（0.0～1.0） |
| assist_level | アシストレベル（0=L, 1=M, 2=H） |
| turbo_mode | ターボモード（0=OFF, 1=ON） |
| control_mode | 制御モード（0=速度, 1=トルク） |
| torque_ref_iq | トルクモード直接Iq指令 [PU] |
| torque_ref_iq_max | Iqクランプ上限 [PU]（デフォルト0.3） |
| speed_filter_alpha | 速度IIRフィルタ係数（0.03=重い, 0.10=標準, 1.0=なし） |
| debug_speed_from_angle | 角度微分速度 [rad/s mech] |
| debug_f_get_speed_raw | FSP生速度 [rad/s elec]（÷8で機械角、×9.55でRPM） |
| adc_rate_result | ADCコールバック実測レート [回/秒]（10000=10kHz） |
| vdc_compensation_en | PWM電圧補償（0=無効, 1=有効） |
| debug_angle_unwrapped | 累積アンラップ角度 [rad] |

| ble_log_streaming | BLEログストリーミング（0=停止, 1=動作中） |
| ble_log_interval | ログ送信間隔 [ms]（100=10Hz） |

### 2026-03-10 DA14531 BLE通信デバッグ＆確立

#### 問題: UART データが全て文字化け
- **症状**: RA6T2→スマホ: "PING" が `00 80 80 80 00 00 80...` に化ける
- **症状**: スマホ→RA6T2: "Hello" が 5×0xF0 または 0xFF に化ける
- **原因**: **ボーレート不一致**。RA6T2=115200、DA14531 SPS=921600

#### ファームウェア変更: CodeLess → SPS
- CodeLess DataPump (`DA14531_codeless_datapump.hex`) から SPS (`DA14531_sps_device.hex`) に変更
- SPS = Serial Port Service（DSPS 透過ブリッジ、AT コマンドなし）
- BLE アドバタイズ名: "SPS_531"
- **SPS のデフォルト UART ボーレート: 921600 baud**（ドキュメントに記載なし）

#### DA14531 SPS ボーレート測定方法（DWT サイクルカウンタ）
1. SCI3 UART を Close、PE05 を GPIO 入力に切り替え
2. ARM DWT->CYCCNT（240MHz CPU サイクルカウンタ）を有効化
3. `R_PORT14->PIDR` 直読みで PE05 の立ち下がりエッジを検出
4. LOW 期間の CPU サイクル数を測定
5. 検証: `0x00` 送信→9ビットLOW=2289サイクル、`A` 送信→1ビットLOW=254サイクル
6. 1ビット=254サイクル → 240MHz/254 ≈ **944kHz ≒ 921600 baud**

#### 解決: FSP で SCI3 UART を 921600 baud に変更
- FSP Configurator → g_uart3 → Baud Rate = 921600 → Generate
- 結果: BRR=7, coefficient=16 → 120MHz/(16×8) = 937,500 baud（誤差1.7%）
- **双方向通信成功**: PING→スマホ ✓、スマホ→RA6T2 "hello" ✓
- LIST コマンド動作確認 ✓

#### 教訓
- DA14531 SPS ファームウェアのデフォルト UART ボーレートは **921600**（CodeLess の 57600 とは異なる）
- DWT サイクルカウンタは GPIO ポーリングでの正確なタイミング測定に非常に有用
- `R_SCI_B_UART_BaudSet` API は使えない（UART が壊れる）。ボーレート変更は FSP Configurator で行うこと
- DSPS Flow Control characteristic に 0x01 を Write しないとデータ転送不可

#### PE02 (DA14531 P0_0) について
- **リセットピンではない**。LOW パルスを送ると DA14531 が起動できなくなる
- 常に HIGH を維持すること

#### MATLAB BLE モニターアプリ作成
- `matlab/ble_monitor.m`: リアルタイムグラフ、パラメータ変更、LOG START/STOP
- BLE DSPS サービス経由で SPS_531 に接続

### 2026-03-11 FSP SPIドライバ修正 & Simulinkアプリインターフェース準備

#### FSP SPI_B ドライバ修正（ICM-42605 IMU）
- **症状**: R_SPI_B_WriteRead が TRANSFER_COMPLETE（event=1）を返すが、受信バッファが常にゼロ
- ビットバング SPI では正常動作（WHO_AM_I=0x42、加速度・ジャイロ正常値）
- **根本原因1: 3線式モード**
  - FSP が `SPI_B_SSL_MODE_CLK_SYN`（Clock Synchronous = 3線式）で生成
  - 3線式では SPI ペリフェラルが CS(SSL0) を駆動しない
  - ICM-42605 は CS アサートなしではデータを返さない → MISO 常時ゼロ
  - **修正**: `SPI_B_SSL_MODE_SPI`（SPI Operation = 4線式）に変更
- **根本原因2: クロック位相**
  - FSP デフォルト `SPI_CLK_PHASE_EDGE_ODD`（立ち下がりエッジでサンプル）
  - ICM-42605 SPI Mode 3 は立ち上がりエッジでサンプルが正しい
  - ビットバングは低速のため立ち下がりでも偶然動作していた
  - **修正**: `SPI_CLK_PHASE_EDGE_EVEN`（立ち上がりエッジ）に変更
- **結果**: imu_who_am_i=0x42 ✓, imu_init_ok=1 ✓, cb_count 継続増加 ✓
- FSP Configurator で正式設定 → hal_data.c 手動編集ゼロ化達成
- ビットバングコード削除（USE_FSP_SPI 条件分岐を除去）

#### hal_data.c 手動編集ゼロ化
- Motor Sense Hall の3パラメータも FSP Configurator で設定可能と判明
  - PWM Carrier Frequency = 10（旧: 20.0→10.0 手編集）
  - Maximum Period = 2000（旧: 48→2000 手編集）
  - Start Speed = 0（旧: 250.0→0.0 手編集）
- SPI1: SPI Operation + Clock Phase EDGE_EVEN も FSP で設定
- **全5箇所の手動編集が不要に** → FSP 再 Generate 安心

#### 方針変更: Simulinkアプリは別プロジェクトで
- 当初 `assist_app_interface.h` を作成したが、方針変更
- ベースソフト = 汎用モータードライバー（速度制御 + トルク制御 + JOG運転）
- Simulink モデル（アシストスーツ等のアプリ）は別プロジェクトで組み込む
- `assist_app_interface.h` および関連コードは削除

### 2026-03-16 ベースソフト確認

#### 汎用モータードライバー ベースソフト機能一覧
- **速度制御** (control_mode=0): 速度PI → Iq指令、ランプ付き
- **トルク制御** (control_mode=1): 直接Iq指令、速度ループバイパス
- **BLE JOG操作**: SET コマンドで全パラメータ制御可能
  ```
  SET enable 1           ← モーターON
  SET control_mode 0     ← 速度制御モード
  SET speed_ref_rpm 100  ← 100RPMで回転（ランプ追従）
  SET torque_ref_iq 0.1  ← トルクモード時のIq指令
  SET enable 0           ← モーターOFF
  ```
- **BLE ストリーミング**: LOG START/STOP、IMU含む最大40 samp/s
- **保護機能**: 低電圧/高温警告・停止、DRV8302 fault ラッチ
- **IMU**: ICM-42605 (FSP SPI_B)、100Hz 6軸データ
- **MATLAB GUI**: ble_monitor.m でリアルタイムモニタリング

### 2026-03-17 LED検証・UART受信改善・MATLAB GUI改良

#### LED ハードウェアピン検証 ✓
- 全6ピン確認済み:
  | ピン | LED |
  |---|---|
  | PD15 | 電源 緑 |
  | PC06 | 電源 赤 |
  | PC07 | Turbo |
  | PD01 | Assist High |
  | PD02 | Assist Middle |
  | PD03 | Assist Low |
- BLE SET `led_state` で全 LED 個別点灯確認済み

#### LED 制御リファクタリング
- BLE デバッグオーバーライド（ble_debug_state, uart_rx_byte_count）を削除
- `led_mode` / `led_state` で auto/manual 切替:
  - led_mode=0 (auto): ベースドライバデフォルト（緑=正常, 赤=fault, Turbo=Enable中）
  - led_mode=1 (manual): BLE SET `led_state` のビットマスクで直接制御
  - Simulink アプリからも `led_state` に書き込むだけで LED 制御可能
- led_state ビット: bit0=PwrGrn, bit1=PwrRed, bit2=Turbo, bit3=AstH, bit4=AstM, bit5=AstL

#### UART 受信リングバッファ化
- **問題**: SET コマンドが時々取りこぼされる（2-3回送らないと反応しない）
- **原因**: `uart_rx_complete` フラグ中に来た次のコマンドのバイトが全て捨てられていた
- **修正**: ISR 内でリングバッファ (`rx_cmd_ring[4][64]`) にコマンドをキューイング
  - 前のコマンド処理中でも次のコマンドを受信可能
  - メインループは 1ms ごとに 1 コマンド処理
  - タイムアウト (200ms) による CR/LF なしコマンドもリングバッファ経由に統一

#### var_registry 更新（23変数）
- 追加: `led_mode` (uint8, rw), `led_state` (uint8, rw)
- 合計: 読み取り専用 14 + 読み書き 9 = 23 変数

#### MATLAB ble_monitor.m 改良
- **全画面表示**: 起動時に `WindowState = 'maximized'`
- **BLE 接続状態表示**: コントロールバー2行目に `BLE: Connected`(緑) / `Disconnected`(赤) / `Scanning...`(黄)
- **Reconnect ボタン**: 切断時にワンクリックで再接続
- **切断検知**: `onNotify` の read エラーで即座に切断状態に遷移（基板電源OFF検知）
- **右パネルタブ化**:
  - Parameters タブ: パラメータ GET/SET + Export CSV（enable, speed_ref_rpm, led_mode, led_state 追加）
  - Command タブ: Raw コマンド入力 + 大きなログエリア（200行保持）+ Clear Log
  - SET/GET 応答時に自動で Command タブに切替
- **CSV 保存フォルダ選択**: STOP+Save 時に `uigetdir` ダイアログ、フォルダを記憶

#### 残課題
- output_derating → PWM 出力への反映（温度保護）
- 電流 PI Ki > 0 で発振する問題
- デバッグ変数の整理（debug_fifo2_* 等の不要変数）

### 2026-03-18 BLEストリーミング安定化 & MATLAB描画改善

#### BLEストリーミング: burst-restart → 連続ストリーミング
- **旧方式**: `LOG START 1` + `F100` + `R` の往復（burst-restart）
  - MCU が1バースト送信 → 自動停止 → MATLAB が R 送信 → 再開
  - レースコンディション: R が streaming 中に届くと無視される（`ble_log_streaming==0` ガード）
  - 結果: 序盤に30秒以上の空白、不安定（~40 samp/s）
- **新方式**: `LOG START 100` で連続ストリーミング
  - MCU が 100Hz で自動送信し続ける（`log_burst_max=0` = auto-stop なし）
  - R コマンド不要、往復レイテンシなし
  - **注意**: `LOG START` のパラメータは**周波数(Hz)**であり burst_max ではない
  - `LOG START 0` → hz=0 → clamp to 1 → 1Hz になるバグあり（要注意）

#### MCU 変更 (hal_entry.c)
- R コマンドの `if (ble_log_streaming == 0)` ガードを削除
  - R は常に無条件でストリーミング再開（レースコンディション解消）

#### MATLAB ble_monitor.m 大幅リファクタリング

##### タイムスタンプ単調増加保証
- 旧: `tNow - (n - k) * dt` でバースト間でタイムスタンプが前後する
- 新: `tBurstStart = max(tNow - (n-1)*dt, csvTime(prev) + dt)` で常に単調増加

##### 描画アーキテクチャ変更（最重要の知見）
- **問題**: uifigure (Web/CEF renderer) の `drawnow` が時々10秒ブロックする
  - BLEコールバック処理と描画がトレードオフ
  - `drawnow` 強い → データ来る(98 samp/s) だが描画フリーズ
  - `drawnow` なし → データ来ない(0.5 samp/s)
- **解決**: プロットを従来型 `figure()` (Java renderer) に分離
  - 従来型 figure: 5段プロット、`drawnow` (full) でも高速描画
  - uifigure: コントロール専用（チャネル選択、ボタン、パラメータ、コマンドログ）
  - メインループ: `pause(0.05)` のみ（BLEコールバック処理）
  - refreshPlot (0.5s毎): `drawnow` (full) でイベントフラッシュ+高速レンダリング

##### 試行錯誤の記録（何がダメだったか）
| 構成 | samp/s | 描画 | 結果 |
|------|--------|------|------|
| uifigure + `drawnow` 1s毎 | 98 | 10秒フリーズ | データ◎ 描画× |
| uifigure + `drawnow limitrate` | 60 | ほぼ更新なし | データ○ 描画× |
| callback内 `drawnow` | 6.4 | × | BLEブロック |
| `pause` のみ (drawnow なし) | 0.5 | × | コールバック未処理 |
| `drawnow limitrate` のみ | 8 | × | データ不足 |
| timer + `drawnow` | 51 | 更新なし | timer実行されず |
| **従来figure + `drawnow`** | **94** | **滑らか** | **最終採用** |

##### その他の改善
- タイムアウト: 15s → 3s（`LOG START <hz>` 再送で高速回復）
- BLE接続直後は数十秒間データ不安定（DA14531コネクションパラメータ安定待ち）
- `sendTripleR()` 廃止、単発 R or 連続ストリーミングに統一
- PACK_SIZE=5（旧10）で ~55 bytes/burst

#### 最終性能（描画のみ）
- **93.8 samp/s** (100Hz MCU sampling, PACK_SIZE=5, 連続ストリーミング)
- **0 restarts** (安定後)
- **滑らかなリアルタイム描画** (従来型 figure, 0.5s 更新間隔)

### 2026-03-18 (続) JOG運転 & BLE MTU問題

#### JOGタブ追加 (ble_monitor.m)
- **MOTOR ON/OFF**: トグルボタン、ON時に自動LOG START + deferred enable
- **Speed/Torque モード切替**: ボタンで control_mode 0/1
- **Speed スライダー**: 0〜500 RPM
- **Torque スライダー**: 0〜0.3 PU
- **Quick Speed ボタン**: 0/50/100/200/300 RPM ワンタッチ
- **EMERGENCY STOP**: 赤ボタン、即 E0 + M0 + T0
- MOTOR OFF → 自動 stopLog（保存ダイアログ）

#### BLE MTU 文字化け問題の発見と対策

##### 症状
- `SET speed_ref_rpm 100` (23 bytes) → `peed_ref_rpm`, `speed_ref_rp`, `speed_rf_rpm` 等
- `SET torque_ref_iq 0.1` (23 bytes) → `torque_rf_iq`, `toque_ref_iq` 等
- `SET enable 1` (14 bytes) → 常に成功
- `SET control_mode 0` (20 bytes) → ほぼ成功

##### 原因
- DA14531 SPS の BLE MTU は ~20 bytes
- 20 bytes を超えるコマンドはBLEパケット分割され、分割時に文字が脱落
- ストリーミング中の双方向トラフィックでさらに悪化

##### 対策: MCU短縮コマンド追加 (hal_entry.c)
```
M<rpm>    → speed_ref_rpm (例: M100)     ≤6 bytes
T<iq>     → torque_ref_iq (例: T0.100)   ≤9 bytes
E0/E1     → enable                       2 bytes
C0/C1     → control_mode                 2 bytes
```
全て BLE MTU 以内。文字化け解消。

##### コマンドキュー
- MATLAB側で `cmdQueue` に積み、300ms間隔で1コマンドずつ送信
- 連続送信すると DA14531 が文字を落とすため

#### uifigure コールバック制約
- **`pause()` をuifigureコールバック内で使用禁止** — BLE DataAvailableFcn がブロックされる
- MOTOR ON → `startLog()` + フラグ設定 → メインループで `totalSamples > 10` 後にコマンド送信
- uifigureが重いと `pause(0.05)` のBLEコールバック処理効率が低下 → `drawnow limitrate` ブートストラップ追加

#### DA14531 SPS 動作制約（追加知見）
- ストリーミングなしだとコマンド応答がBLE通知されない（バッファがフラッシュされない）
- JOG操作にはストリーミング必須 → MOTOR ON で自動 LOG START

#### JOG運転の動作確認
- Speed 100 RPM でモーター回転確認 ✓
- RPM: 57〜96 RPM で変動（速度PI追従中）
- データレート: 9.5〜94 samp/s（DA14531接続品質に依存、要改善）

#### 残課題
- データレート不安定（9.5〜94 samp/s）— drawnow limitrate ブートストラップ時に低下
- output_derating → PWM 出力への反映（温度保護）
- 電流 PI Ki > 0 で発振する問題
- デバッグ変数の整理

### 2026-03-18 (続) uifigure完全廃止 → 94.6 samp/s + JOG運転達成

#### 問題: uifigure がBLEデータレートを破壊
- uifigure (Web/CEF renderer) の `drawnow` が全 figure のイベント処理をブロック
- JOGタブ追加で uifigure が ~90 コンポーネントに膨張
- 結果: 94 samp/s → **19.5 samp/s** に劣化（80% のBLE通知がロスト）
- `drawnow limitrate`, ブートストラップ, タイマー等 全て効果なし

#### 解決: uifigure 完全廃止
- **全 UI を従来型 figure + uicontrol (Java renderer) に移行**
- uicontrol: pushbutton, togglebutton, slider, edit, text, popupmenu, listbox
- 手動配置（normalized units）、見た目は地味だが動作は確実
- `drawnow` をメインループ毎回呼べるようになった（従来型 figure は高速）

#### 最終性能
- **94.6 samp/s** (100Hz MCU, PACK_SIZE=5)
- **99.2% BLE受信率** (MCU 3485 tx → MATLAB 1983+α msgs)
- **スムーズなリアルタイム描画** (0.5s更新)
- **JOG運転**: Speed 100 RPM でモーター回転確認 ✓
- **コマンド送受信**: 短縮コマンド (M/T/E/C) で文字化けなし ✓

#### 教訓: MATLAB の figure 選択
| | uifigure (CEF) | 従来 figure (Java) |
|---|---|---|
| drawnow 速度 | **10秒ブロック** あり | 常に高速 |
| BLE callback 処理 | 圧殺される | 問題なし |
| UI コンポーネント | モダン (uigridlayout等) | 基本的 (uicontrol) |
| **BLE + リアルタイム描画** | **使うな** | **これを使え** |

### 2026-03-18 (続) BLEコマンド検証・リトライ機構

#### BLE WriteWithoutResponse のパケットロスト問題
- BLE `WriteWithoutResponse` はパケット単位で完全にロストする（文字化けではなく消失）
- 失敗率 ~25%/コマンド（ストリーミング中の双方向トラフィック時）
- ストリーミング停止中でも ~12% 失敗（BLE無線層の問題）
- 2回送信: 6% 失敗、3回送信: 1.5% 失敗 → それでも不十分

#### 解決: 検証・リトライ機構
1. `S`, `S` (ストリーミング停止、2回)
2. `M300` (速度コマンド送信)
3. `R` (ストリーミング再開、1byte確実)
4. `?M` (検証クエリ、2bytes) → MCU応答 `=300\r\n` (6bytes)
5. MATLAB が応答を解析:
   - `=300` → VERIFY OK ✓
   - `=30` or `=0` → VERIFY FAIL → step 1 に戻り再送
   - 3秒タイムアウト → VERIFY TIMEOUT → step 1 に戻り再送
6. 最大5回リトライ

#### MCU追加コマンド (hal_entry.c)
- `?M` → `=<speed_ref_rpm>\r\n` (短い応答、BLE確実)
- `?T` → `=<torque_ref_iq>\r\n`
- GETALL応答 (~100 bytes) はBLEで常に文字化け → 検証に使えない

#### タイムアウト競合の修正
- cmdQueue処理中は LOG START リトライを抑制 (`isempty(cmdQueue)` チェック)
- 以前: タイムアウト発火 → `LOG START 100` (17bytes) がJOGコマンドに割り込み → 文字化け

#### テスト結果: 10/10 成功 (100%)
| 試行 | リトライ | 結果 |
|------|---------|------|
| 1 | 3回 (timeout×2, fail:got=0) | OK |
| 2 | 0回 | OK |
| 3 | 2回 (timeout×2) | OK |
| 4 | 0回 | OK |
| 5 | 1回 (timeout) | OK |
| 6 | 1回 (timeout) | OK |
| 7 | 0回 | OK |
| 8 | 2回 (fail:got=30, timeout) | OK |
| 9 | 0回 | OK |
| 10 | 0回 | OK |

#### 教訓: BLE コマンド信頼性
- `WriteWithoutResponse` は配達保証なし → 送りっぱなしは危険
- 短い応答 (`=300`, 6bytes) で検証可能、長い応答 (GETALL ~100bytes) はBLEで壊れる
- 送信→検証→リトライ のループが確実

### 2026-03-18 (続) ストリーミングデータ XOR チェックサム

#### 問題: BLE ストリーミングデータの信頼性
- BLE Notification もパケットロスト/文字化けが起こりうる
- ストリーミングは「自己回復型」（次パケットが来る）なので気づきにくい
- しかし文字化けしたデータが正常値として記録される可能性がある
  - 例: カンマ脱落で値がシフト、1文字欠落で "100" → "10"
  - パーサーは異常を検出できない（もっともらしい値になる）

#### 対策: XOR チェックサム
- MCU: パケット末尾に `*XX` (XOR checksum) を付与
  - 送信形式: `B5,23,67,...,data*A3\r\n`
  - チェックサム対象: `B` から最後のデータまで（`*` の手前）
- MATLAB: チェックサム検証、NG なら破棄 + `csumFail` カウント
- ステータス表示: `xxx samp | xx.x/s | 0 bad`

#### テスト結果
- 1452 メッセージ中 **1 件** csum_fail (0.07%)
- チェックサムなしなら壊れたデータがそのまま記録されていた
- 検出された破損データは確実に破棄 → **CSV は全て検証済みデータ**

#### 残課題
- 300 RPM 指令で 220 RPM しか出ない → 電流制御 Kp_iq=0.3 (P-only) で電圧出力不足
- output_derating → PWM 出力への反映（温度保護）
- 電流 PI Ki > 0 で発振する問題

### 2026-03-18 (続) 電流制御ゲイン調整 & 速度推定ノイズ

#### 問題: 300 RPM 指令で 210 RPM しか出ない
- Kp_iq=0.3 (P-only) で電圧出力不足
- IdqRef[1]=0.19 PU → Vq=0.3×0.19=0.057 PU ≈ 1V → 電流不足

#### Kp_iq 調整結果
| Kp_iq | 結果 |
|-------|------|
| 0.3 | 低速でゆっくり回る (100RPM→57-96RPM) |
| 0.5 | 動いたり動かなかったり。動くと振動大 |
| 0.8 | 0.1秒だけ発振→停止 (RPM=406スパイク) |
| 1.5 | 一瞬振動→即停止 |

#### 真因: 速度推定ノイズ
- Kp_iq を上げると電流制御が速度ノイズに追従→発振
- RPM が 10ms サンプル間で 130〜380 に暴れる（300 RPM 目標）
- ホールセンサー角度微分ベースの速度推定のノイズが大きい

#### 速度フィルタ調整
| speed_filter_alpha | RPM平均 | RPM標準偏差 |
|-------------------|---------|-----------|
| 0.1 (デフォルト) | ~270 | ~80 |
| **0.02** | **267** | **39** |

- alpha=0.02 で標準偏差半減。まだ振動あるがかなり改善

#### DA14531 SPS の WriteWithResponse 非対応
- `write(charRx, data)` (WithResponse) → "この characteristic はこのタイプの書き込みをサポートしません"
- DA14531 SPS Server RX は **WriteWithoutResponse のみ**
- verify-retry が唯一の信頼性確保手段

#### 残課題
- speed_filter_alpha さらに最適化 (0.01 等)
### 2026-03-18 (続) トルク制御安定化 & BLE安全対策

#### トルク制御モード最大速度リミッター
- `torque_max_speed_rpm` 変数追加（デフォルト 500 RPM）
- 90%〜100%でリニアにIqを絞る、100%超でIq=0
- BLEコマンド `W<rpm>` で設定可能
- MATLAB GUIにMax Speed入力欄追加

#### BLE文字化けによる危険値問題の発見
- `T0.150` → BLE文字落ち → `T150` → `torque_ref_iq = 150` (1000倍!)
- 過大電流でモーター即停止（一瞬暴走→Enable=0）
- **対策: MCU側で全短縮コマンドに安全クランプ追加**
  - T コマンド: `torque_ref_iq_max` (0.3) でクランプ
  - M コマンド: ±1000 RPM でクランプ
  - BLE文字化けで危険値になることを防止

#### Verify-retry のトルクモード対応
- トルクモード時は `?T` で `torque_ref_iq` を検証（`?M` ではなく）
- tolerance: speed ±1 RPM、torque ±0.01 PU
- **VERIFY OK: torque_ref_iq=0.15** 確認

#### 最終パラメータ
- Kp_iq = 0.5 (ConfigParameters.c に恒久化)
- speed_filter_alpha = 0.02 (hal_entry.c に恒久化)
- SPEED_EST_WINDOW = 200 (20ms、hal_entry.c)

#### トルク制御 最終性能
- **torque_ref_iq=0.15 PU, 無負荷**
- **平均 293 RPM、標準偏差 6.6** (目標300付近で安定)
- Iq = ±0.03 (無負荷で安定)
- 以前の速度制御 300RPM: std=39 → トルク制御の方が安定

#### CCW（逆回転）対応
- トルクスライダー: 0〜0.3 → **-0.3〜+0.3** に変更
- 正のIq = CW、負のIq = CCW
- MCU側の速度リミッターは絶対値判定済み（CCWでも正しくリミット）

### 2026-03-19 Vbat精度改善 & パラメータ設定アプリ & マキタAS端子

#### ① Vbat 精度改善
- 旧: `(unsigned)(battery_voltage + 0.5f)` → 整数 (常に19V)
- 新: `(int)(battery_voltage * 10 + 0.5f)` → 0.1V分解能 (18.8V等)
- MATLAB側: ÷10 でパース

#### ③ マキタバッテリAS端子 (MCU実装、未テスト)
- PE01→ダイオード→AS: 起床信号 (HIGH 10ms)
- P000(AN016)←220kΩ/22kΩ分圧←AS: 電圧検出
- `makita_battery_en`: 0=無効(デフォルト)、1=有効
- 起床シーケンス: PE01 HIGH 10ms → 60ms待ち
- 放電監視: AS < 5V が 15ms 以上継続 → 自己シャットダウン (LATCH_OFF)
- 5ms定期チェック対策: 15msデバウンス (5ms < 15ms なので誤検出しない)
- BLE変数: `as_voltage`, `makita_bat_state`, `makita_bat_en`

#### パラメータ設定アプリ (ble_config.m)
- `ble_monitor` とは独立したアプリ (BLE接続は排他)
- Connect → 自動 Stream ON → すぐ使える

##### BLE通信の課題と解決
| 方式 | 方向 | 信頼性 | 問題 |
|------|------|--------|------|
| WriteWithoutResponse | MATLAB→MCU | ~75% | パケット単位でロスト |
| Notification | MCU→MATLAB | ~99% | ほぼ確実 |

- **GET/SET コマンド**: 文字化け頻発 (>8bytes で顕著)
- **GETALL応答**: ~200bytes、BLEで常に壊れる → 使えない
- **G/P コマンド** (インデックス指定): `G26`(4bytes)→`#26=0.5000`(12bytes)
  - 短いが応答がストリーミングに埋もれて受信できないことがある
  - idx照合+自動リトライで対応

##### 解決: Dコマンド (MCU→MATLAB Notification活用)
- `D` (1byte) → MCU が 5ms間隔で `V<idx>=<value>` を全パラメータ送信
- Notification (99%信頼) で送るのでコマンド方向の問題を回避
- 35パラメータ × 5ms = 175ms で完了
- 2回Dump で **96%→99%+ 成功率**
- `VEND` で完了通知 → Ref Table 自動更新

##### Ref Table (参照テーブル)
- uitable で列揃え (日本語対応)
- No | 種類 | 定数名称 | 定数内容 | 単位 | 下限 | 上限 | デフォルト | 現在値
- R/Wパラメータ (No.1-17) + モニタ読取専用 (No.18-25)
- Read All (Dコマンド) で現在値が自動更新

##### MCU追加コマンド
- `G<idx>`: var_registry[idx] の値取得 → `#<idx>=<value>`
- `P<idx>=<value>`: var_registry[idx] に値設定 → `#<idx>=<value>`
- `D`: 全var_registry を V パケットで dump (5ms間隔、~175ms完了)
- PI ゲイン (Kp_iq等) を var_registry に追加 → BLE で GET/SET 可能に

##### BLE通信 根本対策: blastCmd (3連射)
- 全コマンドを 3 回連射 (50ms間隔)。1回75%成功 → 3回98.4%成功
- Connect時: R を最大10回送信、dataReceived フラグで到達確認
- Read All: D コマンド×2 (blast) で 96%+ の項目が埋まる
- 5秒毎の自動更新で残りも埋まる → 最終的に 100%
- Write: P コマンド blast → D で再読取り検証
- 応答表示: regIdx → Ref Table No に変換 (#23=2 → No.17=2)

##### regIdx マッピングバグ修正
- as_voltage: regIdx 32→31、makita_bat_state: 33→32
- regIdx=33 は範囲外 (VAR_REGISTRY_SIZE=33, 有効0-32) → 常に失敗していた

### 2026-03-19 (続) 手書きFOC電流制御 & ISenseMax修正

#### Simulink版 完全解析
- FOCCurrentControl.c (~1400行) の信号フロー解析
- Park変換規則: Id = Iα·cos + Iβ·sin, Iq = Iβ·cos - Iα·sin
- 逆Park: Vα = Vd·cos - Vq·sin, Vβ = Vq·cos + Vd·sin
- SVPWM: min-max injection + 2/√3 正規化
- 円リミッター: |Vdq| ≤ 0.95
- アンチワインドアップ: 飽和時に積分器停止（clamping方式）
- **arg_Iq出力はIq測定値（Park変換出力）= 正しい**
- 変数再利用が激しく可読性低い（rtb_algDD_o2_h が6回再利用）
- 速度FB出力に未初期化変数バグ（hal_entry.cで上書きされるので実害なし）

#### 手書きFOC電流制御実装
- Simulink版と完全同等のPark/逆Park変換
- ~60行のクリーンなコード（Simulink版1400行→95%削減）
- ADCオフセット起動時キャリブレーション追加（Iab_offset = {2049, 2061}）
- Enable立ち上がりで積分器リセット
- torque_ref=0 時は duty=0.5（ショートブレーキ、空走防止）

##### P-only動作確認 ✓
- Kp=0.5, Ki=0, torque=0.06: avg=64 RPM（Simulink版 avg=66 と同等）
- **Park変換の座標系は正しいことを確認**

##### Ki追加時の問題（未解決）
| Ki | Kp | 結果 |
|----|-----|------|
| 0 | 0.5 | ✓ 安定（RPM=64、低トルク） |
| 100 | 0.5 | 一度だけ RPM=474 で動作、再現性なし |
| 10 | 0.5 | 振動 (std=152) |
| 50 | 0.5 | 回らない or 一瞬振動 |
| 10 | 3.0 | 回らない（Kp高すぎ→帯域超過で不安定） |

##### 発振/不安定の分析
- **再現性がない**: 同じ設定で回ったり回らなかったり
- **ホールセンサ初期角度位置に依存**: 停止位置で初期トルク方向が変わる
- **Ki積分器+角度推定ノイズ**: Ki が角度誤差を増幅

#### ISenseMax 不整合の発見
- DRV8302 GAIN ピン: 10kΩプルダウン → LOW → **ゲイン 10V/V**
- シャント 10mΩ × ゲイン 10V/V → ISenseMax = 1.65/(0.01×10) = **16.5A**
- **Simulink の inverter.ISenseMax = 8.25 は間違い** → 16.5 に修正
- 1 PU = 16.5A（旧: 8.25A と想定していた）
- torque_ref_iq=0.06 の物理的意味: 0.06 × 16.5 = 1.0A

#### モーターパラメータ（データシート EC_flat_651614.pdf 確認済み）
| パラメータ | データシート | ConfigParameters.c | 備考 |
|-----------|------------|-------------------|------|
| 極対数 | 8 | pmsm.P = 8 | ✅ |
| 相抵抗 | 0.573Ω (相-相) | pmsm.Rs = 0.2865 | ✅ (Y結線 /2) |
| 相インダクタンス | 0.301mH (相-相) | pmsm.Ld/Lq = 0.1505mH | ✅ (/2) |
| 定格電流 | 3.29A | pmsm.I_rated = 3.29 | ✅ |
| 定格速度 | 4750 rpm | pmsm.N_rated = 4750 | ✅ |
| ISenseMax | 16.5A (計算) | inverter.ISenseMax = 16.5 | ✅ (修正済) |

#### 現在のConfigParameters.c
- Kp_id=0.5, Ki_id=50, Kp_iq=0.5, Ki_iq=50
- アンチワインドアップ ±0.4 (定格電流 3.29/16.5 ≈ 0.2 PU の2倍)

#### 次の予定（電流制御の安定化）
- Ki安定性問題の根本対策
  - ホールセンサ初期角度の対策（起動シーケンス改善）
  - Simulinkのアンチワインドアップ方式（clamping）を手書き版に導入
  - SVPWM追加（基本FOC安定後）
- ② 運転範囲制限 (連続3.29A/4400RPM + I²t過負荷保護)
- TI特性測定（CW/CCW、13V/18V/20.6V）
- EEPROM (R1EX24064ASAS01) でパラメータ永続化
- PU表示 → %表示への変更

### 2026-03-21 DA14531フロー制御調査 & UART通信障害

#### DA14531 SPS フロー制御の発見
- 詳しい人からの情報: DA14531 SPS は UART 側で DMA+RTS/CTS が必須
- RA6T2 CTS3(PE03/pin2) → DA14531 P0_7、RTS3(PE04/pin3) → DA14531 P0_8 が接続済み

#### UART 通信障害（約2時間のデバッグ）
- FSP Configurator でフロー制御を有効化→Generate→通信完全停止
- フロー制御を無効に戻しても復旧せず
- **根本原因: PE04 (PORT_14_PIN_04) の設定が消えた**
  - バックアップ(3/16): PE04 = OUTPUT LOW → DA14531 P0_8(CTS) に LOW = 送信許可
  - FSP Generate 後: PE04 = Disabled (フロート) → DA14531 が送信停止
  - **DA14531 SPS は P0_8 を CTS として実際に使っていた**
- PE04 を OUTPUT LOW に復元 → 通信復旧 ✓

#### 重要な教訓
- **PE04 (DA14531 CTS) は OUTPUT LOW 必須** — FSP で設定、Generate で消えないよう注意
- **PE03 (DA14531 RTS) はフロート** — DA14531 が RTS を駆動しているか未確認
- **FSP Generate は pin_data.c を上書きする** — ピン設定変更後は必ず確認
- **バックアップは重要** — 3/16 バックアップで原因特定できた

#### DA14531 SPS ピン構成（確認済み）
| RA6T2 ピン | DA14531 ピン | 機能 | 状態 | 設定 |
|-----------|-------------|------|------|------|
| PE05 (RXD3) | P0_6 (TX) | UART データ | ✓ 動作 | Peripheral SCI3 |
| PE06 (TXD3) | P0_5 (RX) | UART データ | ✓ 動作 | Peripheral SCI3 |
| PE03 | P0_7 (RTS) | DA14531 RTS出力 | ✓ 駆動確認 | **未設定 (GPIO入力)** |
| PE04 | P0_8 (CTS) | DA14531 CTS入力 | ✓ 動作確認 | **OUTPUT LOW 必須** |
| PE02 | P0_0 | Boot | ✓ | OUTPUT HIGH 必須 |

#### DA14531 フロー制御 調査結果

##### 確認できた事実
- DA14531 SPS は **P0_7=RTS, P0_8=CTS として使用している** (UART_AFCE_EN)
- PE04=LOW → DA14531 UART送信許可、PE04=フロート → DA14531 UART送信停止
- PE03 (DA14531 RTS): BLE接続中にトグル（0/1）= **RTS は駆動されている**
- SDK6 の user_periph_setup.h で確認: UART1_RTSN=P0_7, UART1_CTSN=P0_8, AFCE=EN

##### HW CTS を有効にした結果
- SCI3 Hardware CTS (PE03=CTS3ペリフェラル) を有効化 → **ストリーミング完全停止**
- 原因: DA14531 が RTS=HIGH (送信停止) を長時間維持
  - dbg_tx_write_count=1, uart_tx_busy=1 (1回の送信が完了しない)
  - tx_ring_count=8 (リングバッファ満杯)
- DA14531 SPS の RTS 管理が保守的すぎて、100Hz ストリーミングでは MCU 送信がほぼブロックされる

##### 結論
- **HW CTS は 100Hz ストリーミングに不適合** — DA14531 の RTS が高頻度で HIGH
- **現状の構成（PE04=LOW, PE03=未設定, フロー制御なし）が最適** — 94 samp/s, 99% 信頼
- フロー制御なしでも XOR チェックサムで破損データを検出・破棄

##### 将来のBLE品質向上オプション
- **ATT-MTU 拡大**: BLE パケット大型化 → DA14531 スループット向上 → RTS HIGH 時間短縮
- **DLE (Data Length Extension)**: BLE LL パケット拡大 → さらにスループット向上
- **DA14531 SPS ファームウェア再ビルド**: RTS FIFO 閾値調整、バッファサイズ拡大
- これらは MATLAB BLE 接続設定 + DA14531 SDK6 で対応可能

### 2026-03-21 (続) ADCキャリブレーション復活 & Ki安定性テスト

#### ADCオフセットキャリブレーション復活 ✓
- 500ms 起動時キャリブレーション（1000ms→500msに短縮）
- Iab_offset = {2049, 2061}（Iab[1]が13カウントずれ）
- FLOW_CONTROL_SUPPORT=0, PE03=未設定 で UART 正常動作確認

#### FSP Generate 注意事項（再発防止）
- **CTS/RTS Selection**: FSP で "Hardware RTS" に設定必須。"Hardware CTS" だと hal_data.c に `FLOW_CONTROL_CTS` が生成され SCI3 送信ブロック
- **PE04**: FSP で Output/Low に設定必須。Generate で消えないよう確認
- **FLOW_CONTROL_SUPPORT**: FSP で Disabled に設定。Generate 後に (0) を確認

#### ソフト Ki 起動の実装
- Enable ON → P-only (Ki=0) で起動 → 3秒かけて Ki を 0%→100% にランプ
- `ki_scale = ki_ramp_count / 30000.0f` (10kHz × 3秒)
- Ki_id = Ki_iq = 5.0（連続時間値 × Ts = 0.0005/step）

#### Ki 発振の詳細分析（300RPM 付近）
- torque=0.06, Ki=5, 3秒ランプで起動
- t=5.85-6.88: P-only → Ki ランプ中、RPM 38→326 に加速（安定）
- t=6.89: Ki が効き始め、**Iq が符号反転を開始**
  ```
  t=6.89: Iq=-0.11
  t=6.94: Iq=+0.17 ← 反転
  t=6.99: Iq=-0.27 ← 反転、振幅増大
  t=7.04: Iq=+0.29
  t=7.09: Iq=-0.45 ← アンチワインドアップ超過！Vbat=18.2V
  t=7.14: Iq=+0.44
  t=7.19: Iq=0 → RPM 急降下 → 停止
  ```
- **Iq が 100ms 周期で符号反転** → 電流制御ループの位相余裕不足
- P-only では安定（300RPM, Iq≈0）だが Ki を入れると発振
- **速度依存**: 低速（~100RPM）では安定、300RPM付近で発振開始

#### 発振の推定原因
- 10kHz サンプリングで電気的時定数 L/R = 0.15mH/0.29Ω ≈ 0.52ms
- サンプリング周期 0.1ms に対して時定数が短い（5サンプル分）
- Ki 積分器が蓄積 → 過補償 → 符号反転 → 振幅増大 → 発散
- Hall センサー角度推定ノイズも一因（300RPM で遷移間隔 ~2.5ms）

#### 20秒停止問題の原因特定
- `debug_enable_off_src=4` → **電圧保護(protection_state>=3)が発動**
- モーター回転中に電流増加 → 電源(19V/2.37A=45W)の容量超過 → Vbat < 12V → 保護停止
- 500ms デバウンス追加済みだが、電源ダウンは根本的に防げない

#### Ki 電流積み上がり問題の分析
- Ki=2 + torque=0.06: 起動時15秒間 error=0.06 を積分 → 積分器が蓄積
- 速度リミッターで IdqRef 低下しても積分器の残存値で Iq が高いまま
- 時間経過: Iq 0→0.02→0.04→0.07→0.09 と増加、Vbat 18.8→18.5→18.2→17.9
- 最終的に 45W 電源がダウン

#### 電源環境
- **現在**: PC電源アダプタ 19V/2.37A = 45W（モーター 75W に対して不足）
- **必要**: マキタ BL1820B (18V, 数十A) または 75W 以上の電源
- 電源不足では Ki の評価ができない（積分器蓄積 → 電流増加 → 電源ダウン）

#### Ki に関する一般論
- FOC 電流制御の Ki は産業用では必須（定常偏差解消）
- 適切な動作条件: 十分な電源容量、back-calculation アンチワインドアップ、電流リミット
- P-only でも無負荷なら動作するが、負荷時にトルク不足になる

### 2026-03-22 過電流事故 & ソフトウェア過電流保護実装

#### BLE デバイス選択機能
- blelist でスキャン → listdlg で選択 → アドレスで接続
- 基板①: 48233534029C、基板②: 482335340252

#### マキタバッテリテスト（P-only）
- Ki=0, torque=0.03: avg=95 RPM, Vbat=19.8V（安定）✓
- Ki=2, torque=0.03: avg=329 RPM, Vbat=19.8→19.3V（安定だが振動 std=120）

#### FOC 設計改善の実装
- **Iq LPF**: α=0.01→α=0.1 (カットオフ 160Hz)
- **逆起電力フィードフォワード**: Vq_ff = speed_PU, Vd_ff = -ωe×Lq×Iq
- **Back-calculation アンチワインドアップ**: 飽和時に積分を逆補正
- **速度リミッター**: SpeedFb_RPM → SpeedFb_Hall_PU（フィルタ済み）に変更
- **Id ストリーミング追加**: グラフで Id/Iq/Vbat 同時監視可能

#### ★★★ 過電流事故（基板②ハードウェア損傷）★★★
- **状況**: Ki=5 + 逆起電力FF + マキタバッテリで torque=0.03 テスト
- **原因**: モーターが回転しない状態で Ki 積分器が蓄積
  - 積分器安全クランプ ±0.5 PU → Vq=0.5 PU = 4.6V
  - 停止モーター（R=0.29Ω）に 4.6V/0.29Ω = **16A** が流れ続けた
  - **7秒以上の過電流** → モーターから煙
  - Vbat: 19.8→15.6V（マキタバッテリでも電圧降下）
- **なぜ保護が効かなかったか**:
  - **ソフトウェア過電流保護が存在しなかった** ← 根本原因
  - torque_ref_iq_max は指令値のクランプであり、実電流の監視ではない
  - アンチワインドアップ ±0.5 は停止モーターに 16A 流せる値だった
  - DRV8302 nFAULT は反応したが遅すぎた
- **被害**:
  - モーター: 煙発生（巻線絶縁劣化の可能性、要動作確認）
  - 基板②: DRV8302 fault ラッチ → モーター切離しで復旧確認
  - 基板②: モーター接続状態では電源投入時に即電源ダウン（要修理調査）

#### ソフトウェア過電流保護（事故後に実装）
```c
// 10kHz (ADC コールバック毎) で実電流を監視
#define OC_TRIP_PU  0.25  // 即座トリップ: 4.1A (定格125%)
if (|IqFb| > OC_TRIP_PU || |IdFb| > OC_TRIP_PU) {
    Enable = 0;          // 即座に停止
    overcurrent_fault = 1; // ラッチ（E0 でクリア）
    積分器リセット;
    PWM = 50%;
}
```
- **これがあれば今回の事故は Iq=0.25PU (4.1A) で即停止していた**
- 16A まで流れることはなかった

#### 積分器安全クランプも修正
- ±0.5 → **±0.1** PU（停止モーターへの最大電流 ≈ 0.1×9/0.29 = 3.1A）

#### 教訓（最重要）
1. **ソフトウェア過電流保護は最初に実装すべき** — Ki テスト前に必須
2. **積分器クランプは物理的な電流制限で計算する** — PU 値の意味を理解
3. **電源 ON でモーターコネクタを抜き差ししない** — 短絡電流で火花
4. **Ki は逆起電力 FF + 過電流保護 + 適切な積分器クランプの後に導入**

#### 現在の設定
- Kp_id=0.3, Ki_id=0, Kp_iq=0.5, Ki_iq=0 (P-only)
- Iq LPF α=0.1 (160Hz)
- 逆起電力フィードフォワード実装済み
- Back-calculation アンチワインドアップ実装済み
- 積分器クランプ ±0.1
- **ソフトウェア過電流保護 (OC_TRIP_PU=0.25) 実装済み**
- 電圧保護デバウンス 500ms
- torque_ref_iq_max = 0.2 PU (3.3A)
- ble_monitor: Id 表示追加、設定保存、BLE デバイス選択

#### 残課題
- **モーター動作確認**: 基板①ＰＣアダプタ, Ki=0, torque=0.01 で確認
- **基板②の修理調査**: DRV8302/MOSFET 状態確認
- Ki テスト: 過電流保護確認後に再開
- ② 運転範囲制限 (連続3.29A/4400RPM + I²t過負荷保護)
- TI特性測定
- EEPROM パラメータ永続化
- BLE 品質向上

### 2026-03-22〜23 手書きFOC完成・保護システム・UART修復

#### UART修復
- hal_data.c の flow_control が FSP Generate で CTS に戻る問題（3回再発）
- 原因: FSP Configurator の CTS/RTS Selection 設定
- 対策: Hardware RTS に変更して Generate → 恒久修正
- TXI3 割り込みが発火しない問題の真因: flow_control=CTS で SCI ハードウェアが CTS ピン待ち

#### モーター健全性確認
- Simulink FOC で 130-150RPM 安定動作確認 → モーター損傷なし
- トルクモード Iq=0.013PU（Simulink FOC）→ 巻線短絡なし
- 手で回して異音なし → 機械的正常

#### 手書きFOC 根本原因特定・修正
1. **OC保護がFOCを破壊**: ADC callback内でEnable=0 + Vabc_out=50%設定 → FOC制御不能
   - 対策: IdqRef をゼロにする方式（Enable は触らない）+ 起動グレース期間
2. **Anti-windup 積分器ドリフト (Ki=0)**: back-calculation の Kaw×(Vsat-V) が Ki=0 でも積分器を負に蓄積
   - 対策: Ki=0 では積分器更新をスキップ（クランピング方式に変更）
3. **FF ゲイン 1.93倍間違い**: N_base=1559 は PU 速度正規化用、電圧ベース速度は 3004RPM
   - BEMF at N_base = 0.52 PU（1.0 ではない）
   - 対策: FF ゲイン = 0.52
4. **EnCl_smooth=1 即座設定**: Simulink FOC は 1ms 遅延（速度コントローラ経由）
   - 対策: EnCl_smooth = EnCl（速度コントローラ出力）

#### 動作確認結果
- T30 トルク制御: 485RPM 安定回転 33秒（PC adapter で手動停止）
- T60 トルク制御: 467RPM 安定回転 10秒（PC adapter 電流不足で停止）
- FF ゲイン 0.52 で速度安定（振動なし）
- I²t 保護正常動作（37%で安全圏）
- Hall angle offset スイープキャリブレーション実装

#### 未解決課題
- PC adapter (2.37A) では高速・高トルク運転不可 → マキタバッテリ必要
- Hall angle offset 最適値未確定（-0.15 付近の可能性、要再キャリブレーション）
- VDC 補償: 起動時の問題で無効化中
- OC 保護: FOC ブロック内では無効化中（I²t + DRV8302 OCP のみ）
- Simulink FOC が 1000RPM 回れた理由の PU 系統分析が不完全

#### git 履歴
- 初回コミット: 87cd005 (Working baseline)
- 最新コミット: a3c8722 (Hall angle calibration)

---

## 2026-03-23 過熱事故と保護システム再設計

### 事故: モーター過熱 (3/23)
- I²t 100Hz デシメーションが Hall スパイクの発熱を過小評価
- FF が速度リミッタを無視 → RPM 700 まで暴走（目標 500）
- I²t 保護不発動 → Vbat 低下で停止（I²t ではない）
- **モーター過熱（異臭あり）、ただし手回し正常・動作可能**

### 保護システム設計書 v2 作成
- 全保護レイヤー（Layer 0～3）の設計
- 14 シナリオの机上検証
- コード修正前に設計レビュー完了

---

## 2026-03-24 保護システム v2 実装 + テスト

### 実装した修正（一括）
1. **OC 保護**: OC_TRIP_PU=0.40, |I|² 合成値 + 5ms 連続フィルタ + 500ms グレース
2. **I²t**: 100Hz デシメーション → raw I² + clamp 0.12 PU²
3. **FF 速度リミッタ連動**: ff_scale 変数追加、速度/トルク両モード対応
4. **RPM 上限保護**: 4500RPM 超で fault + Enable=0
5. **ストール検出**: RPM<10 + I²>0.01 で 3 秒 → fault
6. **P コマンド安全クランプ**: 全パラメータにハードコード範囲制限
7. **enable read-only**: var_registry から書込不可（E0/E1 のみ）
8. **I²t リセット**: E0 時 50% → 70%（warning zone で derating 即適用）
9. **ADC 故障保護**: battery_voltage < 0.5V → 低電圧停止
10. **OC 時 PI 積分リセット**: ストール電流の即時遮断
11. **トルクモード起動シーケンス**: アライメント(100ms Id=0.08) + ランプ(100ms)

### デバッグ経過
1. **OC_TRIP_PU=0.30 → 誤 trip**: 正常運転中の Hall Id スパイク (0.31 PU) で即 trip
   - 対策1: 個別軸 → |I|² 合成値に変更
   - 対策2: 5ms 連続カウンタ追加
   - まだ trip → 0.30 → 0.40 に引き上げ（|I|² 閾値 0.09 → 0.16）
2. **モーター起動失敗**: トルクモードに起動シーケンスがなく、特定ロータ位置から起動不可
   - 対策: d 軸アライメント (100ms) + q 軸ランプ (100ms)
3. **ストール時の電流継続**: OC trip 後も PI 積分が巻き上がったまま電流を流し続ける
   - 対策: overcurrent_fault 時に PI 積分もリセット

### hall_angle_offset キャリブレーション
- 自動スイープ実施: 13 オフセット × 1.5 秒
- **最適値: 0.25 rad** (≈14.3°) — コード初期値に反映
- Id_rms: 0.242 → 0.217 PU（若干改善）
- Hall 遷移スパイクの Id は offset では消えない（Hall FOC の本質的限界）

### テスト結果
- **T60 → 500RPM 安定回転**（PC adapter, ~25 秒）
  - OC trip なし（oc_peak_id=0.38 < 0.40）
  - i2t_ratio = 57%（warning 70% 手前で手動停止）
  - ff_scale = 1.0（正常）
  - Vbat 安定 18.4~18.8V
- 保護システム全体が設計通りに機能

### 残存課題
- Hall Id スパイク (Id_rms=0.22): Hall FOC 本質的限界。PLL/オブザーバーで改善可能
- VDC 補償未有効
- マキタバッテリでの全範囲テスト未実施
- EEPROM パラメータ永続化未実装

### git
- 3123772: INCIDENT: Motor overheated (3/23)
- (未コミット): Protection v2 全実装 + hall_angle_offset=0.25

---

## 2026-03-25 FOC改善・双方向制御・速度制御

### モーター交換
- 旧モーター（煙事故あり）→ 新モーター（同型 651614）に交換
- 旧モーターは巻線劣化の可能性 → 新モーターで全テストやり直し

### 主要変更一覧

1. **dq軸デカップリング追加** (旧Vq_ff_lpf置換)
   - Vd_decouple = -ωe × Lq_pu × Iq_fb
   - Vq_decouple = +ωe × (Ld_pu × Id_fb + FluxPM_pu)
   - 旧FF(speed_pu × 0.52)を完全置換

2. **Back-calculation アンチワインドアップ**
   - 電圧円リミッター飽和時に積分器を自動減少

3. **FSP Hallモジュール E1時リセット**
   - 原因: FSP内部のdirection=CWデフォルトバイアスがCCW起動を阻害
   - 対策: E1時にRM_MOTOR_SENSE_HALL_Reset()呼出

4. **EN_GATE制御**
   - E0: PWM Disable + EN_GATE=LOW → モーター完全解放（回生ブレーキなし）
   - E1: EN_GATE=HIGH + PWM Enable → FOC制御開始
   - 起動時: PWM Disable + EN_GATE=LOW（モーター軽い状態で起動）

5. **ADC再キャリブレーション (E1時)**
   - EN_GATE OFF状態（電流ゼロ）で100ms平均化
   - 電流センサ温度ドリフト対策

6. **Ki_id=0 (P制御のみ)** ★最重要修正
   - 原因: Hall遷移スパイクにDC偏りがあり、Id積分器が蓄積→ドリフト→不安定
   - 対策: Ki_id=0でId軸は比例制御のみ。Id_ref=0なので積分不要
   - 効果: CW/CCW両方向で30秒+安定、温度変化にもロバスト

7. **速度推定改善**
   - 窓: 200→100サンプル (20ms→10ms)
   - フィルタα: 0.02→0.05
   - デッドバンド: 双方向化 (前方向のみ→±0.5 rad/s)

8. **OC保護リファクタ**
   - Grace期間 5000cycles(500ms) + Sustain 50cycles(5ms)
   - 旧OC(1.0PU=無効)→新OC(0.40PU=6.6A)

9. **Lコマンド追加** (`L60` = torque_ref_iq_max = 0.06PU)

10. **BLEログ拡張** (IqRef_PU, IdRef_PU列追加)

11. **ソフトランプ起動** (100ms、d軸アライメント廃止)

12. **auto-sweep 2段階化** (粗0.05rad + 細0.005rad、現在offset中心±0.20)

### 動作確認結果

| テスト | 結果 |
|--------|------|
| トルク制御 T30 正転 | 安定 (Id=-0.020, 17s+, ドリフトなし) |
| トルク制御 T-30 逆転 | 安定 (Id=-0.009, 30s+, ドリフトなし) |
| トルク制御 T60~T200 | 動作OK (無負荷) |
| 速度制御 500RPM | 安定 (RPM=500, std=6) |
| 速度制御 1000RPM+ | ~900RPMで頭打ち (デカップリング√3補正なし) |
| E0モーター解放 | OK |

### 発見した技術的知見

1. **Id積分器ドリフトの根本原因**: Hall遷移スパイクの非対称偏り + 微小なhall_offset誤差が積分器に蓄積
2. **FSP Hallモジュールの方向バイアス**: リセット時direction=CW固定。CCW起動前にリセット必要
3. **SVPWM電圧ベース = V_dc/√3**: 旧FFゲイン0.52はこれを含んでいた。デカップリングにも√3が必要（次回対応）
4. **18Vで4248RPM到達可能**: 速度定数236RPM/V × 18V。弱め界磁不要

### 残存課題（次回セッション）

1. **デカップリング√3補正** → 逆起電力100%補償 → 速度上限4000RPM+
2. **N_base=4750 + 速度PI再設計** → 全速度域の速度制御
3. **高速域テスト** (1000→2000→3000→4000 RPM)

### git
- 9c412b0: FOCデカップリング, FSPリセット, EN_GATE制御
- 3606393: Ki_id=0, offset=0.01, ADC再cal
- a39233d: BLEログIdRef_PU, CW/CCW安定確認
- 2c36b1a: モーター設計書追加
- 11bcac7: ソフト管理表更新

## 最終更新
2026-03-25 - FOC改善完了、CW/CCW安定、次回√3補正+高速域対応
