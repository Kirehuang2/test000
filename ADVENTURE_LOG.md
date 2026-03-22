# モーター制御プロジェクト - 冒険の書

## プロジェクト概要
- **IDE**: e2 studio (Renesas)
- **MCU**: RA6T2 (ARM Cortex-M)
- **目的**: PMSMモーターのFOC制御（ホールセンサー使用）
- **ワークスペース**: `C:\Users\ikeha\e2_studio\workspace\000\`

## 重要なファイル
- `src/hal_entry.c` - メイン制御ループ、ADCコールバック、状態管理
- `src/FOCCurrentControl.c` - 電流制御（Simulink自動生成、手動修正あり）
- `src/FOCSpeedControl.c` - 速度制御、状態マシン（Simulink自動生成、手動修正あり）
- `src/ConfigParameters.c/.h` - モーターパラメータ、PIゲイン

## 現在の状態（2026-02-02 深夜）

### 現在のPIゲイン（ConfigParameters.c）
```c
Kp_id = 0.3F    // 電流d軸（0.1→1.0→0.3）
Ki_id = 0.0F    // 無効化
Kp_iq = 0.3F    // 電流q軸（0.1→1.0→0.3）
Ki_iq = 0.0F    // 無効化
Kp_speed = 0.5F // 速度（0.02→0.5）
Ki_speed = 0.0F // 無効化
```

### 動作状況
- **TEST_BYPASS_FOC=1（直接電圧出力）** → スムーズに回転（正常）
- **クローズドループ（Kp_iq=0.1）** → 520RPMで固定、47RPM指令に追従せず
- **クローズドループ（Kp_iq=1.0）** → 即座に発振、電源落ち
- **クローズドループ（Kp_iq=0.3）** → **未テスト**（次回確認）

### 今日発見した問題と修正

#### 問題1: FOCSpeedControlの状態マシンがOpenLoopのまま
**原因:** SpeedRef < 0.15 なので遷移条件を満たさない
**修正:** FOCSpeedControl.c でOpenLoop状態でも常に `EnClOut = true` を出力
```c
// FOCSpeedControl.c Line 101-107
default:
  /* case IN_OpenLoop: */
  /* MODIFIED: Always enable closed-loop since Hall sensor is available */
  *arg_SpeedRefOut_PU = arg_SpeedRef_PU;
  *arg_EnClOut = true;  // Always enable closed-loop
  break;
```

#### 問題2: 速度PIの積分器が正の値で初期化
**原因:** `rtb_Add`（IqFb含む値）で初期化されていた
**修正:** 0で初期化
```c
// FOCSpeedControl.c Line 163-175
if (FOCSpeedControl_DW.Integrator_IC_LOADING != 0) {
    FOCSpeedControl_DW.Integrator_DSTATE = 0.0F;  // 元は rtb_Add
}
```

#### 問題3: 電流PIの積分器も正の値で初期化
**原因:** Iq積分器が `rtb_Sum_ox`（オープンループの電圧）で初期化
**症状:** IdqRef[1] = -0.15 でもVqが正になり、モーターが減速しない
**修正:** Id/Iq両方の積分器を0で初期化
```c
// FOCCurrentControl.c Line 1141-1149, 1173-1181
FOCCurrentControl_DW.Integrator_DSTATE_n = 0.0F;  // Iq積分器
FOCCurrentControl_DW.Integrator_DSTATE_l = 0.0F;  // Id積分器
```

#### 問題4: hal_entry.cでIdqRefを上書きしていた
**原因:** 最初の1秒間IdqRef=0にし、その後iq_limitでクランプしていた
**修正:** IdqRef上書きコードを削除、SpeedControlに完全委譲
```c
// hal_entry.c Line 323-329
#else
// Normal closed-loop operation
// Let SpeedControl manage IdqRef directly (no override)
EnCl_smooth = 1;
// IdqRef is set by SpeedControl_step in One_ms_Int callback
// DO NOT override IdqRef here - speed controller needs full control
#endif
```

#### 問題5: 電流Kpが低すぎてVqが小さい
**原因:** Kp_iq = 0.1 → Vq = 0.1 × error = 小さすぎ
**試行:** Kp_iq = 1.0 → 発振して電源落ち
**現在:** Kp_iq = 0.3（次回テスト）

### 速度フィードバックのローパスフィルター
ホールセンサー速度のノイズを低減するため、フィルター済み速度を追加
```c
// hal_entry.c
float SpeedFb_Hall_PU = 0.0F;  // フィルタ済み
SpeedFb_Hall_PU = SpeedFb_Hall_PU * 0.95f + speed_raw * 0.05f;

// One_ms_Int callback
SpeedControl_step(Enable, SpeedRefIn_PU, SpeedFb_Hall_PU, ...);  // フィルタ済みを使用
```

## 次回やること

1. **Kp_iq = 0.3 でテスト**
   - 安定するか確認
   - IqFbがIdqRef[1]に追従するか確認
   - SpeedFb_RPMが47RPM付近に減速するか確認

2. **発振する場合**
   - Kp_iq を 0.2 に下げる
   - または速度フィルターを強化（0.98/0.02）

3. **減速しない場合**
   - IqFbの値を確認（IdqRefに追従しているか）
   - Vabc_outの値を確認（電圧が変化しているか）

## 確認済みの事実

### ホールセンサー角度
- `f_get_angle` は**電気角**を出力（機械角ではない）
- `pmsm.p = 8` を掛けてはいけない
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

## デバッグ用変数
| 変数名 | 説明 |
|--------|------|
| SpeedRefIn_PU | 速度指令 [PU]（現在0.03） |
| SpeedFb_Hall_PU | フィルタ済み速度FB [PU] |
| SpeedFb_RPM | 速度FB [RPM] |
| IdqRef[1] | q軸電流指令（トルク）[PU] |
| IqFb | q軸電流FB [PU] |
| EnCl | クローズドループフラグ（SpeedControlから） |
| debug_speed_error | SpeedRef - SpeedFb |
| drv8302_fault | ゲートドライバフォルト（0=正常） |

## 最終更新
2026-02-02 深夜 - 速度制御デバッグ中、Kp_iq=0.3で次回テスト予定
