

現行の実機ソフトと等価なSimulinkモデルを構築し、以下を達成する：

1. **ループ安定性評価** — 電流ループ・速度ループそれぞれで Bode/margin/sensitivity を取得
2. **ゲイン再設計の事前検討** — 実機試行錯誤を減らし、Kp/Ki 変更の影響を数値で予測
3. **PWM周波数変更影響の予測** — 18kHz化時の制御帯域・位相余裕を事前評価
4. **外乱応答の設計** — 負荷急変時の速度/電流応答を机上評価

**非対象（本モデルでは扱わない）**:
- BLE通信、ログ出力、パラメータSET/GET
- i2t熱保護（実時間ベースで動くが、制御安定性には非影響）
- オーバーカレント検出（安定性評価後の保護層）
- Hall校正シーケンス（定常動作を評価）

## 2. モデル全体構成

### 2.1 トップレベル構成

```
┌─────────────────────────────────────────────────────────────────┐
│ top.slx (top-level model)                                        │
│                                                                   │
│ ┌───────────────┐  IqRef  ┌───────────────┐  Vdq  ┌────────────┐ │
│ │               │────────▶│               │──────▶│            │ │
│ │  SpeedCtrl    │         │  CurrentCtrl  │       │  Inverter  │ │
│ │  (1kHz)       │         │  (10kHz)      │       │  + PMSM    │ │
│ │               │◀────────│               │◀──────│            │ │
│ └───────────────┘ SpdFb   └───────────────┘ Idq   └────────────┘ │
│         ▲                       ▲ theta_e              │         │
│         │                       │                       │         │
│         │                  ┌────────────┐              │         │
│         └──────────────────│ HallSensor │◀─────────────┘         │
│                            └────────────┘   theta_mech            │
│                                                                   │
│  SpeedRef (input)                                  LoadTorque    │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 サブシステム階層

```
top.slx
├── SpeedController    (離散 Ts_speed=1ms)
│   └── FOCSpeedControl（既存Simulinkコード参照、移植）
├── CurrentController  (離散 Ts=100μs、18kHz化時は Ts=55.56μs)
│   ├── Clarke
│   ├── Park           (hall_angle + angle_advance)
│   ├── Decoupling     (ωe×L×I + back-EMF)
│   ├── PI_d / PI_q
│   ├── CircleLimiter  (voltage saturation + back-calc AW)
│   ├── RealCurrentLimiter
│   ├── InvPark
│   └── SVPWM          (min-max 3rd harmonic injection)
├── InverterPlant      (連続時間)
│   ├── SVPWM→Vabc     (理想、または dead time 含む)
│   ├── DeadTimeModel  (任意、非線形歪みモデル)
│   ├── Vabc→Vdq       (解析用)
│   └── PMSM_dqModel   (状態方程式 Ld, Lq, FluxPM, P, J, B)
└── HallSensorModel    (離散化角度 60° step)
```

## 3. 座標系と信号定義

### 3.1 座標系の規約（コードと一致）

| 座標 | 定義 | 備考 |
|------|-----|------|
| abc | モーター3相（A=巻線3, B=巻線2, C=巻線1） | コードで Vabc_out[0]=C, [1]=B, [2]=A |
| αβ | 静止座標（Clarke変換後） | α=A相, β=(A+2B)/√3 |
| dq | 回転座標（Park変換後） | d=磁極方向, q=dに90°進み |

### 3.2 PU正規化（実機と同じ）

| 量 | 基準値 | 備考 |
|----|-------|------|
| 電流 | ISenseMax = 16.5 A | 1.0 PU = 16.5A |
| 電圧 | V_dc/√3 = 10.39 V | SVPWM実効 |
| 速度 | N_base = 4750 RPM | 定格回転数 |
| 時間 | 秒（PU化しない） | Simulink規約 |

### 3.3 信号命名規則

コードの変数名をそのまま使用。`SpeedFb_RPM`, `IqRef_PU`, `Iq_fb`, `Vd_sat`, `hall_angle` 等。

---

## 4. ブロック詳細仕様

### 4.1 PMSM プラント（Inverter.PMSM_dqModel）

**状態方程式（dq座標系、連続時間）**:

```
Ld * dId/dt = Vd - Rs*Id + ω_e*Lq*Iq
Lq * dIq/dt = Vq - Rs*Iq - ω_e*Ld*Id - ω_e*FluxPM
J  * dω_m/dt = Te - TL - B*ω_m
Te = (3/2) * P * FluxPM * Iq      (Id=0 制御時)
ω_e = P * ω_m
dθ_m/dt = ω_m
```

**I/O仕様**:

| ポート | 方向 | 単位 | 型 | 備考 |
|-------|-----|------|---|------|
| Vd_phys | In | V（実電圧） | real | PU → V変換後に入力 |
| Vq_phys | In | V | real | |
| T_load | In | N·m | real | 負荷トルク（外乱） |
| Id_phys | Out | A | real | 実電流、PU化してCtrlへ |
| Iq_phys | Out | A | real | |
| omega_m | Out | rad/s | real | 機械角速度 |
| theta_m | Out | rad | real | 機械角（積分） |
| Te | Out | N·m | real | 電磁トルク（解析用） |

**パラメータ**（コード `pmsm` 構造体と一致）:

| param | 値 | 単位 | 備考 |
|-------|-----|------|------|
| Rs | 0.2865 | Ω | 相抵抗 |
| Ld | 0.1505e-3 | H | 相インダクタンス |
| Lq | 0.1505e-3 | H | 同上（表面磁石型） |
| FluxPM | 0.002915 | Wb | Kt/(√3×P) |
| P | 8 | - | 極対数 |
| J | 1.81e-5 | kg·m² | ロータ慣性 |
| B | 0.001 | N·m·s/rad | 摩擦（推定） |

**実装方法（Simulink）**:
- Simscape Electrical の `PMSM` ブロック（物理モデル、最も簡単）、または
- 手書き MATLAB Function / Integrator で上記状態方程式

Simscape版推奨（Park/Clarke内蔵、検証済）。

### 4.2 インバーターモデル（Inverter.SVPWMtoVabc）

**レベル1（理想、まず安定性評価用）**:
```
Vd_PU, Vq_PU → Inv Park → Valpha_PU, Vbeta_PU
             → Inv Clarke + SVPWM (min-max injection) → Vabc_PU
             → × (V_dc/√3) → Vabc_phys [V]
```

ブロック: `MATLAB Function` で実装。
参考コード [hal_entry.c:1510-1520](src/hal_entry.c#L1510):
```c
float Va = Valpha;
float Vb = 0.866025f * Vbeta - 0.5f * Valpha;
float Vc = -0.5f * Valpha - 0.866025f * Vbeta;
float Vmax = max(Va,Vb,Vc);
float Vmin = min(Va,Vb,Vc);
float offset = (Vmax + Vmin) * 0.5f;
Va -= offset; Vb -= offset; Vc -= offset;
```

**レベル2（デッドタイム歪み込み、必要時）**:
- 相電流符号検出 → デッドタイム電圧降下 `Δduty = 2×t_dead/T_pwm` を減算
- 非線形モデル、ゼロクロスでsign関数チャタリング再現

### 4.3 Hall センサーモデル（HallSensorModel）

**実機動作**: 60°ステップで離散角度を返す（8極対なので機械45°で1電気360°）

**Simulink実装**:
```
theta_m × P = theta_e_continuous
    → mod(2π)
    → 60°で量子化 (floor(theta_e / (π/3)) × π/3 + π/6)
    → + hall_angle_offset (0.20 rad)
```

**パラメータ**:
- `hall_offset = 0.20` (rad)
- `hall_step = pi/3` (60° = π/3 rad)

**I/O**:
| ポート | 方向 | 単位 | 備考 |
|-------|-----|------|------|
| theta_m | In | rad | プラントから |
| theta_hall | Out | rad | 電気角、60°量子化 |

**速度推定**: 実機はHall遷移時間から推定（デルタ方式）。モデルでは簡単に `d/dt(theta_m) × P × 60/(2π)` で RPM 直接取得、または実装模倣の遅延+量子化モデル。

### 4.4 電流コントローラ（CurrentController）

**サンプル時間**: `Ts = 100e-6` (10kHz)

**構成（Simulink Blocks）**:

```
Iab_PU ──▶ Clarke ──▶ ┬─▶ Ialpha_PU
                       └─▶ Ibeta_PU
                       
theta_hall + omega_e_filt × angle_advance_us × 1e-6 ──▶ theta_e_adv
                       
[Ialpha, Ibeta, theta_e_adv] ──▶ Park ──▶ [Id_raw, Iq_raw]

[Id_raw, Iq_raw] ──▶ LPF(α=0.1) ──▶ [Id_fb, Iq_fb]  (fc≈159Hz)

[omega_m × P] ──▶ LPF(α=0.006) ──▶ omega_e_filt  (fc≈10Hz)

Vd_decouple = -omega_e_filt × Lq_pu × Iq_fb
Vq_decouple = +omega_e_filt × Ld_pu × Id_fb + omega_e_filt × FluxPM_pu

Id_error = IdRef - Id_fb
Iq_error = IqRef - Iq_fb

Vd_raw = Kp_id × Id_error + Id_integral + Vd_decouple
Vq_raw = Kp_iq × Iq_error + Iq_integral + Vq_decouple

[Vd_raw, Vq_raw] ──▶ CircleLimiter (|V|<0.95) ──▶ [Vd_sat, Vq_sat]
                                                  │
(Vd_sat - Vd_raw) × (1/Kp_id) ──▶ Id_integral_anti_windup (back-calc)
(Vq_sat - Vq_raw) × (1/Kp_iq) ──▶ Iq_integral_anti_windup

Id_integral(k+1) = Id_integral(k) + Ki_id × Ts × Id_error + Kb × (Vd_sat - Vd_raw)
                  (clamp ±int_limit)

[Vd_sat, Vq_sat, |I|²] ──▶ RealCurrentLimiter (|I|² > 0.16 → scale)
```

**I/O（CurrentController サブシステム）**:

| ポート | 方向 | 単位 | 備考 |
|-------|-----|------|------|
| IdRef_PU | In | PU | 通常0 |
| IqRef_PU | In | PU | 速度ループ出力 or 直接T指令 |
| Iab_PU | In [2] | PU | A相・B相電流（C相は -(Ia+Ib)） |
| theta_hall | In | rad | 電気角 |
| omega_m | In | rad/s | 機械角速度（デカップリング用） |
| Enable | In | bool | E1フラグ |
| Vdq_PU | Out [2] | PU | インバーターへ |
| Id_fb_PU | Out | PU | 診断 |
| Iq_fb_PU | Out | PU | 速度ループへ（フィードバック） |

**パラメータ**（コード `PI_params` と一致）:

| param | 値 | 備考 |
|-------|-----|------|
| Kp_id | 0.30 | |
| Ki_id | 0.00 | **P制御のみ**（Idドリフト防止） |
| Kp_iq | 0.50 | |
| Ki_iq | 0.10 | |
| int_limit | 0.50 | PU |
| Kb_id | 1/Kp_id ≈ 3.33 | back-calc |
| Kb_iq | 1/Kp_iq = 2.00 | back-calc |
| Ts | 100e-6 | s（10kHz） |

### 4.5 速度コントローラ（SpeedController）

**既存資産**: `src/FOCSpeedControl.c` は Simulink R2023a 生成済。元の `FOCSpeedControl.slx` が失われていれば、下記仕様で再構築。

**サンプル時間**: `Ts_speed = 1e-3` (1kHz)

**構成**:
```
SpeedRef_PU ──┐
              ├──▶ Error ──▶ × Kp_speed ──▶ Vp
SpeedFb_PU ───┘                              │
                                              ├──▶ + ──▶ IqRef_raw
              Error ──▶ × Ki_speed × Ts_speed ──▶ Integrator ──▶ Vi
                                                    │
                                           anti-windup (back-calc)

IqRef_raw ──▶ Saturation ±0.20 PU ──▶ IqRef
            │
            └──▶ (IqRef - IqRef_raw) × Kb_speed ──▶ integrator correction
```

**I/O**:

| ポート | 方向 | 単位 | 備考 |
|-------|-----|------|------|
| Enable | In | bool | |
| SpeedRef_PU | In | PU | Mコマンドから |
| SpeedFb_PU | In | PU | Hall速度推定 |
| IqFb | In | PU | back-calc用（未使用なら省略） |
| Mode | In | int | 速度/トルク切替 |
| IdqRef_PU | Out [2] | PU | 電流ループへ |
| SpeedRefOut_PU | Out | PU | ソフトランプ後 |
| EnClOut | Out | bool | クローズドループ有効 |

**パラメータ**:

| param | 値 |
|-------|-----|
| Kp_speed | 0.78 |
| Ki_speed | 1.5 |
| IqRef_max | 0.20 PU |
| Kb_speed | 1/Kp_speed ≈ 1.28 |
| Ts_speed | 1e-3 s |

**ソフトランプ**: SpeedRef → 実指令間にレートリミッタ `speed_ramp_rate = 0.00004 PU/call @10kHz` = 0.4 PU/s = 1900 RPM/s。電流ループ側で実装されているが、モデルではSpeedCtrl出力に Rate Limiter を置く。

---

## 5. 信号接続（配線図）

### 5.1 配線リスト（階層間）

| 送信元 | 送信信号 | 受信先 | 備考 |
|--------|---------|--------|------|
| Input (root) | SpeedRef_PU | SpeedController.SpeedRef_PU | ユーザー指令 |
| Input (root) | Enable | SpeedCtrl.Enable, CurrCtrl.Enable | 共通 |
| SpeedCtrl | IdqRef_PU[1] (=IqRef) | CurrCtrl.IqRef_PU | 主経路 |
| SpeedCtrl | IdqRef_PU[0] (=IdRef, 通常0) | CurrCtrl.IdRef_PU | |
| Constant 0 | IdRef | CurrCtrl.IdRef_PU | (speed modeでは0) |
| CurrCtrl | Vdq_PU | Inverter.Vdq_PU | PWM指令 |
| CurrCtrl | Iq_fb_PU | SpeedCtrl.IqFb | back-calc用 |
| Inverter | Iab_phys → /ISenseMax → Iab_PU | CurrCtrl.Iab_PU | 電流センサ |
| HallModel | theta_hall | CurrCtrl.theta_hall | Park変換用 |
| Inverter | omega_m | CurrCtrl.omega_m | decoupling用 |
| Inverter | omega_m × 60/(2π) / N_base | SpeedCtrl.SpeedFb_PU | 速度FB |
| Inverter | theta_m | HallModel.theta_m | |
| Input (root) | T_load | Inverter.T_load | 外乱 |

### 5.2 レート遷移（Rate Transition）

Simulink で異なるサンプルレート接続時は `Rate Transition` ブロック必須：

| 遷移 | 方向 | ブロック設定 |
|------|-----|------------|
| SpeedCtrl (1ms) → CurrCtrl (100μs) | 下流が速い | Deterministic data transfer (ZOH) |
| CurrCtrl (100μs) → SpeedCtrl (1ms) | 下流が遅い | Deterministic data transfer |
| CurrCtrl (100μs) → Inverter (continuous) | 離散→連続 | Zero-Order Hold |
| Inverter (continuous) → CurrCtrl (100μs) | 連続→離散 | Sample+Hold (周期 100μs) |

### 5.3 CurrentController 内部ブロックリスト（Ts=100μs）

**入力ポート（サブシステム境界）**

| # | 名称 | 型/次元 | 上位モデルの接続元 |
|---|------|--------|------------------|
| IN1 | IdRef_PU | scalar | (top) SpeedCtrl.IdqRef[0] または Const 0 |
| IN2 | IqRef_PU | scalar | (top) SpeedCtrl.IdqRef[1] via Rate Transition |
| IN3 | Iab_PU | vec[2] | (top) Inverter.Iab_phys ÷ ISenseMax |
| IN4 | theta_hall | scalar (rad) | (top) HallModel.theta_hall |
| IN5 | omega_m | scalar (rad/s) | (top) Inverter.omega_m via S+H |
| IN6 | Enable | bool | (top) root Enable |

**内部ブロック（Simulink標準ブロック最小単位）**

| ID | ブロック種別 | 入力元（複数は`,`区切り） | 出力先 | パラメータ / 式 |
|----|------------|--------------------------|--------|---------------|
| B01 | `Gain` | IN5 (omega_m) | B02 | K = `pmsm.P` (= 8) → omega_e_raw |
| B02 | `Discrete Filter` (1st) | B01 | B03, B11 | α = `alpha_decouple` (0.006) → omega_e_filt |
| B03 | `Gain` | B02 | B04[2] | K = `angle_advance_us * 1e-6` |
| B04 | `Sum` [+,+] | IN4, B03 | B05, B06 | θ_e_adv = θ_hall + Δθ |
| B05 | `Trigonometric Function` (sin) | B04 | B08, B24 | sin_θ |
| B06 | `Trigonometric Function` (cos) | B04 | B08, B24 | cos_θ |
| B07 | `MATLAB Function` Clarke | IN3 (Iab_PU) | B08, B23 | Iα=Ia; Iβ=(Ia+2·Ib)/√3 |
| B08 | `MATLAB Function` Park | B07, B05, B06 | [Id_raw]→B09, [Iq_raw]→B10 | Id=Iα·cos+Iβ·sin; Iq=Iβ·cos−Iα·sin |
| B09 | `Discrete Filter` (1st) | B08 (Id_raw) | B11, B12, OUT4 | α = `alpha_iq_lpf` (0.1) → Id_fb |
| B10 | `Discrete Filter` (1st) | B08 (Iq_raw) | B11, B13, OUT3 | α = 0.1 → Iq_fb |
| B11 | `MATLAB Function` Decoupling | B02, B09, B10 | [Vd_dec]→B18, [Vq_dec]→B19 | Vd_dec=−ωe·Lq_pu·Iq_fb; Vq_dec=+ωe·Ld_pu·Id_fb+ωe·Φ_pu |
| B12 | `Sum` [+,−] | IN1, B09 | B14, B16 | Id_err |
| B13 | `Sum` [+,−] | IN2, B10 | B15, B17 | Iq_err |
| B14 | `Gain` | B12 | B18 | K = Kp_id (0.30) |
| B15 | `Gain` | B13 | B19 | K = Kp_iq (0.50) |
| B16 | `Discrete-Time Integrator` | B12 (err入力), B21 (ext reset or AW input), IN6 (reset) | B18 | Gain = Ki_id·Ts (=0); Sat = ±int_limit (0.5) |
| B17 | `Discrete-Time Integrator` | B13, B22, IN6 | B19 | Gain = Ki_iq·Ts (=1e-5); Sat = ±0.5 |
| B18 | `Sum` [+,+,+] | B14, B16, B11 (Vd_dec) | B20 | Vd_raw |
| B19 | `Sum` [+,+,+] | B15, B17, B11 (Vq_dec) | B20 | Vq_raw |
| B20 | `MATLAB Function` Circle Limiter | B18, B19 | [Vd_sat, Vq_sat]→B23, [Vd_err]→B21, [Vq_err]→B22 | \|V\|²>0.9025 なら scale=0.95/\|V\| |
| B21 | `Gain` | B20 (Vd_err) | B16 (AWポート) | K = Kb_id = 1/Kp_id ≈ 3.33 |
| B22 | `Gain` | B20 (Vq_err) | B17 (AWポート) | K = Kb_iq = 1/Kp_iq = 2.0 |
| B23 | `MATLAB Function` RealI Limiter | B20 (Vd_sat, Vq_sat), B07 (Iα, Iβ) | [Vd_out, Vq_out]→B24 | \|I\|²>0.16 なら scale=I_rated/\|I\| |
| B24 | `MATLAB Function` InvPark+SVPWM | B23, B05, B06 | OUT1 | Vα=Vd·cos−Vq·sin; Vβ=Vq·cos+Vd·sin; Vabc=InvClarke(Vα,Vβ)+SVPWMoffset |

**出力ポート**

| # | 名称 | 型/次元 | 上位モデルの接続先 |
|---|------|--------|------------------|
| OUT1 | Vabc_PU | vec[3] | (top) Inverter.Vabc_PU (B24→ZOH→Inverter) |
| OUT2 | EnClOut | bool | （未使用、SpeedCtrlのmask入口で判定） |
| OUT3 | Iq_fb_PU | scalar | (top) SpeedCtrl.IqFb via Rate Transition (100μs→1ms) |
| OUT4 | Id_fb_PU | scalar | (top) Scope（診断用） |

**備考**:
- B16, B17 の Discrete-Time Integrator は "Limit output"、"External reset = level（Enable=0で0）"、および back-calc用 "External reset or Aux input" を使用。Simulinkで back-calc 実装する場合は、通常 `Discrete-Time Integrator`（Clamping + Back-calc オプション有り）または手動で積分器ループ組立。
- B11 Decoupling の係数 `Lq_pu, Ld_pu, FluxPM_pu` はワークスペース変数参照。
- B24 InvPark+SVPWM のコード対応: [hal_entry.c:1507-1520](src/hal_entry.c#L1507)。

---

### 5.4 SpeedController 内部ブロックリスト（Ts=1ms）

**入力ポート**

| # | 名称 | 型 | 上位接続元 |
|---|------|---|-----------|
| IN1 | Enable | bool | (top) root Enable |
| IN2 | SpeedRef_PU | scalar | (top) root SpeedRef_PU |
| IN3 | SpeedFb_PU | scalar | (top) Inverter.omega_m×60/(2π)/N_base via RT |
| IN4 | IqFb | scalar | (top) CurrCtrl.Iq_fb_PU via RT (未使用可) |
| IN5 | Mode | scalar | (top) Const 2（速度制御モード） |

**内部ブロック**

| ID | ブロック種別 | 入力元 | 出力先 | パラメータ / 式 |
|----|------------|--------|--------|---------------|
| S01 | `Sum` [+,−] | IN2, IN3 | S02, S03 | err = SpeedRef − SpeedFb |
| S02 | `Gain` | S01 | S04 | K = Kp_speed (0.78) → Vp |
| S03 | `Discrete-Time Integrator` | S01 (err), S07 (AW), IN1 (reset) | S04 | Gain = Ki_speed·Ts_speed (= 1.5e-3) |
| S04 | `Sum` [+,+] | S02, S03 | S05, S06 | IqRef_raw = Vp + Vi |
| S05 | `Saturation` | S04 | S06, OUT1[2] | ±0.20 PU → IqRef_sat |
| S06 | `Sum` [+,−] | S05, S04 | S07 | IqRef_sat − IqRef_raw |
| S07 | `Gain` | S06 | S03 (AWポート) | K = Kb_speed = 1/Kp_speed ≈ 1.28 |
| S08 | `Constant` 0 | — | OUT1[1] | IdRef_PU = 0 |
| S09 | `Rate Limiter` (optional soft ramp) | S05 | — or OUT1[2] | Rate = speed_ramp_rate×1000 ≈ 0.04 PU/ms |
| S10 | `Switch` (Enable) | IN1, S05, 0 | OUT1[2] | if !Enable: output 0 |

**出力ポート**

| # | 名称 | 型 | 上位接続先 |
|---|------|---|-----------|
| OUT1 | IdqRef_PU | vec[2] | (top) CurrCtrl.[IdRef, IqRef] via RT |
| OUT2 | SpeedRefOut_PU | scalar | (diag) Scope |
| OUT3 | EnClOut | bool | (diag) |

**備考**:
- 元 `FOCSpeedControl.slx` が存在すればそれを使用（State Chart によるオープンループ→クローズドループ遷移を含む）。
- 線形解析用には S01〜S07 の単純PI+AWだけで十分（State Chart は定常動作では透過）。

---

### 5.5 InverterPlant 内部ブロックリスト（continuous）

**入力ポート**

| # | 名称 | 型 | 上位接続元 |
|---|------|---|-----------|
| IN1 | Vabc_PU | vec[3] | (top) CurrCtrl.Vabc_PU via ZOH |
| IN2 | T_load | scalar (N·m) | (top) root T_load |

**内部ブロック**

| ID | ブロック種別 | 入力元 | 出力先 | パラメータ / 式 |
|----|------------|--------|--------|---------------|
| P01 | `Gain` | IN1 | P02 | K = V_dc/√3 (≈10.39) → Vabc_phys [V] |
| P02 | `Simscape PMSM` | P01 (stator voltage), IN2 (mech load) | P03, OUT2, OUT3 | Rs, Ld, Lq, FluxPM, P, J, B（§6.3 参照） |
| P03 | `Demux` | P02 (Iabc) | OUT1 | [Ia, Ib, Ic] → Ia, Ib を選択 |
| P04 | `Mux` [2] | P03 (Ia, Ib) | OUT1 | Iab_phys[2] |

**出力ポート**

| # | 名称 | 型 | 上位接続先 |
|---|------|---|-----------|
| OUT1 | Iab_phys | vec[2] | (top) ÷ISenseMax → CurrCtrl.Iab_PU via S+H |
| OUT2 | omega_m | scalar (rad/s) | (top) CurrCtrl.omega_m via S+H; HallModel.theta_m経由 |
| OUT3 | theta_m | scalar (rad) | (top) HallModel.theta_m |
| OUT4 | Te | scalar (N·m) | (diag) Scope |

**備考**:
- Simscape PMSM ブロック（Simscape Electrical > Specialized Power Systems > Fundamental Blocks > Machines > Permanent Magnet Synchronous Machine）を使用。
- Simscape 未使用時: Vd/Vq 状態方程式を `Integrator` × 4 + `MATLAB Function` で手動実装（§4.1 の式）。
- デッドタイム歪みを含めるなら、P01 の前に `MATLAB Function` でDTC歪みブロックを挿入（§4.2 レベル2）。

---

### 5.6 HallSensorModel 内部ブロックリスト（連続 or Ts=1μs）

**入力ポート**

| # | 名称 | 型 | 接続元 |
|---|------|---|--------|
| IN1 | theta_m | scalar (rad) | (top) Inverter.theta_m |

**内部ブロック**

| ID | ブロック種別 | 入力元 | 出力先 | パラメータ / 式 |
|----|------------|--------|--------|---------------|
| H01 | `Gain` | IN1 | H02 | K = pmsm.P (= 8) → theta_e_cont |
| H02 | `Math Function` (mod) | H01 | H03 | mod 2π |
| H03 | `MATLAB Function` quantize | H02 | OUT1 | θ_q = floor((θ−offset)/(π/3))·(π/3) + offset + π/6 |

**出力ポート**

| # | 名称 | 型 | 接続先 |
|---|------|---|--------|
| OUT1 | theta_hall | scalar (rad) | (top) CurrCtrl.theta_hall |

**備考**:
- 量子化1行の `MATLAB Function`:
```matlab
function th_q = hall_quantize(th_e, offset)
    step = pi/3;
    th_q = floor((th_e - offset) / step) * step + offset + step/2;
end
```

---

### 5.7 共通: Goto/From 推奨信号

全画面に線を引くのを避け、以下はGoto/Fromタグ化推奨：

| 信号 | Goto tag | 書込元 | 読出先 |
|------|---------|--------|--------|
| Enable | `ena` | root In Enable | SpeedCtrl.IN1, CurrCtrl.IN6 |
| theta_hall | `th_hall` | HallModel.OUT1 | CurrCtrl.IN4 |
| omega_m | `wm` | Inverter.OUT2 | CurrCtrl.IN5, SpeedFb換算 |
| Iq_fb_PU | `iq_fb` | CurrCtrl.OUT3 | SpeedCtrl.IN4 |

Tag visibility = **global**。

---

### 5.8 配線時チェックリスト

- [ ] すべてのサブシステム境界で `Rate Transition` または `Zero-Order Hold` / `Sample and Hold` が挿入されている
- [ ] Signals > Sample Time Display > Colors で各レートが色分け表示され、意図通り
- [ ] Signals > Port Data Types 表示で `single` または `double` 統一（コードはfloat=single）
- [ ] 信号次元: `Iab_PU`=[2], `Vabc_PU`=[3], `IdqRef_PU`=[2], それ以外はスカラ
- [ ] Display > Blocks > Sorted Execution Order で実行順: 速度ループ→電流ループ→プラント
- [ ] `Update Diagram` (Ctrl+D) エラーなし
- [ ] シミュレーション 1秒 完走、代数ループ警告なし

---

## 6. 実装手順

### 6.1 環境準備

1. **必要ツールボックス**:
   - Simulink
   - Simscape Electrical（PMSMブロック使用時）
   - Control System Toolbox（Bode/margin等）
   - Simulink Control Design（linearize、Linear Analysis Tool）

2. **作業フォルダ**: `matlab/simulink_model/` を新規作成

3. **ワークスペース変数ファイル**: `matlab/simulink_model/motor_params.m` を作成（後述§6.3）

### 6.2 構築順序（ボトムアップ推奨）

**Phase A: プラント単体検証**
1. PMSM モデル作成（Simscape版推奨）
2. Vd/Vq ステップ入力 → Id/Iq 応答が電気時定数 τe=525μs と一致
3. コマンド: `step`、`bode` でプラント特性確認

**Phase B: 電流ループ単独評価**
4. CurrentController 構築（§4.4 の順）
5. 上位速度ループなし、IqRef を直接入力
6. Step応答: IqRef=0→0.1 PU、Iq_fb の立上り測定
7. 実機ログ（M2000 の motor_log_20260329_232130.csv 等）と重ね合わせ

**Phase C: 速度ループ接続**
8. SpeedController 構築（§4.5）
9. SpeedRef ステップ→RPM 応答
10. 実機データと比較（M500/M1000/M2000）

**Phase D: 安定性評価**（§7）

### 6.3 ワークスペース変数設定

`matlab/simulink_model/motor_params.m`:

```matlab
% === Motor (maxon EC flat 651614) ===
pmsm.Rs     = 0.2865;       % phase resistance [Ω]
pmsm.Ld     = 0.1505e-3;    % phase inductance [H]
pmsm.Lq     = 0.1505e-3;    % same as Ld (surface magnet)
pmsm.FluxPM = 0.002915;     % PM flux linkage [Wb]
pmsm.P      = 8;            % pole pairs
pmsm.J      = 1.81e-5;      % rotor inertia [kg·m²]
pmsm.B      = 0.001;        % friction [N·m·s/rad]
pmsm.I_rated = 3.29;        % rated current [A]

% === Inverter ===
inv.V_dc       = 18.0;      % [V] Makita BL1820B
inv.ISenseMax  = 16.5;      % current base [A]
inv.f_pwm      = 10e3;      % PWM frequency [Hz] (update to 18e3 after migration)
inv.T_pwm      = 1/inv.f_pwm;
inv.t_dead     = 500e-9;    % dead time [s]
inv.dtc_comp   = 2*inv.t_dead/inv.T_pwm;  % 0.01 @10kHz, 0.018 @18kHz

% === PU bases ===
V_base   = inv.V_dc / sqrt(3);     % 10.39 V
I_base   = inv.ISenseMax;          % 16.5 A
N_base   = 4750;                   % RPM
omega_base_elec = 2*pi*N_base/60 * pmsm.P;  % rad/s elec

% === Current loop (10kHz) ===
Ts_curr = 100e-6;           % update to 55.56e-6 for 18kHz
PI.Kp_id = 0.30;
PI.Ki_id = 0.00;
PI.Kp_iq = 0.50;
PI.Ki_iq = 0.10;
PI.int_limit = 0.50;
PI.Kb_id = 1/max(PI.Kp_id, 0.01);
PI.Kb_iq = 1/max(PI.Kp_iq, 0.01);

% === Speed loop (1kHz) ===
Ts_speed = 1e-3;
Sp.Kp = 0.78;
Sp.Ki = 1.5;
Sp.IqRef_max = 0.20;
Sp.Kb = 1/Sp.Kp;

% === Hall / Angle ===
hall_offset     = 0.20;     % [rad]
angle_advance_us = 200;     % [μs]

% === Filters ===
alpha_iq_lpf     = 0.10;    % fc ≈ 1592*alpha/Ts Hz ≈ 159Hz @10kHz
alpha_decouple   = 0.006;   % fc ≈ 9.5Hz @10kHz
speed_ramp_rate  = 0.00004; % PU/call @10kHz

% === Decoupling PU coefficients (match code §4.4) ===
Lq_pu     = pmsm.Lq * inv.ISenseMax * sqrt(3) / inv.V_dc;   % 0.000239
Ld_pu     = pmsm.Ld * inv.ISenseMax * sqrt(3) / inv.V_dc;
FluxPM_pu = pmsm.FluxPM * sqrt(3) / inv.V_dc;               % 0.000281
```

**注**: モデル初期化スクリプトで `run motor_params.m` を呼ぶ or PreLoadFcn に登録。

---

## 7. 安定性評価手順

### 7.1 評価シナリオ

| # | シナリオ | 目的 | 評価指標 |
|---|---------|------|---------|
| 1 | 電流ループ単体、無負荷、0 RPM | 基本帯域確認 | Bode帯域、Phase margin |
| 2 | 電流ループ、M2000 定常 | 高速域の安定余裕 | 同上 |
| 3 | 速度ループ閉、M1000 定常、無負荷 | 外乱応答 | Sensitivity S |
| 4 | 速度ループ閉、負荷急変 | 過渡応答 | Settling time, overshoot |
| 5 | 電圧飽和時 | AW動作確認 | 非線形シミュレーションのみ |

### 7.2 線形化による解析（Phase D）

**Linear Analysis Tool 使用手順**:

1. モデル上で `Analysis > Linear Analysis Points > Open Linear Analysis Tool`
2. 開ループ評価点を設定：
   - **電流ループ開ループ**: IqRef入力に `Input Perturbation`、Iq_fb出力に `Output Measurement`、ループを開く点として `Loop Opening` を Iq_fb→Error の経路に
   - **速度ループ開ループ**: 同様に SpeedRef と SpeedFb_PU で
3. **Operating Point** を設定：
   - Model Initial Condition、または
   - Steady-State Manager で Trim（M1000 で定常トリム）
4. **Linearize** 実行 → `linsys1` 生成
5. **Bode, Margin, Nichols** プロット：
   ```matlab
   bode(linsys1); grid
   [Gm, Pm, Wgm, Wpm] = margin(linsys1)
   ```

**評価基準**:

| 指標 | 電流ループ | 速度ループ |
|------|-----------|-----------|
| Phase Margin | > 45° | > 45° |
| Gain Margin | > 6 dB | > 6 dB |
| 帯域 (0dB crossover) | 100~200 Hz | 10~50 Hz |
| ピーク sensitivity \|S\| | < 6 dB (2倍) | < 6 dB |

### 7.3 外乱応答シミュレーション

**構成**: Model Input = T_load（ステップ）、Output = SpeedFb_RPM

```matlab
% 負荷急変シミュレーション
simOut = sim('top', 'StopTime', '2.0');
t = simOut.tout;
rpm = simOut.SpeedFb_RPM;
plot(t, rpm); grid on
```

**評価**:
- 0.75kg 負荷印加時の RPM ドロップと復帰時間（実機: 5.5kg で M500 安定）
- 電流ピーク値

### 7.4 PWM周波数変更の影響予測

モデルで `inv.f_pwm`、`Ts_curr` を切替えて比較：

| 周波数 | Ts_curr | 期待される変化 |
|--------|---------|--------------|
| 10kHz | 100μs | 現状ベースライン |
| 18kHz | 55.56μs | 電流ループ帯域↑、PM若干悪化の可能性 |
| 20kHz | 50μs | 同上、更に顕著 |

Ki の自動スケーリング（`PI.Ki_iq_scaled = PI.Ki_iq` のまま、Ts が式に入る離散形式を前提）。

---

## 8. 実機データとの突合

### 8.1 校正（モデル信頼性確保）

実機ログ CSV（例: `matlab/motor_log_20260329_232130.csv` M2000 angle_advance=200μs）を読込み、同条件でシミュレーション：

```matlab
logD = readtable('../motor_log_20260329_232130.csv');
sim('top');
figure
subplot(2,1,1); plot(logD.Time_s, logD.Iq_PU, simOut.tout, simOut.Iq_fb_PU); 
legend('実機','モデル'); ylabel('Iq [PU]');
subplot(2,1,2); plot(logD.Time_s, logD.RPM, simOut.tout, simOut.SpeedFb_RPM); 
legend('実機','モデル'); ylabel('RPM');
```

**許容誤差**:
- 定常値: ±10%
- 立上り時間: ±20%
- リップル: 定性的に振幅オーダー一致

乖離が大きい場合、原因探索：
- 摩擦係数 B が推定値 → 実機負荷試験データで同定
- デッドタイム歪み（レベル1モデルでは無視）
- Hall センサの量子化ノイズ

### 8.2 パラメータ同定（補正）

未知パラメータを同定：
1. **B（粘性摩擦）**: 無負荷 M500/M1000/M2000 の定常 Iq からトルクバランスで推定
2. **Coulomb摩擦**: 起動トルクから推定（モデルに追加）
3. **Lq/Ld 違い**: 突極性あれば Id→Iq 干渉で顕著

---

## 9. 成果物とコミット

| 成果物 | 配置 | 備考 |
|--------|-----|------|
| `top.slx` | `matlab/simulink_model/` | メインモデル |
| `motor_params.m` | 同上 | ワークスペース変数 |
| `analyze_stability.m` | 同上 | Bode/margin自動化スクリプト |
| `compare_with_log.m` | 同上 | 実機ログ突合スクリプト |
| `doc/Simulinkモデル解析結果.md` | `doc/` | Bode図と余裕度まとめ |

**コミット単位**:
- Phase A 完了時 (PMSM単体検証) → "Add Simulink PMSM plant model and validation script"
- Phase B 完了時 → "Add current loop model, matches code behavior"
- Phase C 完了時 → "Add speed loop and full closed-loop simulation"
- Phase D 完了時 → "Add linearization + stability analysis"

---

## 10. チェックリスト

### モデル構築
- [ ] `motor_params.m` 作成、パラメータ値がコードと一致
- [ ] PMSM モデル単体で Id ステップ応答が τe=525μs と一致
- [ ] Clarke/Park 変換、実機コードと符号一致（逆転しない）
- [ ] Hall 量子化 + angle_advance 実装
- [ ] 電流 PI のback-calc AW が実機と等価
- [ ] Circle limiter（|V|<0.95）実装
- [ ] SVPWM min-max 3rd harmonic injection 実装
- [ ] 速度ループ（FOCSpeedControl 相当）実装
- [ ] Rate Transition 全箇所に挿入

### 検証
- [ ] IqRef ステップでモデル Iq 応答、実機と定常値一致
- [ ] M500/M1000/M2000 シミュレーション、RPM 定常誤差<5%
- [ ] 実機ログと重ね合わせ、波形オーダー一致

### 安定性評価
- [ ] 電流ループ Bode、Phase Margin > 45°
- [ ] 電流ループ帯域 ≈ Kp_iq/(Lq/(V_base/I_base) × 2π) ≈ 151Hz と一致
- [ ] 速度ループ Bode、Phase Margin > 45°
- [ ] Sensitivity |S| < 6dB
- [ ] 外乱応答：負荷ステップで RPM 復帰 < 500ms

### ドキュメント
- [ ] `doc/Simulinkモデル解析結果.md` に Bode 図と余裕度を記録
- [ ] `doc/ソフト管理表.md` に Simulinkモデルのバージョンとコミット記録

---

## 11. 参考・参考資料

### コードからSimulinkブロックへの対応表

| コード場所 | Simulinkブロック |
|-----------|---------------|
| [hal_entry.c:1305](src/hal_entry.c#L1305) `hall_angle += omega_e_filt × angle_advance_us × 1e-6` | Gain(`angle_advance_us*1e-6`) + Sum |
| [hal_entry.c:1309-1310](src/hal_entry.c#L1309) `sin_theta/cos_theta` | Trigonometric Function × 2 |
| [hal_entry.c:1314-1319](src/hal_entry.c#L1314) Clarke/Park | `MATLAB Function` 推奨 |
| [hal_entry.c:1361-1362](src/hal_entry.c#L1361) `α=0.1 LPF` | Discrete Filter `1-z⁻¹`ベース |
| [hal_entry.c:1430-1435](src/hal_entry.c#L1430) decoupling ω_e×L×I | `MATLAB Function` |
| [hal_entry.c:1448-1478](src/hal_entry.c#L1448) PI + AW | Discrete Integrator + Saturation + Kb path |
| [hal_entry.c:1489-1505](src/hal_entry.c#L1489) 実電流リミッター | `MATLAB Function` |
| [hal_entry.c:1507-1520](src/hal_entry.c#L1507) inv Park + SVPWM | `MATLAB Function` |

### 関連ドキュメント

- [モーター設計書.md](doc/モーター設計書.md) — 物理パラメータ、PU正規化、電気時定数
- [ソフト管理表.md](doc/ソフト管理表.md) — 実機ゲイン値、既知課題
- [PWM18kHz化手順書.md](doc/PWM18kHz化手順書.md) — Ts変更時の影響表
- [既存実装分析.md](doc/既存実装分析.md) — 実機ブロック構成詳細
