# Pico 2 Artillery Ballistic Calculator

Raspberry Pi Pico 2 (RP2350) 双核弹道计算器，以一战德军 420mm "大贝尔莎"攻城榴弹炮为原型，通过 4×4 矩阵键盘输入目标参数，LCD1602 显示计算结果，Core 1 专责执行 RK4 弹道积分与寻根算法。

---

## 目录

- [1. 程序结构与功能](#1-程序结构与功能)
- [2. 第三方库算法结构](#2-第三方库算法结构)
- [3. 自定义火炮型号](#3-自定义火炮型号)
- [4. 程序使用说明](#4-程序使用说明)
- [5. PC 端离线调用与验证](#5-pc-端离线调用与验证)
- [6. 硬件接线](#6-硬件接线)
- [7. 构建说明](#7-构建说明)
- [8. 验证数据](#8-验证数据)
- [9. 参考文献](#9-参考文献)

---

## 1. 程序结构与功能

```
jsqpico2/
├── CMakeLists.txt              CMake 构建配置
├── main.c                      Core 0 入口，键盘扫描 + 状态机驱动
├── pico_sdk_import.cmake       Pico SDK 导入脚本
├── ui/
│   ├── lcd1602_i2c.c/h         LCD1602 I²C 驱动（第三方库）
│   ├── pico_keypad4x4.c/h      4×4 矩阵键盘驱动（第三方库）
│   └── menu.c/h                多页 UI 状态机，字段缓冲区
├── ballistics/
│   ├── gun_data.c/h            火炮装药/速度/质量查找表
│   ├── atmosphere.c/h          ICAO 标准大气模型
│   ├── drag.c/h                Cd(Mach) 分段线性插值
│   ├── integrator.c/h          RK4 四阶龙格-库塔积分器
│   └── solver.c/h              扫描+二分法寻根，装药循环
├── core1/
│   └── core1_task.c/h          SDK FIFO 双核通信协议
└── tests/
    └── simtest.c               PC 端离线模拟验证程序
```

### 1.1 双核分工

| 核心 | 职责 |
|---|---|
| **Core 0** | 键盘扫描与消抖、LCD1602 I²C 驱动、UI 状态机、结果格式化显示 |
| **Core 1** | 纯数值计算：大气模型、RK4 弹道积分、扫描+二分法寻根、装药等级循环 |

核间通信基于 Pico SDK `multicore_fifo`：Core 0 推送 `solve_request_t` → Core 1 计算 → 推送 `solver_result_t` 返回。

### 1.2 弹道物理模型

- **大气模型**：ICAO 标准气压高公式 + 理想气体状态方程，输入阵地海拔与现场温度，输出空气密度与声速
- **阻力模型**：Cd(Mach) 查找表，10 个节点分段线性插值，覆盖亚音速至 M=3.0
- **积分器**：固定步长 0.01s 的 RK4，while 循环直至弹丸降落到目标高度
- **求解器**：扫描 5°/1° 两种步长定位 θ_max ± 二分法在严格单调区间内收敛
- **最低装药优先策略**：从装药 1 到 5 依次尝试，返回首个可达目标的最低装药等级

### 1.3 UI 流程

```
SCENARIO(1-5) → CHG MODE(AUTO/1-5) → AMMO(HE/AP) → 7参数 → CONFIRM → 计算 → 结果
```

5 种预设场景：

| # | 场景 | 仰角分支 | 推荐弹种 |
|---|---|---|---|
| 1 | 平坦地形直射 | 低伸 (0–42°) | HE |
| 2 | 目标位于高地 | 自动尝试双分支 | AP |
| 3 | 目标位于洼地 | 低伸 | HE |
| 4 | 远程轰炸 | 低伸 | HE |
| 5 | 近距高抛 | 高抛 (43–65°) | HE |

---

## 2. 第三方库算法结构

### 2.1 lcd1602_i2c — LCD1602 I²C 驱动

```
接口:  lcd1602_init()              初始化 I²C 与 LCD
       lcd1602_clear()             清屏
       lcd1602_set_cursor(line,pos) 设置光标 (line=0/1, pos=0–15)
       lcd1602_string("text")      打印字符串
       lcd1602_char('A')           打印单个字符

原理:  4 位模式 I²C 通信，PCF8574 背板。每次传输高 4 位 + 低 4 位，
       通过拉高/拉低 EN 位产生使能脉冲。支持背光控制（LCD_BACKLIGHT 宏）。
       
配置:  LCD1602_SDA_PIN     (默认 4)
       LCD1602_SCL_PIN     (默认 5)
       LCD1602_I2C_ADDR    (默认 0x27)
       LCD1602_I2C_BAUD    (默认 100000)
       可在编译时通过 -D 覆盖，无需修改源码。
```

### 2.2 pico_keypad4x4 — 4×4 矩阵键盘

```
接口:  pico_keypad_init(columns[4], rows[4], matrix[16])
           初始化 GPIO，列=输入，行=输出。matrix 为 16 字符的键值映射。
       pico_keypad_get_key()
           行扫描法读取按键，返回 '\0' 或无按键时返回 0。
           内置消抖：连续两次读到相同键才确认。

原理:  4 行 GPIO 依次拉高，读取 4 列 GPIO 状态。被按下的 (行,列) 交叉点
       对应 matrix 数组中的字符。行列引脚号在调用侧指定。

映射:  matrix[16] = {'1','2','3','A', '4','5','6','B',
                     '7','8','9','C', '*','0','#','D'}
```

### 2.3 ballistics — 弹道数学库

模块间依赖关系：

```
solver.c
  ├── gun_data.c      装药速度表 + 弹丸质量
  ├── atmosphere.c    空气密度 + 声速
  └── integrator.c    RK4 轨迹积分
        └── drag.c     Cd(Mach) 查找表
```

---

## 3. 自定义火炮型号

只需修改 `ballistics/gun_data.h` 和 `ballistics/gun_data.c`，其余模块无需改动。

### 3.1 修改火炮参数 (`gun_data.h`)

```c
#define CHARGE_LEVELS 5          // 装药等级数量
#define PROJECTILE_HE 0          // 高爆弹索引
#define PROJECTILE_AP 1          // 穿甲弹索引

#define MASS_HE   819.0f         // HE 弹质量 (kg)
#define MASS_AP   1000.0f        // AP 弹质量 (kg)
#define CALIBER_RADIUS 0.21f     // 口径半径 (m)，420mm/2
#define PROJECTILE_AREA 0.138544f // 横截面积 π·r²
#define MAX_ELEVATION_DEG 65.0f  // 最大仰角 (度)
```

### 3.2 修改装药初速表 (`gun_data.c`)

```c
// 数组索引 = 装药等级-1，值 = 炮口初速 (m/s)
const float muzzle_velocity_he[CHARGE_LEVELS] = {
    253.0f,  // 装药 1 (40%)
    297.0f,  // 装药 2 (55%)
    335.0f,  // 装药 3 (70%)
    369.0f,  // 装药 4 (85%)
    400.0f   // 装药 5 (100%)
};

const float muzzle_velocity_ap[CHARGE_LEVELS] = {
    229.0f, 269.0f, 303.0f, 334.0f, 362.0f
};
```

### 3.3 添加新弹种

1. 在 `gun_data.h` 中添加 `#define PROJECTILE_NEW 2`
2. 在 `gun_data.c` 中添加对应的初速数组和 `gun_mass()` 分支
3. 在 `ui/menu.c` 的 `FIELD_AMMO` 页面添加新选项

### 3.4 调整阻力模型 (`drag.c`)

```c
static const float cd_table[][2] = {
    {0.3f, 0.65f},  // {马赫数, Cd}
    {0.7f, 0.70f},
    {0.9f, 0.90f},
    {1.0f, 1.15f},  // 跨音速峰值
    ...
};
```

Cd 表基于 G1 标准阻力函数乘以形状系数。流线型弹可用更低的 Cd，钝弹需更高的 Cd。节点间线性插值。

### 3.5 修改建议

- 修改 `gun_data.c` 后运行 `tests/simtest.exe` 验证新数据
- 可添加火炮名称字符串供 LCD 启动屏显示
- 若 CHARGE_LEVELS ≠ 5，需同步修改 `ui/menu.c` 中的装药选择逻辑

---

## 4. 程序使用说明

### 4.1 键盘映射

| 键 | 功能 |
|---|---|
| `0–9` | 数字输入 |
| `*` | 切换当前字段正负号（Δh、温度） |
| `#` | 小数点 |
| `A` | 确认当前字段 / 进入下一字段 / 装药模式确认 |
| `B` | 返回 / 编辑上一个字段 |
| `C` | 清除当前输入 |
| `D` | 跳到确认屏幕 / 开始计算 |

### 4.2 LCD 页面序列

| 步骤 | 第 1 行 | 第 2 行示例 | 按键 |
|---|---|---|---|
| 1 | `SCENARIO 1-5:` | `1` | `1` `A` |
| 2 | `CHG MODE:` | `AUTO` | `A`(自动) 或 `3` `A`(手动装药3) |
| 3 | `AMMO 1=HE 2=AP` | `1` | `A` |
| 4 | `DIST(m):` | `5000` | `5000` `A` |
| 5 | `BRG(deg,0-359):` | `90` | `90` `A` |
| 6 | `ALT ASL(m):` | `0` | `0` `A` |
| 7 | `TGT dH(m):` | `0` | `0` `A` |
| 8 | `TEMP(C):` | `15` | `15` `A` |
| 9 | `WIND SPD(m/s):` | `0` | `0` `A` |
| 10 | `WIND DIR(deg):` | `0` | `0` `D` |
| 11 | `D=CALC B=EDIT` | `A=REVIEW` | `D` |
| 12 | `ELEV:24.1 DEG` | `CHG:2 HE AUTO` | 任意键返回 |

### 4.3 装药模式

| 操作 | 显示 | 效果 |
|---|---|---|
| 首次进入 | `CHG MODE:` / `AUTO` | 默认自动 |
| 按 `A` | → 前进 | 自动选装药 |
| 按 `3` | `CHG MODE:` / `MANUAL C3` | 预选手动装药3 |
| 再按 `A` | → 前进 | 确认手动 |
| 按 `C` | `CHG MODE:` / `AUTO` | 恢复自动 |
| 按 `B` | → 返回场景 | 回退 |

### 4.4 结果解读

- **命中**：`ELEV: 24.1°` / `CHG: 2 HE AUTO` — 炮管仰角 + 推荐装药等级
- **射程外**：`OUT OF RANGE` / `MAX: 8394m C5` — 当前条件下最大射程
- **手动失败**：`OUT OF RANGE` — 指定装药不可达，尝试更高装药或切回 AUTO

---

## 5. PC 端离线调用与验证

`ballistics/` 目录下的代码为 **纯 C**，无任何 Pico SDK 依赖，可在 PC 上直接用 gcc 编译。

### 5.1 编译运行测试程序

```bash
# 进入测试目录
cd tests

# 编译
gcc -std=c11 -O2 -Wall -o simtest \
    simtest.c \
    ../ballistics/atmosphere.c \
    ../ballistics/drag.c \
    ../ballistics/gun_data.c \
    ../ballistics/integrator.c \
    ../ballistics/solver.c \
    -I.. -lm

# 运行
./simtest
```

### 5.2 在自己的 C 程序中调用

```c
#include "ballistics/solver.h"
#include "ballistics/gun_data.h"

int main() {
    // 自动选装药
    solver_result_t r = solve_ballistics(
        5000,     // range_m:        目标距离
        0,        // delta_h_m:      目标高度差
        0,        // altitude_m:     阵地海拔
        15,       // temperature_c:  现场温度
        0, 0, 90, // wind_spd, wind_dir, bearing
        0,        // projectile:     0=HE, 1=AP
        0);       // branch:         0=LOW, 1=HIGH

    printf("CHG=%d ELEV=%.1f° %s\n",
           r.charge_number, r.elevation_deg,
           r.reachable ? "HIT" : "MISS");

    // 手动指定装药
    r = solve_ballistics_fixed_charge(
        5000, 0, 0, 15, 0, 0, 90, 0, 0, 3);
    //                                  ↑ charge=3
}
```

### 5.3 接口速查

```c
// atmosphere.h
float atm_density(float altitude_m, float temperature_c);
float atm_sound_speed(float temperature_c);

// integrator.h
float rk4_trajectory(float elevation_rad, float v0,
    float mass, float area,
    float rho, float sound_speed,
    float wx, float delta_h, bool *reached);

// solver.h
solver_result_t solve_ballistics(
    float range_m, float delta_h_m,
    float altitude_m, float temperature_c,
    float wind_speed_ms, float wind_dir_deg,
    float bearing_deg, uint8_t projectile,
    uint8_t branch);

solver_result_t solve_ballistics_fixed_charge(
    float range_m, float delta_h_m,
    float altitude_m, float temperature_c,
    float wind_speed_ms, float wind_dir_deg,
    float bearing_deg, uint8_t projectile,
    uint8_t branch, uint8_t charge);

// gun_data.h
float gun_mass(uint8_t projectile);
float gun_muzzle_velocity(uint8_t charge, uint8_t projectile);
```

### 5.4 集成到其他项目（CMake）

```cmake
# 方式一：直接复制 ballistics/ 到新项目
add_executable(my_app main.c ballistics/atmosphere.c ...)
target_include_directories(my_app PRIVATE . ballistics)

# 方式二：引用外部目录
add_subdirectory(../jsqpico2/ballistics ${CMAKE_BINARY_DIR}/ballib)
target_link_libraries(my_app ballib)
```

---

## 6. 硬件接线

| 信号 | Pico 2 GPIO | 说明 |
|---|---|---|
| I2C0 SDA | **GP4** | LCD1602 数据线 |
| I2C0 SCL | **GP5** | LCD1602 时钟线 |
| LCD VCC | VBUS (5V) | PCF8574 背板需 5V |
| LCD GND | GND | |
| 键盘行 1–4 | **GP10, GP11, GP12, GP13** | 输出，扫描时依次拉高 |
| 键盘列 1–4 | **GP18, GP19, GP20, GP21** | 输入，启用内部上拉 |

> PCF8574 背板推荐外接 4.7kΩ 上拉电阻至 5V。键盘列依赖 GPIO 内部上拉，无需外接电阻。

---

## 7. 构建说明

### 前置条件

- Raspberry Pi Pico C/C++ SDK 2.2.0+
- Arm GNU Toolchain 14.2+
- CMake 3.13+ / Ninja
- VS Code + Raspberry Pi Pico 扩展（推荐）

### 构建

```bash
mkdir build && cd build
cmake .. -G Ninja
ninja
```

产物：`build/jsqpico2.uf2` — 按住 Pico 2 的 BOOTSEL 按钮上电，拖入 UF2 文件即可烧录。

---

## 8. 验证数据

以下结果由 `tests/simtest.c` 在 PC 端生成，与 Pico 硬件输出一致。

```
=== Ballistics Solver Verification ===

Atmosphere: rho(0,15)=1.2250  sound(15)=340.3
Atmosphere: rho(500,20)=1.1344  sound(20)=343.2
Atmosphere: rho(0,-20)=1.3944  sound(-20)=319.0

--- Standard Tests ---
Test 1 flat        CHG=2  ELEV=24.1  ✓
Test 2 uphill AP   CHG=2  ELEV=28.8  ✓
Test 3 downhill    CHG=1  ELEV=10.4  ✓
Test 4 long        CHG=5  ELEV=31.6  ✓
Test 5 high ang    CHG=1  ELEV=65.0  ✓
Test 6 out range   OUT OF RANGE  ✓
Test 7 cold        CHG=1  ELEV=16.8  ✓

--- Max Range Sweep (HE, flat, sea level, 15C) ---
  CHG 1 (253 m/s): max_range=4877m at 43 deg
  CHG 2 (297 m/s): max_range=6077m at 42 deg
  CHG 3 (335 m/s): max_range=6988m at 42 deg
  CHG 4 (369 m/s): max_range=7711m at 42 deg
  CHG 5 (400 m/s): max_range=8394m at 41 deg

--- Fixed Charge Tests (5000m, HE, flat) ---
  CHG 1 FIXED: OUT OF RANGE
  CHG 2 FIXED: ELEV=24.1
  CHG 3 FIXED: ELEV=18.7
  CHG 4 FIXED: ELEV=15.7
  CHG 5 FIXED: ELEV=13.3

--- Wind Tests (5000m, HE, flat, 10m/s) ---
  Headwind (10m/s): CHG=2 ELEV=25.1  (逆风→射程缩短)
  Tailwind (10m/s): CHG=1 ELEV=43.0  (顺风→射程增加)
  No wind:          CHG=2 ELEV=24.1
```

### 测试用例参数表

| # | 场景 | 装药 | 弹 | 距离 | 方位 | 海拔 | dH | 温度 | 风 | 期望 |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 1 | A | HE | 5000 | 90 | 0 | 0 | 15 | 0 | **CHG 2 · 24°** |
| 2 | 2 | A | AP | 4000 | 45 | 200 | +500 | 5 | 0 | **CHG 2 · 29°** |
| 3 | 3 | A | HE | 3500 | 180 | 500 | -500 | 20 | 0 | **CHG 1 · 10°** |
| 4 | 4 | A | HE | 8000 | 0 | 0 | 0 | 15 | 0 | **CHG 5 · 32°** |
| 5 | 5 | A | HE | 1500 | 270 | 100 | 0 | 25 | 0 | **CHG 1 · 65°** |
| 6 | 4 | A | HE | 12000 | 0 | 0 | 0 | 15 | 0 | **OUT OF RANGE** |
| 7 | 1 | A | HE | 3000 | 0 | 0 | 0 | -20 | 0 | **CHG 1 · 17°** |

> 手动装药测试：CHG 1→OUT, CHG 2→24.1°, CHG 3→18.7°, CHG 4→15.7°, CHG 5→13.3°

---

## 9. 参考文献

- [Big Bertha (howitzer) — Wikipedia](https://en.wikipedia.org/wiki/Big_Bertha_(howitzer))
- [42cm kurze Marinekanone 14 L/12 — historyofwar.org](https://www.historyofwar.org/articles/weapons_42cm_big_bertha.html)
- ICAO Standard Atmosphere (1993)
- G1 Drag Function — 外弹道学标准阻力模型
- [Raspberry Pi Pico C/C++ SDK](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf)
- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
- Romanych & Rupp (2013) — *42cm "Big Bertha" and German Siege Artillery of World War I*, Osprey Publishing

---

## 许可

本项目基于 Raspberry Pi Pico SDK 构建。`pico_keypad4x4` 与 `lcd1602_i2c` 为第三方开源库。弹道计算代码仅供数值方法演示与历史研究，不可作为实际火力控制仪器使用。

GPL v3 License