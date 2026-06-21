/**
 * @文件 atmosphere.c
 * @简介 用于空气密度和声速的ICAO标准大气模型。
 *
 * 本模块使用国际民用航空组织（ICAO）
 * 标准大气方程计算给定海拔和温度下的大气特性。
 *
 * 参考文献：
 *   - ICAO Doc 7488（标准大气）
 *   - 美国标准大气，1976
 *
 * 模型假设：
 *   - 海平面气压 P0 = 101325 Pa（1 atm）
 *   - 海平面温度 T0 = 288.15 K（15℃）
 *   - 温度递减率 L = 0.0065 K/m（对流层，有效至约11 km）
 *   - 流体静力学平衡结合理想气体定律
 *   - 恒定重力加速度 g = 9.80665 m/s^2
 *   - 干燥空气摩尔质量 M = 0.0289644 kg/mol
 *   - 通用气体常数 R = 8.3144598 J/(mol*K)
 *   - 干燥空气比气体常数 Rs = R/M = 287.05 J/(kg*K)
 *   - 比热比 gamma = 1.4（双原子理想气体）
 */

#include "atmosphere.h"
#include <math.h>

/** 海平面标准气压（Pa） */
#define P0         101325.0f
/** 海平面标准温度（K）= 15℃ */
#define T0_STD     288.15f
/** 对流层温度递减率（K/m） */
#define L          0.0065f
/** 干燥空气摩尔质量（kg/mol） */
#define M_AIR      0.0289644f
/** 通用气体常数（J/(mol*K)） */
#define R_GAS      8.3144598f
/** 干燥空气比气体常数：R / M（J/(kg*K)） */
#define R_SPECIFIC 287.05f
/** 标准重力加速度（m/s^2） */
#define G_STD      9.80665f
/** 干燥空气比热比（Cp/Cv） */
#define GAMMA      1.4f

/**
 * @简介 计算给定海拔和温度下的空气密度。
 *
 * 使用ICAO对流层模型：
 *   T(h) = T0 - L * h                                  （给定海拔的温度）
 *   P(h) = P0 * (1 - L*h / T0)^(g*M / (R*L))           （气压高度公式）
 *   rho = P / (Rs * T_实际)                              （理想气体定律）
 *
 * 气压高度公式指数 g*M/(R*L) = 5.25588 在运行时预计算以便于理解。
 * 给定海拔的温度由标准递减率导出，但密度计算使用用户提供的温度
 * （允许非标准天气条件）。超过对流层边界时，气压被钳制为零。
 *
 * @参数 altitude_m    平均海平面以上的几何海拔（m）。
 * @参数 temperature_c 站点实际空气温度（摄氏度）。
 * @返回 空气密度，单位为 kg/m^3。
 */
float atm_density(float altitude_m, float temperature_c) {
    float T_K = temperature_c + 273.15f; /**< 将摄氏度转换为开尔文 */
    float exponent = G_STD * M_AIR / (R_GAS * L); /**< g*M/(R*L) ≈ 5.25588 */
    float pressure = P0 * powf(1.0f - L * altitude_m / T0_STD, exponent); /**< 气压高度公式 */
    if (pressure < 0.0f) pressure = 0.0f; /**< 超出模型有效范围的海拔将气压钳制为零 */
    return pressure / (R_SPECIFIC * T_K); /**< 理想气体定律：rho = P/(Rs*T) */
}

/**
 * @简介 计算给定温度下空气的声速。
 *
 * 对于理想双原子气体：
 *   c = sqrt(gamma * Rs * T)
 *
 * 其中 gamma = 1.4，Rs = 287.05 J/(kg*K)，T为绝对温度。
 * 与气压/密度无关（理想气体假设）。
 *
 * @参数 temperature_c 空气温度（摄氏度）。
 * @返回 声速，单位为 m/s。
 */
float atm_sound_speed(float temperature_c) {
    float T_K = temperature_c + 273.15f; /**< 将摄氏度转换为开尔文 */
    return sqrtf(GAMMA * R_SPECIFIC * T_K); /**< c = sqrt(1.4 * 287.05 * T) */
}
