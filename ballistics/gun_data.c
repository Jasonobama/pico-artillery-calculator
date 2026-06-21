/**
 * @文件 gun_data.c
 * @简介 42cm大贝莎（Dicke Bertha）榴弹炮的装药相关初速和弹丸质量表。
 *
 * 大贝莎（M-Gerat）是德军在第一次世界大战中使用的420mm攻城榴弹炮。
 * 它使用药包式发射药（装药1至5）来改变初速和射程。
 * 使用两种弹丸类型：
 *
 *   - HE（高爆弹）：819 kg弹丸，较大装药量，最大装药时
 *     末速约400 m/s。
 *   - AP（穿甲弹）：1000 kg弹丸，更重的穿甲头，由于质量更大
 *     初速较低。
 *
 * 装药等级：
 *   | 装药 | HE初速 | AP初速 |
 *   |------|--------|--------|
 *   | C1   | 253 m/s| 229 m/s|
 *   | C2   | 297 m/s| 269 m/s|
 *   | C3   | 335 m/s| 303 m/s|
 *   | C4   | 369 m/s| 334 m/s|
 *   | C5   | 400 m/s| 362 m/s|
 *
 * 弹丸物理参数（来自gun_data.h）：
 *   - 口径半径：0.21 m（420 mm / 2）
 *   - 截面积：pi * r^2 = 0.138544 m^2
 *   - HE质量：819 kg
 *   - AP质量：1000 kg
 */

#include "gun_data.h"

/**
 * @简介 各装药等级HE弹丸的初速表。
 *
 * 索引0 = 装药1，...，索引4 = 装药5。
 * 数值单位为m/s，来源于历史射表。
 */
const float muzzle_velocity_he[CHARGE_LEVELS] = {
    253.0f,  /**< 装药1：最低初速 */
    297.0f,  /**< 装药2 */
    335.0f,  /**< 装药3 */
    369.0f,  /**< 装药4 */
    400.0f   /**< 装药5：最大装药 */
};

/**
 * @简介 各装药等级AP弹丸的初速表。
 *
 * AP弹比HE弹重181 kg，因此相同发射药装药下初速较低。
 * 索引0 = 装药1，...，索引4 = 装药5。
 */
const float muzzle_velocity_ap[CHARGE_LEVELS] = {
    229.0f,  /**< 装药1 */
    269.0f,  /**< 装药2 */
    303.0f,  /**< 装药3 */
    334.0f,  /**< 装药4 */
    362.0f   /**< 装药5 */
};

/**
 * @简介 获取给定类型的弹丸质量。
 *
 * @参数 projectile PROJECTILE_HE（0）或PROJECTILE_AP（1）。
 * @返回 弹丸质量，单位为kg（819或1000）。
 */
float gun_mass(uint8_t projectile) {
    return (projectile == PROJECTILE_HE) ? MASS_HE : MASS_AP; /**< 819 kg HE，1000 kg AP */
}

/**
 * @简介 获取特定装药等级和弹丸类型的初速。
 *
 * 将装药编号（从1开始，1-5）映射到静态查找表中相应的初速。
 * 对无效装药值返回0。
 *
 * @参数 charge     装药等级（1到CHARGE_LEVELS=5）。
 * @参数 projectile PROJECTILE_HE（0）或PROJECTILE_AP（1）。
 * @返回 初速（m/s），若装药超出范围则返回0.0。
 */
float gun_muzzle_velocity(uint8_t charge, uint8_t projectile) {
    if (charge < 1 || charge > CHARGE_LEVELS) return 0.0f; /**< 防护超出范围的装药值 */
    return (projectile == PROJECTILE_HE)
        ? muzzle_velocity_he[charge - 1]  /**< 将1-based装药转换为0-based数组索引 */
        : muzzle_velocity_ap[charge - 1];
}
