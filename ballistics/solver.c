/**
 * @文件 solver.c
 * @简介 用于火炮发射仰角的扫描-精化求根求解器。
 *
 * 确定击中给定射程和相对海拔目标所需的仰角，
 * 考虑了大气条件、风力和弹丸类型。采用两阶段方法：
 *
 * 阶段1 - 粗扫描：以5度为增量从5度到65度评估弹道，
 *   以找到产生最大射程的近似仰角（theta_max）。
 *
 * 阶段2 - 求根：射程-仰角曲线在theta_max的任一侧
 *   是严格单调的（在theta_max以下的角度，LOW分支上递增；
 *   在theta_max以上的角度，HIGH分支上递减）。
 *   在适当分支上进行二分搜索，找出在容差范围内
 *   命中目标射程的仰角。
 *
 * 两个入口点：
 *   - solve_ballistics()：自动选择装药等级（尝试C1到C5，
 *     直到目标可达）。
 *   - solve_ballistics_fixed_charge()：使用指定的装药等级。
 *
 * 容差：自适应，为目标射程的0.5%，钳制在[25, 100] m范围内。
 * 这与历史上火炮精度（约射程的0.5%）一致。
 */

#include "solver.h"
#include "gun_data.h"
#include "atmosphere.h"
#include "integrator.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846  /**< 圆周率常量（嵌入式系统math.h中不一定定义） */
#endif

/**
 * @简介 将度转换为弧度。
 * @参数 deg 角度（度）。
 * @返回 角度（弧度）。
 */
static inline float deg_to_rad(float deg) { return deg * ((float)M_PI / 180.0f); }

/**
 * @简介 便捷包装函数：运行弹道并返回射程。
 *
 * 将仰角从度转换为弧度并委托给
 * rk4_trajectory()。
 *
 * @参数 ang_deg  仰角（度）。
 * @参数 v0       初速（m/s）。
 * @参数 mass     弹丸质量（kg）。
 * @参数 area     弹丸参考面积（m^2）。
 * @参数 rho      空气密度（kg/m^3）。
 * @参数 snd      声速（m/s）。
 * @参数 wx       水平风分量（m/s）。
 * @参数 dh       目标相对于炮位的海拔（m）。
 * @参数 reached  输出：若弹丸达到目标海拔则为true。
 * @返回 目标交叉点的射程（m），若不可达则返回 -1.0。
 */
static float compute(float ang_deg, float v0, float mass, float area,
                     float rho, float snd, float wx, float dh, bool *reached) {
    return rk4_trajectory(deg_to_rad(ang_deg), v0,
                           mass, area, rho, snd, wx, dh, reached);
}

/**
 * @简介 使用单一固定装药等级求解发射仰角。
 *
 * 算法（两阶段扫描 + 二分法）：
 *
 * 阶段1 - 粗扫描（5度步长）：
 *   从5度到MAX_ELEVATION_DEG（65度）以5度为增量扫描仰角。
 *   找到theta_max = 产生最大射程的角度（射程-仰角曲线的峰值）。
 *   若最大射程小于目标射程减去容差，
 *   则该装药下目标不可达。
 *
 * 阶段1b - 精扫描（1度步长）：
 *   在theta_max +/- 5度范围内以1度分辨率重新扫描，
 *   以精化峰值估计。
 *
 * 阶段2 - 二分法（二分搜索）：
 *   射程-仰角曲线有两个单调分支：
 *     - LOW分支：  角度 [1, theta_max]，射程随角度增加。
 *     - HIGH分支： 角度 [theta_max, 65]，射程随角度减小。
 *   根据 @p branch 参数选择适当的分支。
 *
 *   在二分之前，向上步进lo直到找到可达的角度
 *   （处理上坡射击时低角度射程不足的情况）。
 *   然后进行最多20次迭代的二分，当区间宽度 < 0.02度或
 *   射程误差 < 1.0 m时停止。最终中点与容差进行校核。
 *
 *   对于HIGH分支中接近极限的目标，回退检查
 *   MAX_ELEVATION_DEG本身是否能命中目标。
 *
 * @参数 range_m    目标水平射程（m）。
 * @参数 delta_h_m  目标相对于炮位的海拔（m，正值 = 上方）。
 * @参数 v0         该装药的初速（m/s）。
 * @参数 mass       弹丸质量（kg）。
 * @参数 rho        空气密度（kg/m^3）。
 * @参数 snd        声速（m/s）。
 * @参数 wx         有效水平风分量（m/s）。
 * @参数 charge     装药编号（1-5），仅用于结果报告。
 * @参数 branch     BRANCH_LOW（0）或BRANCH_HIGH（1）。
 * @返回 包含可达标志、仰角（度）和装药编号的solver_result_t。
 */
static solver_result_t solve_one_charge(
    float range_m, float delta_h_m,
    float v0, float mass, float rho, float snd, float wx,
    uint8_t charge, uint8_t branch) {

    solver_result_t result = {false, 0.0f, 0};
    float tol = range_m * 0.005f;        /**< 目标射程的0.5% */
    if (tol < 25.0f) tol = 25.0f;        /**< 最小容差：25 m（约5 km以下） */
    if (tol > 100.0f) tol = 100.0f;      /**< 最大容差：100 m（约20 km以上） */

    /** 阶段1：以5度间隔粗扫描以找到theta_max */
    float max_r = -1.0f;                 /**< 目前找到的最大射程（m） */
    float max_a = 45.0f;                 /**< 产生max_r的角度（度） */
    bool any = false;                    /**< 是否有任何角度达到了目标海拔 */
    for (float a = 5.0f; a <= MAX_ELEVATION_DEG; a += 5.0f) {
        bool reached;
        float r = compute(a, v0, mass, PROJECTILE_AREA,
                         rho, snd, wx, delta_h_m, &reached);
        if (reached && r > max_r) { max_r = r; max_a = a; any = true; }
    }
    if (!any || max_r < range_m - tol) return result; /**< 该装药下目标不可达 */

    /** 阶段1b：在theta_max周围以1度间隔精扫描 */
    float lo_scan = max_a - 5.0f; if (lo_scan < 1.0f) lo_scan = 1.0f;           /**< 扫描下限，最小1度 */
    float hi_scan = max_a + 5.0f; if (hi_scan > MAX_ELEVATION_DEG) hi_scan = MAX_ELEVATION_DEG; /**< 扫描上限 */
    for (float a = lo_scan; a <= hi_scan; a += 1.0f) {
        bool reached;
        float r = compute(a, v0, mass, PROJECTILE_AREA,
                         rho, snd, wx, delta_h_m, &reached);
        if (reached && r > max_r) { max_r = r; max_a = a; }
    }

    if (max_r < range_m - tol) return result; /**< 精扫描后仍然不可达 */

    /** 阶段2：在所选单调分支上设置二分区间 */
    float lo, hi;
    if (branch == BRANCH_HIGH) {
        lo = max_a; hi = MAX_ELEVATION_DEG; /**< HIGH分支：theta_max以上的角度 */
    } else {
        lo = 1.0f; hi = max_a;              /**< LOW分支：theta_max以下的角度 */
    }

    /** 找到最小可行角度（对上坡/远处射击向上步进lo） */
    bool reached_lo, reached_hi;
    float r_lo = compute(lo, v0, mass, PROJECTILE_AREA,
                         rho, snd, wx, delta_h_m, &reached_lo);
    while (!reached_lo && lo < hi) {
        lo += 1.0f; /**< 递增lo直到弹丸能到达目标海拔 */
        if (lo >= hi) break;
        r_lo = compute(lo, v0, mass, PROJECTILE_AREA,
                       rho, snd, wx, delta_h_m, &reached_lo);
    }
    if (!reached_lo) return result; /**< 即使在hi处也无法到达目标海拔 */

    float r_hi = compute(hi, v0, mass, PROJECTILE_AREA,
                         rho, snd, wx, delta_h_m, &reached_hi);
    if (!reached_hi) return result;

    bool increasing = (r_lo < r_hi); /**< 单调性：LOW分支上递增，HIGH分支上递减 */

    /** 快速返回：若lo已在容差内命中目标 */
    if (!increasing && r_lo >= range_m && fabsf(r_lo - range_m) <= tol) {
        result.reachable = true;
        result.elevation_deg = lo;
        result.charge_number = charge;
        return result;
    }

    /** 二分搜索（最多20次迭代，0.02度区间阈值） */
    for (int iter = 0; iter < 20; iter++) {
        float mid = (lo + hi) * 0.5f;               /**< 当前区间中点 */
        if (fabsf(hi - lo) < 0.02f) break;          /**< 区间收敛到 <0.02度 */

        bool reached_mid;
        float r_mid = compute(mid, v0, mass, PROJECTILE_AREA,
                              rho, snd, wx, delta_h_m, &reached_mid);
        if (!reached_mid) { lo = mid; continue; }   /**< 中点无法到达海拔：将lo上移 */

        if (fabsf(r_mid - range_m) < 1.0f) {        /**< 射程误差 < 1 m：立即接受 */
            result.reachable = true;
            result.elevation_deg = mid;
            result.charge_number = charge;
            return result;
        }

        /** 基于单调性缩小区间 */
        if (increasing) {
            if (r_mid < range_m) lo = mid; else hi = mid;   /**< r(mid)太近 -> lo上移 */
        } else {
            if (r_mid > range_m) lo = mid; else hi = mid;   /**< r(mid)太远 -> lo上移（递减） */
        }
    }

    /** 最终检查：评估最终区间的中点 */
    float mid = (lo + hi) * 0.5f;
    bool reached_final;
    float r_final = compute(mid, v0, mass, PROJECTILE_AREA,
                            rho, snd, wx, delta_h_m, &reached_final);
    if (reached_final && fabsf(r_final - range_m) <= tol) {
        result.reachable = true;
        result.elevation_deg = mid;
        result.charge_number = charge;
        return result;
    }

    /** HIGH分支回退：即使最大仰角能否命中目标 */
    if (branch == BRANCH_HIGH) {
        bool reached_max;
        float r_max = compute(MAX_ELEVATION_DEG, v0, mass, PROJECTILE_AREA,
                              rho, snd, wx, delta_h_m, &reached_max);
        if (reached_max && r_max >= range_m) {
            result.reachable = true;
            result.elevation_deg = MAX_ELEVATION_DEG; /**< 返回最大仰角作为解 */
            result.charge_number = charge;
            return result;
        }
    }

    return result; /**< 未找到解 */
}

/**
 * @简介 使用自动装药选择求解弹道。
 *
 * 根据海拔和温度计算大气特性，将风分解为
 * 有效水平分量（逆风/顺风，平行于射击线），
 * 然后遍历装药等级C1到C5。
 * 返回找到的第一个可达解（能够命中目标的
 * 最低装药，以最小化发射药消耗）。
 *
 * 风模型：
 *   风向相对于真北。方位角是从真北的
 *   射击方向。有效风分量wx是风在射击线上的投影：
 *     wx = -wind_speed * cos(wind_dir - bearing)
 *   负wx为逆风（减小射程），正wx为顺风。
 *
 * @参数 range_m        目标水平射程（m）。
 * @参数 delta_h_m      目标相对于炮位的海拔（m，正值 = 上方）。
 * @参数 altitude_m     炮位海拔（m）。
 * @参数 temperature_c  站点空气温度（摄氏度）。
 * @参数 wind_speed_ms  风速大小（m/s）。
 * @参数 wind_dir_deg   风向（从真北的度数，气象学惯例）。
 * @参数 bearing_deg    射击方位角（从真北的度数）。
 * @参数 projectile     PROJECTILE_HE（0）或PROJECTILE_AP（1）。
 * @参数 branch         BRANCH_LOW（0）或BRANCH_HIGH（1）角度分支。
 * @返回 包含可达标志、仰角和装药编号的solver_result_t。
 */
solver_result_t solve_ballistics(
    float range_m, float delta_h_m,
    float altitude_m, float temperature_c,
    float wind_speed_ms, float wind_dir_deg,
    float bearing_deg, uint8_t projectile,
    uint8_t branch) {

    solver_result_t result = {false, 0.0f, 0};

    float rho = atm_density(altitude_m, temperature_c); /**< 炮位的空气密度 */
    float snd = atm_sound_speed(temperature_c);          /**< 炮位的声速 */
    float mass = gun_mass(projectile);                   /**< 弹丸质量 */

    float wind_rad = deg_to_rad(wind_dir_deg - bearing_deg); /**< 风相对于射击线的角度 */
    float wx = -wind_speed_ms * cosf(wind_rad); /**< 投影：负值 = 逆风 */

    /** 从最低（C1）到最高（C5）尝试装药；返回第一个可达的 */
    for (uint8_t charge = 1; charge <= CHARGE_LEVELS; charge++) {
        float v0 = gun_muzzle_velocity(charge, projectile);
        result = solve_one_charge(range_m, delta_h_m, v0, mass, rho, snd, wx, charge, branch);
        if (result.reachable) return result; /**< 命中目标的最低装药 */
    }

    return result; /**< 任何装药均不可达 */
}

/**
 * @简介 使用单一固定装药等级求解弹道。
 *
 * 与solve_ballistics()相同，但使用调用者指定的装药
 * 等级而非自动选择。用于手动装药模式。
 *
 * @参数 range_m        目标水平射程（m）。
 * @参数 delta_h_m      目标相对于炮位的海拔（m）。
 * @参数 altitude_m     炮位海拔（m）。
 * @参数 temperature_c  空气温度（摄氏度）。
 * @参数 wind_speed_ms  风速（m/s）。
 * @参数 wind_dir_deg   风向（从真北的度数）。
 * @参数 bearing_deg    射击方位角（从真北的度数）。
 * @参数 projectile     PROJECTILE_HE或PROJECTILE_AP。
 * @参数 branch         BRANCH_LOW或BRANCH_HIGH。
 * @参数 charge         固定装药编号（1-5）。
 * @返回 该特定装药的solver_result_t。
 */
solver_result_t solve_ballistics_fixed_charge(
    float range_m, float delta_h_m,
    float altitude_m, float temperature_c,
    float wind_speed_ms, float wind_dir_deg,
    float bearing_deg, uint8_t projectile,
    uint8_t branch, uint8_t charge) {

    float rho = atm_density(altitude_m, temperature_c);
    float snd = atm_sound_speed(temperature_c);
    float mass = gun_mass(projectile);

    float wind_rad = deg_to_rad(wind_dir_deg - bearing_deg);
    float wx = -wind_speed_ms * cosf(wind_rad);

    float v0 = gun_muzzle_velocity(charge, projectile);
    return solve_one_charge(range_m, delta_h_m, v0, mass, rho, snd, wx, charge, branch);
}
