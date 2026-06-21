/**
 * @文件 integrator.c
 * @简介 用于带阻力和风的2自由度质点弹丸运动的四阶龙格-库塔（RK4）弹道积分器。
 *
 * 本模块实现了弹丸在均匀重力场中运动方程的固定步长RK4 ODE求解器，
 * 包含二次阻力和恒定水平风。状态向量有4个分量：
 *
 *   state[0] = x  （水平位置，m，正值表示弹着方向）
 *   state[1] = z  （垂直位置，m，正值表示向上）
 *   state[2] = vx （水平速度，m/s）
 *   state[3] = vz （垂直速度，m/s）
 *
 * 运动方程（导数）：
 *   dx/dt  = vx
 *   dz/dt  = vz
 *   dvx/dt = -(Fd / m) * (vx - wx) / |v_rel|      （阻力与相对运动方向相反）
 *   dvz/dt = -g  -(Fd / m) * vz / |v_rel|           （重力 + 垂直阻力）
 *
 * 其中：
 *   Fd = 0.5 * Cd * rho * A * |v_rel|^2             （二次阻力大小）
 *   |v_rel| = sqrt((vx - wx)^2 + vz^2)              （相对于空气的速度）
 *   wx = 有效逆风分量（m/s）
 *
 * 假设：
 *   - 质点（无自旋，无马格努斯效应，无科里奥利力）
 *   - 整个弹道中空气密度恒定
 *   - 平面地球（重力方向恒定）
 *   - 无风速梯度（wx恒定）
 *   - Cd仅依赖于马赫数（参见drag.c）
 *
 * 时间步长：dt = 0.01 s（100 Hz积分）。在典型炮弹速度下，这给出约1 mm的
 * 位置分辨率，远在求解器使用的25 m射程容差之内。
 */

#include "integrator.h"
#include "drag.h"
#include <math.h>

#define G 9.80665f  /**< 标准重力加速度（m/s^2） */

/**
 * @简介 计算4元素状态向量的时间导数。
 *
 * 在当前状态处计算ODE系统的右侧：
 *
 *   deriv[0] = vx
 *   deriv[1] = vz
 *   deriv[2] = -(Fd/m) * (vx - wx) / vrel      [水平阻力减速度]
 *   deriv[3] = -g - (Fd/m) * vz / vrel          [垂直：重力 + 阻力]
 *
 * 阻力被分解为与相对速度分量成比例的水平分量和垂直分量。
 * 使用一个小值epsilon（0.001 m/s）防止vrel可忽略时
 * （例如在弹道顶点vz经过零时）除以零。
 *
 * @参数 state       当前状态向量 [x, z, vx, vz]。
 * @参数 deriv       输出导数向量 [dx/dt, dz/dt, dvx/dt, dvz/dt]。
 * @参数 mass        弹丸质量（kg）。
 * @参数 area        弹丸横截参考面积（m^2）。
 * @参数 rho         空气密度（kg/m^3）。
 * @参数 sound_speed 当地声速（m/s），用于计算马赫数。
 * @参数 wx          沿x轴的恒定水平风分量（m/s）。
 *                   正值 = 顺风，负值 = 逆风。
 */
static void derivatives(const float state[4], float deriv[4],
                        float mass, float area,
                        float rho, float sound_speed, float wx) {
    float vx = state[2];
    float vz = state[3];
    float vrx = vx - wx;                            /**< 相对水平速度（弹丸 - 风） */
    float vrel = sqrtf(vrx * vrx + vz * vz);        /**< 相对于气团的速度 */

    float mach = vrel / sound_speed;                /**< 马赫数 */
    float cd = drag_cd(mach > 0.0f ? mach : 0.0f); /**< 当前马赫数下的阻力系数 */
    float fd = 0.5f * cd * rho * area * vrel * vrel; /**< 阻力大小：Fd = 0.5*Cd*rho*A*v^2 */

    float inv_vrel = (vrel > 0.001f) ? (1.0f / vrel) : 1.0f; /**< vrel的倒数，带epsilon保护 */
    deriv[0] = vx;                                  /**< dx/dt = vx */
    deriv[1] = vz;                                  /**< dz/dt = vz */
    deriv[2] = -(fd / mass) * vrx * inv_vrel;       /**< dvx/dt = -(Fd/m)*(vrx/vrel) */
    deriv[3] = (vrel > 0.001f) ? (-G - (fd / mass) * vz * inv_vrel) : (-G); /**< dvz/dt = -g - (Fd/m)*(vz/vrel) */
}

/**
 * @简介 将状态向量推进一个RK4时间步长。
 *
 * 经典四阶龙格-库塔方法：
 *   k1 = f(t, y)
 *   k2 = f(t + dt/2, y + dt/2 * k1)
 *   k3 = f(t + dt/2, y + dt/2 * k2)
 *   k4 = f(t + dt,   y + dt * k3)
 *   y_next = y + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
 *
 * 该方法具有四阶精度（O(dt^4)局部截断误差）且
 * 不需要在分数时间处求导函数值（ODE系统是自治的；
 * 导数仅依赖于状态，不显式依赖于时间）。
 *
 * @参数 state       输入/输出状态向量 [x, z, vx, vz]（原地更新）。
 * @参数 dt          时间步长（s）。
 * @参数 mass        弹丸质量（kg）。
 * @参数 area        弹丸参考面积（m^2）。
 * @参数 rho         空气密度（kg/m^3）。
 * @参数 sound_speed 声速（m/s）。
 * @参数 wx          水平风分量（m/s）。
 */
void rk4_step(float state[4], float dt, float mass, float area,
               float rho, float sound_speed, float wx) {
    float k1[4], k2[4], k3[4], k4[4], temp[4];

    derivatives(state, k1, mass, area, rho, sound_speed, wx); /**< k1 = f(y) */

    for (int i = 0; i < 4; i++) temp[i] = state[i] + 0.5f * dt * k1[i]; /**< y + dt/2 * k1 */
    derivatives(temp, k2, mass, area, rho, sound_speed, wx);             /**< k2 = f(y + dt/2 * k1) */

    for (int i = 0; i < 4; i++) temp[i] = state[i] + 0.5f * dt * k2[i]; /**< y + dt/2 * k2 */
    derivatives(temp, k3, mass, area, rho, sound_speed, wx);             /**< k3 = f(y + dt/2 * k2) */

    for (int i = 0; i < 4; i++) temp[i] = state[i] + dt * k3[i];        /**< y + dt * k3 */
    derivatives(temp, k4, mass, area, rho, sound_speed, wx);             /**< k4 = f(y + dt * k3) */

    for (int i = 0; i < 4; i++)
        state[i] += (dt / 6.0f) * (k1[i] + 2.0f * k2[i] + 2.0f * k3[i] + k4[i]); /**< y += dt/6*(k1+2k2+2k3+k4) */
}

/**
 * @简介 积分完整弹道，直到弹丸到达或超过目标海拔，或超出最大射程。
 *
 * 弹道从原点(0, 0)开始，以初速v0和仰角elevation_rad发射。
 * 积分在以下情况停止：
 *   1. 弹丸下落到目标海拔或以下（state[1] >= delta_h
 *      且 state[3] <= 0，即通过目标平面下降）。
 *   2. 弹丸超出20,000 m水平射程（安全限制）。
 *
 * @p reached 标志指示弹丸在飞行中是否曾达到
 * 海拔 >= delta_h。这处理了delta_h > 0（目标高于炮位）
 * 且弹丸无法达到该海拔的情况。
 *
 * 即使在达到目标海拔后，积分也继续进行，直到弹丸
 * 回落通过它（在目标上方时 state[3] > 0，然后下降时 state[3]
 * <= 0）。最终射程是该交叉点的x位置；
 * 求解器通过小dt隐式处理插值。
 *
 * @参数 elevation_rad 发射仰角（弧度，0 = 水平）。
 * @参数 v0            初速（m/s）。
 * @参数 mass          弹丸质量（kg）。
 * @参数 area          弹丸参考面积（m^2）。
 * @参数 rho           空气密度（kg/m^3）。
 * @参数 sound_speed   声速（m/s）。
 * @参数 wx            水平风分量（m/s）。
 * @参数 delta_h       目标相对于炮位的海拔（m，正值 = 上方）。
 * @参数 reached       输出：若弹丸曾达到海拔 >= delta_h 则为true。
 * @返回 弹丸在下降过程中越过目标海拔点的射程（m），
 *         若目标从未达到则返回 -1.0。
 */
float rk4_trajectory(float elevation_rad, float v0,
                     float mass, float area,
                     float rho, float sound_speed,
                     float wx, float delta_h,
                     bool *reached) {
    const float dt = 0.01f; /**< 固定积分时间步长：10 ms */
    float state[4] = {
        0.0f, 0.0f,                          /**< 初始位置：原点 */
        v0 * cosf(elevation_rad),            /**< 初始 vx = v0 * cos(theta) */
        v0 * sinf(elevation_rad)             /**< 初始 vz = v0 * sin(theta) */
    };

    bool ever_reached = (delta_h <= 0.0f); /**< 若目标在炮位或以下，平凡可达 */

    while (state[1] >= delta_h || state[3] > 0.0f) { /**< 当在目标上方或仍在上升时继续 */
        if (state[1] >= delta_h) ever_reached = true; /**< 标记至少一次在目标上方 */
        rk4_step(state, dt, mass, area, rho, sound_speed, wx);
        if (state[0] > 20000.0f) break; /**< 安全：超过20 km射程时停止 */
    }

    if (reached) *reached = ever_reached;
    return ever_reached ? state[0] : -1.0f; /**< 若可达则返回射程，否则返回-1 */
}
