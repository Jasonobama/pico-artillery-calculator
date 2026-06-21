/**
 * @文件 core1_task.c
 * @简介 使用RP2040硬件FIFO的双核通信协议。
 *
 * 本模块通过内置的核间FIFO（multicore_fifo）
 * 在两个RP2040 Cortex-M0+核之间实现简单的请求/响应协议。
 * Core0运行UI（菜单、键盘、LCD）并将弹道求解请求派发给Core1，
 * Core1运行计算密集型的弹道积分和求根求解器。
 *
 * 协议流程：
 *
 *   Core0（客户端）                        Core1（服务端）
 *   --------------                        --------------
 *   1. 填充 g_request 结构体
 *   2. 设置 g_request_pending = true
 *   3. 设置 g_result_ready = false
 *   4. multicore_fifo_push(1)  ───────>   （阻塞等待FIFO字）
 *                                          5. 将 g_request 复制到本地 req
 *                                          6. 运行 solve_ballistics()
 *                                          7. 将结果存入 g_result
 *                                          8. 设置 g_result_ready = true
 *   9. multicore_fifo_pop()  <─────────   9. multicore_fifo_push(1)
 *   10. 读取 g_result
 *
 * 同步机制：
 *   - g_request_pending：防止Core0在Core1尚未消费请求之前
 *     覆盖请求。若请求待处理且尚未完成，新请求将被拒绝
 *     （返回false）。
 *   - g_result_ready：向Core0发出结果可用的信号。
 *     若在结果就绪之前调用core1_get_result()，将阻塞在FIFO上
 *     （提供隐式屏障）。
 *   - 所有共享变量声明为volatile，以防止编译器
 *     跨核边界优化。
 *
 * 内存模型：
 *   RP2040核心共享统一的264 kB SRAM。Cortex-M0+上无需
 *   显式缓存一致性（无数据缓存）。multicore_fifo_push/pop中
 *   隐含的编译器屏障足以确保正确的顺序。
 */

#include "core1_task.h"
#include "pico/multicore.h"

static volatile solve_request_t  g_request;          /**< 从Core0到Core1的共享请求 */
static volatile solver_result_t  g_result;           /**< 从Core1到Core0的共享结果 */
static volatile bool             g_request_pending = false; /**< 保护：请求已发布 */
static volatile bool             g_result_ready = false;    /**< 保护：结果可用 */

/**
 * @简介 Core1主循环：等待请求，运行求解器，返回结果。
 *
 * 此函数在Core1上无限运行。它阻塞在核间FIFO上等待
 * Core0的信号。收到信号后，它将请求从共享volatile结构体
 * 复制到本地（以避免长时间计算期间的TOCTOU问题），
 * 运行相应的求解器（自动或固定装药），将结果存入g_result，
 * 设置就绪标志，并通过FIFO向Core0发回信号。
 *
 * 本地复制请求是有意为之：求解器可能运行数百毫秒，
 * 在此期间Core0理论上可能尝试发布另一个请求。
 * 本地复制确保求解器在一致的快照上运行。
 */
void core1_entry(void) {
    while (true) {
        multicore_fifo_pop_blocking(); /**< 阻塞直到Core0发出请求信号 */

        /** 将请求复制到本地栈以避免长时间求解器运行期间的TOCTOU */
        solve_request_t req;
        req.target_range_m  = g_request.target_range_m;
        req.delta_height_m  = g_request.delta_height_m;
        req.site_altitude_m = g_request.site_altitude_m;
        req.temperature_c   = g_request.temperature_c;
        req.wind_speed_ms   = g_request.wind_speed_ms;
        req.wind_dir_deg    = g_request.wind_dir_deg;
        req.bearing_deg     = g_request.bearing_deg;
        req.projectile      = g_request.projectile;
        req.angle_branch    = g_request.angle_branch;
        req.charge_auto     = g_request.charge_auto;
        req.fixed_charge    = g_request.fixed_charge;

        solver_result_t res;
        if (req.charge_auto) {
            res = solve_ballistics(
                req.target_range_m, req.delta_height_m,
                req.site_altitude_m, req.temperature_c,
                req.wind_speed_ms, req.wind_dir_deg,
                req.bearing_deg,
                req.projectile, req.angle_branch);
        } else {
            res = solve_ballistics_fixed_charge(
                req.target_range_m, req.delta_height_m,
                req.site_altitude_m, req.temperature_c,
                req.wind_speed_ms, req.wind_dir_deg,
                req.bearing_deg,
                req.projectile, req.angle_branch,
                req.fixed_charge);
        }

        g_result = res;
        g_result_ready = true; /**< 向Core0发出结果可用信号 */

        multicore_fifo_push_blocking(1); /**< 解除Core0的core1_get_result()阻塞 */
    }
}

/**
 * @简介 使用求解器入口点启动Core1。
 *
 * 必须在系统初始化期间调用一次（通常从Core0上的main()
 * 调用）。启动Core1执行core1_entry()。
 */
void core1_launch(void) {
    multicore_launch_core1(core1_entry);
}

/**
 * @简介 向Core1发布求解请求。
 *
 * 填充共享请求结构体并通过FIFO向Core1发信号。
 * 若前一个请求仍在待处理状态（g_request_pending为true
 * 且g_result_ready为false），则拒绝请求（返回false）。
 *
 * @参数 req 指向包含所有求解器参数的请求结构体的指针。
 * @返回 若请求被接受并派发则返回true；若Core1正在
 *         处理前一个请求则返回false。
 */
bool core1_request_solve(solve_request_t *req) {
    if (g_request_pending && !g_result_ready) return false; /**< Core1忙碌：拒绝 */
    g_request = *req;                /**< 将请求复制到共享volatile结构体 */
    g_request_pending = true;
    g_result_ready = false;
    multicore_fifo_push_blocking(1); /**< 向Core1发出开始处理的信号 */
    return true;
}

/**
 * @简介 检查Core1是否已完成当前请求。
 *
 * 非阻塞轮询。从Core0的主循环中调用此函数以
 * 检查结果是否可用而不阻塞UI。
 *
 * @返回 若g_result_ready已设置（结果可用）则返回true。
 */
bool core1_result_ready(void) {
    return g_result_ready;
}

/**
 * @简介 获取最近一次求解请求的结果。
 *
 * 若结果尚未就绪，此函数将阻塞在FIFO上直到
 * Core1发出完成信号。获取后，重置g_request_pending以
 * 允许发布新请求。
 *
 * @返回 Core1计算出的solver_result_t。
 */
solver_result_t core1_get_result(void) {
    if (!g_result_ready) multicore_fifo_pop_blocking(); /**< 阻塞直到Core1推送结果 */
    g_request_pending = false; /**< 允许新请求 */
    return (solver_result_t)g_result;
}
