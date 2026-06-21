/**
 * @文件 menu.c
 * @简介 弹道计算器参数输入的多页UI状态机。
 *
 * 本模块实现了一个由4x4键盘驱动的线性向导式菜单。
 * 用户通过按键'A'（下一步/确认）和'B'（上一步/编辑）依次浏览输入字段
 * （场景、装药模式、弹药、距离、方位角、海拔、目标高度差、温度、风速、风向）。
 * 在最后一个数值字段之后，按'D'键进入确认页面，
 * 然后在core1上触发弹道计算。结果显示在LCD1602上，
 * 包含仰角、装药号、弹药类型和模式。
 *
 * 键盘映射：
 *   - A: 确认当前字段值并前进到下一个字段（或确认）。
 *   - B: 返回上一个字段（或从结果页面重置）。
 *   - C: 清除当前输入缓冲区。
 *   - D: 从最后一个字段前进到确认页面 / 触发计算。
 *   - *: 在允许负输入的字段上切换正负号（+/-）。
 *   - #: 在数值字段上添加小数点。
 *   - 0-9: 向输入缓冲区追加数字。
 *
 * 场景预设（影响默认弹药和角度分支）：
 *   - 场景1-4, 5: 用户首先选择场景；场景5使用
 *     BRANCH_HIGH（大角度），其他使用BRANCH_LOW。
 *   - 场景2自动尝试低角度和高角度两个分支。
 */

#include "menu.h"
#include "lcd1602_i2c.h"
#include "core1_task.h"
#include "gun_data.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static menu_field_t current_field = FIELD_SCENARIO; /**< 当前活动菜单字段 */

/** 各数值/文本字段的输入字符缓冲区 */
static char scenario_buf[2] = "";
static char charge_buf[2] = "";
static char ammo_buf[2] = "";
static char dist_buf[16] = "";
static char brg_buf[16] = "";
static char alt_buf[16] = "";
static char dh_buf[16] = "";
static char temp_buf[16] = "";
static char wind_spd_buf[16] = "";
static char wind_dir_buf[16] = "";

static char *current_input_buf = NULL; /**< 指向当前活动字段缓冲区的指针 */
static int   current_buf_size = 0;     /**< 当前活动缓冲区的容量（含NUL终止符） */
static bool  allow_sign = false;       /**< 当前字段是否允许按'*'键切换正负号 */

static uint8_t scenario = 0;          /**< 选中的场景编号（1-5） */
static bool    charge_auto = true;    /**< true = 自动装药选择；false = 手动 */
static uint8_t fixed_charge = 0;      /**< 手动模式下的装药等级（1-5） */
static uint8_t ammo_type = 1;         /**< 1 = HE（高爆弹），2 = AP（穿甲弹） */
static bool calculating = false;      /**< 标记计算正在进行中 */
static solver_result_t last_result;   /**< 最近一次计算的缓存结果 */

static void set_field(menu_field_t field);
static void display_current_field(void);

/**
 * @简介 获取给定场景的默认弹药类型。
 *
 * 场景2默认为AP（2）；其他所有场景默认为HE（1）。
 *
 * @参数 s 场景编号（1-5）。
 * @返回 默认弹药类型（1或2）。
 */
static uint8_t get_default_ammo(uint8_t s) {
    return (s == 2) ? 2 : 1; /**< 场景2为AP专用；其他使用HE */
}

/**
 * @简介 初始化菜单系统：LCD、默认状态和显示。
 *
 * 设置LCD1602，将结果初始化为不可达，并
 * 显示第一个输入字段（场景选择）。
 */
void menu_init(void) {
    lcd1602_init();
    last_result.reachable = false;
    last_result.elevation_deg = 0.0f;
    last_result.charge_number = 0;
    set_field(FIELD_SCENARIO);
    lcd1602_clear();
    display_current_field();
}

/**
 * @简介 切换当前活动输入字段并绑定相应的缓冲区。
 *
 * 更新current_field，重置allow_sign，并将current_input_buf
 * 指向正确的静态缓冲区及其关联的容量。个位数字段
 * （场景、装药、弹药）的缓冲区为2字节；数值字段为16字节。
 * 高度差和温度字段允许正负号切换。
 * 进入字段时缓冲区被清空，用户以空白输入开始。
 *
 * @参数 field 要激活的菜单字段。
 */
static void set_field(menu_field_t field) {
    current_field = field;
    allow_sign = false;

    switch (field) {
        case FIELD_SCENARIO:   current_input_buf = scenario_buf;  current_buf_size = 2;  break;
        case FIELD_CHARGE_MODE:current_input_buf = charge_buf;    current_buf_size = 2;  break;
        case FIELD_AMMO:       current_input_buf = ammo_buf;      current_buf_size = 2;  break;
        case FIELD_DIST:       current_input_buf = dist_buf;      current_buf_size = 16; break;
        case FIELD_BRG:        current_input_buf = brg_buf;       current_buf_size = 16; break;
        case FIELD_ALT:        current_input_buf = alt_buf;       current_buf_size = 16; break;
        case FIELD_DH:         current_input_buf = dh_buf;        current_buf_size = 16; allow_sign = true; break; /**< 高度差可为负值（目标低于炮位） */
        case FIELD_TEMP:       current_input_buf = temp_buf;      current_buf_size = 16; allow_sign = true; break; /**< 温度可为零下 */
        case FIELD_WIND_SPD:   current_input_buf = wind_spd_buf;  current_buf_size = 16; break;
        case FIELD_WIND_DIR:   current_input_buf = wind_dir_buf;  current_buf_size = 16; break;
        default:               current_input_buf = NULL;          current_buf_size = 0;  break;
    }
    if (current_input_buf) current_input_buf[0] = '\0'; /**< 进入字段时清空缓冲区 */
}

/**
 * @简介 清空当前输入缓冲区并刷新显示。
 */
static void clear_current_input(void) {
    if (current_input_buf) current_input_buf[0] = '\0';
    display_current_field();
}

/**
 * @简介 切换当前输入缓冲区的正负号前缀。
 *
 * 仅当当前字段的allow_sign为true时执行。处理三种情况：
 * (a) 缓冲区以'-'开头，移除它；(b) 缓冲区非空且无'-'，在前面添加'-'；
 * (c) 缓冲区为空，无操作。
 * 使用memmove进行原地移位，避免临时缓冲区分配。
 */
static void toggle_sign(void) {
    if (!allow_sign || !current_input_buf) return;
    if (current_input_buf[0] == '-') {
        memmove(current_input_buf, current_input_buf + 1, strlen(current_input_buf)); /**< 左移覆盖'-' */
    } else {
        memmove(current_input_buf + 1, current_input_buf, strlen(current_input_buf) + 1); /**< 右移腾出空间 */
        current_input_buf[0] = '-';
    }
    display_current_field();
}

/**
 * @简介 如果未满，向当前输入缓冲区追加一个字符。
 *
 * @参数 c 要追加的字符（数字、小数点等）。
 */
static void append_char(char c) {
    if (!current_input_buf) return;
    int len = strlen(current_input_buf);
    if (len >= current_buf_size - 1) return; /**< 保留1字节给NUL终止符 */
    current_input_buf[len] = c;
    current_input_buf[len + 1] = '\0';
    display_current_field();
}

/**
 * @简介 安全地从字符串解析浮点数，空输入默认为0。
 *
 * @参数 s 来自键盘缓冲区的输入字符串。
 * @返回 解析出的浮点数值，若字符串为空则返回0.0f。
 */
static float parse_float(const char *s) {
    if (s[0] == '\0') return 0.0f;
    return (float)atof(s);
}

/**
 * @简介 安全地从字符串解析整数，空输入默认为0。
 *
 * @参数 s 来自键盘缓冲区的输入字符串。
 * @返回 解析出的整数值，若字符串为空则返回0。
 */
static int parse_int(const char *s) {
    if (s[0] == '\0') return 0;
    return atoi(s);
}

/**
 * @简介 收集所有参数并派发弹道计算。
 *
 * 从所有输入缓冲区构建solve_request_t。场景5强制使用
 * BRANCH_HIGH（大角度高抛弹道）。场景2先尝试
 * BRANCH_LOW，若不可达则回退到BRANCH_HIGH。
 * 请求通过FIFO发送到core1，结果显示在RESULTS页面上。
 */
static void do_calculate(void) {
    solve_request_t req;
    req.target_range_m  = parse_float(dist_buf);
    req.bearing_deg     = parse_float(brg_buf);
    req.site_altitude_m = parse_float(alt_buf);
    req.delta_height_m  = parse_float(dh_buf);
    req.temperature_c   = parse_float(temp_buf);
    req.wind_speed_ms   = parse_float(wind_spd_buf);
    req.wind_dir_deg    = parse_float(wind_dir_buf);
    req.projectile      = (ammo_type == 2) ? PROJECTILE_AP : PROJECTILE_HE;
    req.charge_auto     = charge_auto;
    req.fixed_charge    = fixed_charge;

    uint8_t branch = (scenario == 5) ? BRANCH_HIGH : BRANCH_LOW; /**< 场景5 = 大角度 */

    lcd1602_clear();
    lcd1602_set_cursor(0, 0);
    lcd1602_string("CALCULATING...");
    lcd1602_set_cursor(1, 0);
    lcd1602_string("                "); /**< 清除第二行 */

    if (scenario == 2) {
        /** 场景2：尝试两种角度分支（先低角度，后高角度） */
        req.angle_branch = BRANCH_LOW;
        core1_request_solve(&req);
        last_result = core1_get_result();
        if (!last_result.reachable) {
            req.angle_branch = BRANCH_HIGH;
            core1_request_solve(&req);
            last_result = core1_get_result();
        }
    } else {
        req.angle_branch = branch;
        core1_request_solve(&req);
        last_result = core1_get_result();
    }

    current_field = FIELD_RESULT;
    display_current_field();
}

/**
 * @简介 为当前菜单字段渲染LCD显示。
 *
 * 每个字段有双行格式：第1行是字段标签/提示，
 * 第2行是当前输入值或结果数据。FIELD_CONFIRM
 * 屏幕显示操作键（D=计算, B=编辑, A=复核）。
 * FIELD_RESULT屏幕显示仰角和装药信息（若可达），
 * 或"超射程"消息。
 */
static void display_current_field(void) {
    lcd1602_clear();
    char line1[17] = "";
    char line2[17] = "";
    const char *val = current_input_buf ? current_input_buf : "";

    switch (current_field) {
        case FIELD_SCENARIO:  snprintf(line1, 17, "SCENARIO 1-5:");  snprintf(line2, 17, "%s", val); break;
        case FIELD_CHARGE_MODE:
            if (current_input_buf[0] == '\0')
                { snprintf(line1, 17, "CHG MODE:");
                  snprintf(line2, 17, "AUTO"); } /**< 空缓冲区 = 自动模式 */
            else
                { snprintf(line1, 17, "CHG MODE:");
                  snprintf(line2, 17, "MANUAL C%c", current_input_buf[0]); } /**< 输入数字 = 手动装药 */
            break;
        case FIELD_AMMO:      snprintf(line1, 17, "AMMO 1=HE 2=AP"); snprintf(line2, 17, "%s", val); break;
        case FIELD_DIST:      snprintf(line1, 17, "DIST(m):");       snprintf(line2, 17, "%s", val); break;
        case FIELD_BRG:       snprintf(line1, 17, "BRG(deg,0-359):");snprintf(line2, 17, "%s", val); break;
        case FIELD_ALT:       snprintf(line1, 17, "ALT ASL(m):");    snprintf(line2, 17, "%s", val); break;
        case FIELD_DH:        snprintf(line1, 17, "TGT dH(m):");     snprintf(line2, 17, "%s", val); break;
        case FIELD_TEMP:      snprintf(line1, 17, "TEMP(C):");       snprintf(line2, 17, "%s", val); break;
        case FIELD_WIND_SPD:  snprintf(line1, 17, "WIND SPD(m/s):"); snprintf(line2, 17, "%s", val); break;
        case FIELD_WIND_DIR:  snprintf(line1, 17, "WIND DIR(deg):"); snprintf(line2, 17, "%s", val); break;
        case FIELD_CONFIRM:   snprintf(line1, 17, "D=CALC B=EDIT");  snprintf(line2, 17, "A=REVIEW");   break;
        case FIELD_RESULT:
            if (last_result.reachable) {
                const char *al = (ammo_type == 2) ? "AP" : "HE";
                const char *mode = charge_auto ? "AUTO" : "FIXED";
                snprintf(line1, 17, "ELEV:%.1f DEG", last_result.elevation_deg);
                snprintf(line2, 17, "CHG:%d %s %s", last_result.charge_number, al, mode);
            } else {
                snprintf(line1, 17, "OUT OF RANGE");
                snprintf(line2, 17, "MAX:9300m C5"); /**< 硬编码的最大射程参考值 */
            }
            break;
        default: break;
    }

    lcd1602_set_cursor(0, 0);
    lcd1602_string(line1);
    lcd1602_set_cursor(1, 0);
    lcd1602_string(line2);
}

/**
 * @简介 在菜单状态机内处理来自4x4键盘的按键。
 *
 * 这是核心事件处理函数。按键绑定：
 *   - 'A': 确认/前进。在普通字段上，验证输入并前进到
 *          下一个字段。在FIELD_CONFIRM上，重新进入复核。在FIELD_RESULT上，
 *          重置所有状态并返回FIELD_SCENARIO。
 *   - 'B': 后退。返回上一个字段。在FIELD_RESULT上，重置到
 *          FIELD_SCENARIO（新计算）。
 *   - 'C': 清除当前输入缓冲区。
 *   - 'D': 完成/计算。在FIELD_WIND_DIR处，前进到确认页面。在
 *          FIELD_CONFIRM处，触发弹道计算。
 *   - '*': 在启用allow_sign的字段（高度差、温度）上切换正负号。
 *   - '#': 添加小数点（仅限DIST到WIND_DIR之间的数值字段）。
 *   - '0'-'9': 追加数字。在FIELD_CHARGE_MODE上仅接受数字1-5。
 *
 * @参数 key pico_keypad_get_key()返回的字符。
 */
void menu_process_key(char key) {
    if (key == 'A') {
        /** A = 确认 / 前进 */
        if (current_field == FIELD_RESULT) {
            /** 完全重置：清空所有缓冲区并返回场景选择 */
            scenario = 0; charge_auto = true; fixed_charge = 0; ammo_type = 1;
            scenario_buf[0] = '\0'; charge_buf[0] = '\0'; ammo_buf[0] = '\0';
            dist_buf[0] = '\0'; brg_buf[0] = '\0'; alt_buf[0] = '\0';
            dh_buf[0] = '\0'; temp_buf[0] = '\0';
            wind_spd_buf[0] = '\0'; wind_dir_buf[0] = '\0';
            set_field(FIELD_SCENARIO);
            display_current_field();
            return;
        }
        if (current_field == FIELD_CHARGE_MODE) {
            /** 装药模式：数字1-5 = 手动固定装药；否则 = 自动 */
            if (current_input_buf[0] >= '1' && current_input_buf[0] <= '5') {
                charge_auto = false;
                fixed_charge = (uint8_t)(current_input_buf[0] - '0'); /**< 将ASCII数字转换为整数 */
            } else {
                charge_auto = true;
            }
            set_field(FIELD_AMMO);
            display_current_field();
            return;
        }
        /** 验证并应用场景和弹药选择 */
        if (current_field == FIELD_SCENARIO) {
            scenario = parse_int(scenario_buf);
            if (scenario < 1 || scenario > 5) { clear_current_input(); return; } /**< 无效场景：清空并停留 */
            ammo_type = get_default_ammo(scenario);
            snprintf(ammo_buf, 2, "%d", ammo_type); /**< 用场景默认值预填弹药 */
        } else if (current_field == FIELD_AMMO) {
            ammo_type = parse_int(ammo_buf);
            if (ammo_type != 1 && ammo_type != 2) { clear_current_input(); return; } /**< 仅1或2有效 */
            snprintf(ammo_buf, 2, "%d", ammo_type);
        }
        /** 若尚未到达确认页面，前进到下一个字段 */
        if (current_field < FIELD_CONFIRM) {
            set_field(current_field + 1);
            display_current_field();
        }
        return;
    }

    if (key == 'B') {
        /** B = 后退 / 编辑 */
        if (current_field == FIELD_RESULT) {
            /** 重置到场景选择以进行新计算 */
            scenario = 0;
            set_field(FIELD_SCENARIO);
            display_current_field();
            return;
        }
        if (current_field > FIELD_SCENARIO) {
            set_field(current_field - 1); /**< 后退一个字段 */
            display_current_field();
        }
        return;
    }

    if (key == 'C') {
        /** C = 清除当前输入 */
        clear_current_input();
        return;
    }

    if (key == 'D') {
        /** D = 完成 / 计算 */
        if (current_field == FIELD_WIND_DIR) {
            set_field(FIELD_CONFIRM); /**< 最后一个数值字段 -> 确认页面 */
            display_current_field();
            return;
        }
        if (current_field == FIELD_CONFIRM) {
            do_calculate(); /**< 触发弹道求解器 */
            return;
        }
        return;
    }

    if (key == '*') {
        /** * = 在高度差和温度字段上切换负号 */
        toggle_sign();
        return;
    }

    if (key == '#') {
        /** # = 添加小数点（仅限数值测量字段） */
        if (current_field >= FIELD_DIST && current_field <= FIELD_WIND_DIR) {
            append_char('.');
        }
        return;
    }

    if (key >= '0' && key <= '9') {
        /** 数字键：追加，对装药模式进行范围校验 */
        if (current_field == FIELD_CHARGE_MODE && (key < '1' || key > '5'))
            return; /**< 装药模式仅接受数字1-5 */
        append_char(key);
    }
}

/**
 * @简介 强制刷新显示（重新渲染当前字段）。
 *
 * 在可能破坏LCD状态的外部事件之后使用。
 */
void menu_update_display(void) {
    display_current_field();
}
