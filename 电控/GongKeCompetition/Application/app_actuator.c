/**
  * @file       app_actuator.c
  * @brief      执行器控制：蜂鸣器、继电器（激光）、补光 PWM、蜂鸣任务（非阻塞请求执行 + 陀螺看门狗）。
  */

#include "app_actuator.h"
#include "app_beam.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"

/*
 * ⚠️ 两个引脚名跟随「工科赛集成板 V1.2」最后一版（2026-09-30）的改动，
 *    定义在 Core/Inc/main.h —— 别照旧版板子/旧笔记改回去：
 *      蜂鸣器 BEEP_CONTORL : PD7  -> PB13  （板上 PB13 紧挨 3.3V，走线短）
 *      激光   JIGUANG      : PD11 -> PD10  （PD10 原为线性 CCD 的 SI, CCD 已作废弃用）
 *    极性：蜂鸣器**高电平触发**（上电须为低 = 静音，见 MX_GPIO_Init 的 PinState）、
 *    激光高电平触发。换脚/换板时 .ioc / main.h / gpio.c 三处必须一起动, 否则 CubeMX 重生成会冲掉。
 */

/*
 * 补光灯独占定时器: TIM9_CH1 @ PE5（原挂 TIM3 只有 50Hz, 灯肉眼可见地闪）。
 * 与 UART TX DMA 同理手写在应用层、不进 CubeMX/.ioc, 重生成冲不掉;
 * TIM3 从此只挂舵机（50Hz）, 原 PA6/TIM3_CH1 的 MX 配置闲置不碍事（线已挪走）。
 *
 * ⚠️ 频率可调（FILL_LIGHT_PWM_HZ）:
 *    TIM9 挂 APB2（84MHz; APB2 分频≠1 ⇒ 定时器时钟 = 2×84 = 168MHz）。
 *    TIM9 是 **16 位**定时器（ARR ≤ 65535）, 所以先把时钟分频到 FILL_LIGHT_TICK_HZ
 *    再数 ARR —— 21kHz 时 PSC=0 也装得下, 但 1kHz 的 ARR=168000 装不下, 必须分频。
 * ⚠️ 为什么要能降频: **光耦隔离的 MOS 模块**规格通常只到 ≤1kHz（少数 ≤5kHz）,
 *    21kHz 下光耦来不及翻转 → 灯很暗甚至完全不亮; 直驱 MOSFET 则 21kHz 毫无问题。
 *    2026-10 现场排查"补光灯不亮"时默认值先从 21000 降到 1000 试。
 */
#define FILL_LIGHT_TIM_CLK_HZ   168000000u                              /* APB2 84MHz ×2 */
#define FILL_LIGHT_TICK_HZ      1000000u                                /* 1MHz 计时单元 */
#define FILL_LIGHT_PWM_HZ       20000u                                  /* 目标频率[Hz]（直驱/TB6612 用 20k; 100 仅排查 LED 阈值问题用） */
#define FILL_LIGHT_PSC          ((FILL_LIGHT_TIM_CLK_HZ / FILL_LIGHT_TICK_HZ) - 1u)   /* = 167 */
#define FILL_LIGHT_PERIOD       (FILL_LIGHT_TICK_HZ / FILL_LIGHT_PWM_HZ)              /* 计数跨度 */

TIM_HandleTypeDef htim9;

void vAppActuatorInit(void)
{
    GPIO_InitTypeDef stGpio = {0};
    TIM_OC_InitTypeDef stOc = {0};

    /* 蜂鸣器、继电器初始为关闭 */
    vBuzzerOff();
    vRelayOff();

    /* 补光 PWM（TIM9_CH1）: 先配 CCR=0（常灭）再启动输出 */
    __HAL_RCC_TIM9_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    htim9.Instance = TIM9;
    htim9.Init.Prescaler = FILL_LIGHT_PSC;
    htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim9.Init.Period = FILL_LIGHT_PERIOD - 1u; /* 频率见 FILL_LIGHT_PWM_HZ */
    htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim9);

    stGpio.Pin = GPIO_PIN_5;
    stGpio.Mode = GPIO_MODE_AF_PP;
    stGpio.Pull = GPIO_NOPULL;
    stGpio.Speed = GPIO_SPEED_FREQ_LOW;
    stGpio.Alternate = GPIO_AF3_TIM9;
    HAL_GPIO_Init(GPIOE, &stGpio);

    stOc.OCMode = TIM_OCMODE_PWM1;
    stOc.Pulse = 0;
    stOc.OCPolarity = TIM_OCPOLARITY_HIGH;
    stOc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim9, &stOc, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
}

void vBuzzerOn(void)
{
    HAL_GPIO_WritePin(BEEP_CONTORL_GPIO_Port, BEEP_CONTORL_Pin, GPIO_PIN_SET); /* 高电平触发 */
}

void vBuzzerOff(void)
{
    HAL_GPIO_WritePin(BEEP_CONTORL_GPIO_Port, BEEP_CONTORL_Pin, GPIO_PIN_RESET);
}

void vBuzzerBeep(uint16_t usOnMs, uint16_t usOffMs, uint8_t ucTimes)
{
    uint8_t i;
    for (i = 0; i < ucTimes; i++)
    {
        vBuzzerOn();
        vTaskDelay(pdMS_TO_TICKS(usOnMs));
        vBuzzerOff();
        vTaskDelay(pdMS_TO_TICKS(usOffMs));
    }
}

/* ==================== 蜂鸣任务（2026-10-27 加）====================
 * 非阻塞蜂鸣请求的唯一执行者（请求槽见 app_beam.h::vSetBuzzerReq）。
 * ⚠️ 为什么必须有这个任务: vBuzzerBeep 内部 vTaskDelay, 失败音整段 2.1s ——
 *    vRpiTask 禁阻塞（收帧节拍）、平移精调环禁阻塞（控制周期节拍）、OLED 任务
 *    禁阻塞（50ms 一拍, 见 app_oled.c 那条惯例）, 它们只能"投请求", 由本任务
 *    慢慢响。本任务优先级最低（APP_PRIO_OLED）: 被饿只是响得晚一点, 不影响别的。
 * ⚠️ 与直用口的关系: vBuzzerBeep 的直用方（底盘动作组的拒绝/超时长鸣、STEP_BEEP
 *    积木）与本任务并发写同一个 GPIO, 音型会叠 —— 两边都是"报警音", 叠了不难听,
 *    不做互斥（谁都不许被静音: 动作组那条「必须响」是语义承诺）。
 *
 * 顺带跑「陀螺仪损坏」看门狗（2026-10-27 加）:
 *   判据 = **姿态帧号停涨**（ulGetYawFrameCount, 与 app_line.c/app_chassis_action.c
 *   同一套帧号法 —— 别用「yaw 值不变」判, 分不清停摆与车没动）。
 *   · 只在**跑动**窗口内盯（脚本在跑 或 底盘动作组在播; 陀螺校准期间豁免 ——
 *     校准不出帧是预期, 不豁免每次校准必白响一声失败音）; 台架空闲时不响,
 *     且进窗口那一刻重置基线 —— 上电时陀螺就已经死透的车, 起跑立刻报。
 *   · 帧号停涨超过 BUZZER_YAW_STALE_MS（app_config.h）= 损坏 → 失败音一声;
 *     每次损坏事件只报一声（期间帧恢复哪怕一拍就重新武装 —— 抖动不至于连环响）。
 *   · 报警走自己的请求槽（不直接响）: 与收到的 Pi 蜂鸣命令共用同一条执行路径,
 *     本拍已在响的话下一拍接上。
 */
#define BUZZER_POLL_MS       20u    /* 请求轮询节拍[ms] */
#define BUZZER_FAIL_ON_MS    800u   /* 失败音: 单响时长（用户 2026-10-27 定: 0.8s×2 隔 0.5s） */
#define BUZZER_FAIL_GAP_MS   500u   /* 失败音: 两响间隔 */
#define BUZZER_FAIL_TIMES    2u
#define BUZZER_TICK_ON_MS    200u   /* 丢靶短滴: 一声 0.2s（同上定） */

void vBuzzerTask(void *argument)
{
    uint32_t   ulYawCnt      = 0u;
    TickType_t xYawTick      = 0u;
    uint8_t    ucYawWasRun   = 0u;   /* 上拍是否在跑动窗口（0→1 沿重置基线） */
    uint8_t    ucYawReported = 0u;   /* 本次停摆事件已报过（帧恢复即重新武装） */
    (void)argument;

    for (;;)
    {
        uint8_t ucPat = ucTakeBuzzerReq();
        /* 跑动窗口豁免陀螺校准（2026-10-31 补）: 校准期间有效源按文档口径不出帧,
         * 帧号必然停涨 —— 不豁免的话每次校准都会白响一声「陀螺仪损坏」。校准结束
         * ucRun 走 0→1 的沿会重置基线（下面 ucYawWasRun 块）, 顺带盖住恢复瞬态。 */
        uint8_t ucRun = (((ucGetScriptRunning() != 0u) || (ucGetChActBusy() != 0u))
                         && (ucGetGyroCalBusy() == 0u)) ? 1u : 0u;

        /* ---- 陀螺仪损坏看门狗（帧号法, 见段头） ---- */
        if (ucRun != 0u)
        {
            if (ucYawWasRun == 0u)
            {
                /* 刚进跑动窗口: 基线从现在起算 */
                ulYawCnt      = ulGetYawFrameCount();
                xYawTick      = xTaskGetTickCount();
                ucYawReported = 0u;
            }
            else
            {
                uint32_t ulNow = ulGetYawFrameCount();

                if (ulNow != ulYawCnt)
                {
                    ulYawCnt      = ulNow;
                    xYawTick      = xTaskGetTickCount();
                    ucYawReported = 0u;
                }
                else if ((ucYawReported == 0u)
                         && ((xTaskGetTickCount() - xYawTick)
                             > pdMS_TO_TICKS(BUZZER_YAW_STALE_MS)))
                {
                    vSetBuzzerReq(BUZZER_PAT_FAIL);
                    ucYawReported = 1u;
                }
            }
        }
        ucYawWasRun = ucRun;

        /* ---- 执行待响请求（阻塞完成整段音型; 期间新请求留在槽里, 响完接着响） ---- */
        switch (ucPat)
        {
        case BUZZER_PAT_FAIL:
            vBuzzerBeep(BUZZER_FAIL_ON_MS, BUZZER_FAIL_GAP_MS, BUZZER_FAIL_TIMES);
            break;
        case BUZZER_PAT_TICK:
            vBuzzerBeep(BUZZER_TICK_ON_MS, 0u, 1u);
            break;
        default:    /* NONE / 未知值: 忽略（未知值 vRpiTask 解析侧已拦, 这里是最后一道） */
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(BUZZER_POLL_MS));
    }
}

/* ⚠️ 这一对是激光的**唯一**写引脚路径（脚本积木 STEP_LASER_ON/OFF、vLaserShoot、
 * 上电初始化都经它们）, 所以顺便把状态写进 Beam —— OLED 第 3 页「开关激光」那行
 * 显示的、以及菜单按键判据读的, 都是这同一个量, 不会跟实物脱节。
 * 反过来, 谁要改激光都请走这两条, 别新开一条 HAL_GPIO_WritePin 的旁路。 */
void vRelayOn(void)
{
    vSetLaserOn(1u);
    HAL_GPIO_WritePin(JIGUANG_GPIO_Port, JIGUANG_Pin, GPIO_PIN_SET);   /* 高电平触发: 高=开 */
}

void vRelayOff(void)
{
    vSetLaserOn(0u);
    HAL_GPIO_WritePin(JIGUANG_GPIO_Port, JIGUANG_Pin, GPIO_PIN_RESET); /* 低=关 */
}

/*
 * 激光射击（脚本动作 STEP_LASER_SHOOT 的原语）: 开激光 → 等 ulMs → 关激光。
 * 把「开-等-关」封成一步, 免得路线里排三行（开、STEP_DELAY、关）——那三行的
 * 中间那段延时容易被后面改路线的人漏掉, 变成激光一直开着。
 * ⚠️ 延时用 vTaskDelay（让出式）而不是 HAL_Delay —— 后者会饿死按键/OLED/传感器
 *    任务, 跑路线时按不了急停（中断与并发铁律第 2 条）。
 */
void vLaserShoot(uint32_t ulMs)
{
    vRelayOn();
    vTaskDelay(pdMS_TO_TICKS(ulMs));
    vRelayOff();
}

void vFillLightSet(uint16_t usDuty)
{
    /* usDuty = 千分比 0~1000（不是百分比）; CCR 按当前周期 FILL_LIGHT_PERIOD 等比换算,
     * 改频率（FILL_LIGHT_PWM_HZ）不用动这里。>1000 静默钳到满占空。 */
    uint32_t ulCcr;

    if (usDuty > 1000u)
    {
        usDuty = 1000u;
    }
    ulCcr = ((uint32_t)usDuty * FILL_LIGHT_PERIOD) / 1000u;

    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, ulCcr);

    /* 顺手记进 Beam（钳位后的值）: 本函数是全工程**唯一**的补光占空比出口 ——
     * OLED「补光灯」开关的标签读的就是它, 写一次状态就永远等于实物（同
     * vRelayOn/vRelayOff 写 vSetLaserOn 那套; 四个写者全从这儿过, 没有旁路）。 */
    vSetFillDuty(usDuty);
}

/*
 * 任务补光（6500K 灯随任务步骤自动开关, 配置在 app_config.h「任务补光」区）。
 * 与脚本积木 STEP_BUGUANG_KAI/_GUAN 的分工: 积木是路线作者手动排的、开多久排多久;
 * 这一对是「任务步骤全程点亮、出步必灭」的自动行为, 唯一调用点在
 * app_script.c::vScriptExecStep（步骤→类别映射表也在那儿）。
 * ⚠️ 后写者胜: 任务步骤里手动开的灯会被步末的 vFillLightTaskEnd 灭掉,
 *    排路线别依赖「任务步骤开灯、过了这一步还亮着」。
 * ⚠️ 2026-10 起开关/亮度读 **Beam 运行时量**（app_config.h 的 FILL_TASK_* 只是上电
 *    默认）—— 「蓝牙」Tab「补光」组 / OLED「名字调参」改完, **下一个任务步骤就生效**,
 *    不用重编译; 现场调好把终值写回宏固化。PCT 哨兵语义原样: >100（含 255）=跟随默认。
 */
uint16_t vFillLightTaskBegin(uint8_t ucTask)
{
    uint8_t  ucEn;
    uint32_t ulPct;

    if (ucGetFillTaskEn() == 0u)
    {
        return 0u;                      /* 总开关关: 一个寄存器都不碰（手动积木开灯不受影响） */
    }

    ucEn = 0u;
    ulPct = 0u;
    switch (ucTask)
    {
    case FILL_TASK_ROAD:    ucEn = ucGetFillTaskRoadEn();  ulPct = ucGetFillTaskRoadPct();  break;
    case FILL_TASK_BALL:    ucEn = ucGetFillTaskBallEn();  ulPct = ucGetFillTaskBallPct();  break;
    case FILL_TASK_SHOOT:   ucEn = ucGetFillTaskShootEn(); ulPct = ucGetFillTaskShootPct(); break;
    case FILL_TASK_HOSTAGE: ucEn = ucGetFillTaskHsEn();    ulPct = ucGetFillTaskHsPct();    break;
    case FILL_TASK_QR:      ucEn = ucGetFillTaskQrEn();    ulPct = ucGetFillTaskQrPct();    break;
    default:
        return 0u;                      /* 不认识的类别: 不碰灯 */
    }

    if (ucEn == 0u)
    {
        return 0u;                      /* 这个任务不要补光 */
    }
    if (ulPct > 100u)
    {
        ulPct = ucGetFillTaskPct();     /* 未填（哨兵 255, 调参页渲染成空框 + 「默认」hint）或填飞了
                                         * → 用默认亮度。填 0 = 开灯但 0% 亮（等效灭, 合法值,
                                         * 「不亮」与「未填」从此分得开） */
    }
    ulPct *= 10u;                       /* % → 千分比（vFillLightSet 的口径） */
    vFillLightSet((uint16_t)ulPct);
    return (uint16_t)ulPct;             /* 实际点亮值交回去（调用方打调试串用） */
}

void vFillLightTaskEnd(void)
{
    vFillLightSet(0u);                  /* 无条件灭: EN=0/类别不识时 begin 没开过灯, 再灭一次无副作用 */
}
