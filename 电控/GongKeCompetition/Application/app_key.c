/**
  * @file       app_key.c
  * @brief      按键模块：初始化 5 个按键，任务里轮询事件并更新菜单状态。
  *
  *             按键消抖由 1ms 时基里的 vKeyDevicePeriodExecute 完成，
  *             本任务只负责读取事件并分发。
  */

#include "app_key.h"
#include "app_beam.h"

/**
 * @brief      初始化按键模块
 * @retval       None
 */
void vAppKeyInit(void) {
    stKeyStaticParamTdf  stInit;
    stKeyRunningParamTdf stRun;

    memset(&stRun, 0, sizeof(stRun));

    /* 统一按键参数：低电平有效，20ms 消抖，1s 长按，300ms 双击间隔 */
    stInit.emValidLevel        = emKeyValidLevel_Low;
    stInit.ulDebounceThreshold = 20;
    stInit.ulLongPressThreshold   = 0;
    stInit.ulDoubleClickThreshold = 0;

    /* 确认按钮/启动按钮 */
    stInit.pstGpioBase = MAIN_BUTTON_GPIO_Port;   stInit.usGpioPin = MAIN_BUTTON_Pin;
    vKeyDeviceInit(&stInit, KEY_MAIN);            vKeyDeviceRunningParamInit(&stRun, KEY_MAIN);

    /* 左键 */
    stInit.pstGpioBase = LEFT_BUTTON_GPIO_Port;   stInit.usGpioPin = LEFT_BUTTON_Pin;
    vKeyDeviceInit(&stInit, KEY_LEFT);            vKeyDeviceRunningParamInit(&stRun, KEY_LEFT);

    /* 右键 */
    stInit.pstGpioBase = RIGHT_BUTTON_GPIO_Port;  stInit.usGpioPin = RIGHT_BUTTON_Pin;
    vKeyDeviceInit(&stInit, KEY_RIGHT);           vKeyDeviceRunningParamInit(&stRun, KEY_RIGHT);

    /* 上键 */
    stInit.pstGpioBase = UP_BUTTON_GPIO_Port;     stInit.usGpioPin = UP_BUTTON_Pin;
    vKeyDeviceInit(&stInit, KEY_UP);              vKeyDeviceRunningParamInit(&stRun, KEY_UP);

    /* 下键 */
    stInit.pstGpioBase = DOWN_BUTTON_GPIO_Port;   stInit.usGpioPin = DOWN_BUTTON_Pin;
    vKeyDeviceInit(&stInit, KEY_DOWN);            vKeyDeviceRunningParamInit(&stRun, KEY_DOWN);
}


/**
 * @brief       按键设备号 -> OLED 菜单位掩码
 * @param       emKey  按键设备号
 * @retval      对应的 OLED_KEY_* 位；菜单不认识的键回 0
 *
 * 菜单只认这 5 个键（也正是 KEY_MAIN..KEY_DOWN 这一段的全部）。
 */
static uint8_t ucKeyToOledMask(emKeyDevNumTdf emKey)
{
    switch (emKey) {
        case KEY_MAIN:  return OLED_KEY_MAIN;
        case KEY_LEFT:  return OLED_KEY_LEFT;
        case KEY_RIGHT: return OLED_KEY_RIGHT;
        case KEY_UP:    return OLED_KEY_UP;
        case KEY_DOWN:  return OLED_KEY_DOWN;
        default:        return 0u;
    }
}


/**
 * @brief      按键实现
 * @param       emKey  按键设备号
 * @retval       None
 */
void vKeyHandler(emKeyDevNumTdf emKey){
    /* ⚠️⚠️ **先读事件、立刻清事件, 然后才做有阻塞的事**（原来是读到末尾才清）。
     * 为什么: 后面任何分支只要做了有阻塞/耗时的事（历史上的 MAIN 蜂鸣就是 ~60ms）, 期间
     * 1ms 时基中断（prio 7）完全来得及把新的一次"按下-松开"再置成 Click —— 而末尾那句**无条件**
     * `vKeyClearEvent` 会把它抹掉 → **整个按键静默丢失**（菜单与脚本两条通道都没了）。
     * 现场表现正是最难查的「按了没反应」: 停在 STEP_DEAD / STEP_STOP / STEP_ARM_EDIT_WAIT
     * 或测试积木上的路线不往下走, 必须再按一次。
     * 顺序: 读 → 清 → 才干活。清在这里对**所有**事件类型都成立（与旧行为一致）。
     * ⚠️ 按住连发（数值调参快进快退, 2026-10-18 加）**不在这层** —— 在 OLED 任务的
     *    调参行分派里（ucKeyIsPressed 查按住态 + 只对数值类调参行合成按键）。
     *    放按键层会影响所有页面的单击/按住手感, 用户明确要求只对调参生效。 */
    emKeyEventTdf emEv = emKeyGetEvent(emKey);

    vKeyClearEvent(emKey);

    if (emEv == emKeyEvent_Click) {
        /* 先给 OLED 菜单投一份。与下面那个 switch 是**两条独立通道**:
         * 菜单有自己的位掩码（读后清零的那份归脚本任务, 不能被菜单偷走）。 */
        vOledKeyPost(ucKeyToOledMask(emKey));

        switch (emKey) {
            case KEY_MAIN:              /* 确认 */
                /* 菜单正占着 MAIN（OLED 不在第 1 页）时**不再**喂脚本 —— 否则
                 * 「在菜单里按 MAIN 选任务」会顺带把停在 STEP_STOP 的路线放走, 车突然动。
                 * 第 1 页（默认页）busy=0, 路线行为一字不变。 */
                if (ucGetOledMenuBusy() == 0u) {
                    vSetMenuConfirm();
                }
                break;

            case KEY_UP:                /* 菜单上翻 */
                if (iGetMenuIndex() > 0) {
                    vSetMenuIndex(iGetMenuIndex() - 1);
                }
                break;

            case KEY_DOWN:              /* 菜单下翻 */
                vSetMenuIndex(iGetMenuIndex() + 1);
                break;

            case KEY_RIGHT:             /* 姿态角显示翻页（0=Roll 1=Pitch 2=Yaw 循环） */
                vSetAttPage((iGetAttPage() + 1) % 3);
                break;

            case KEY_LEFT:              /* 预留 */
                break;
            default:
                break;
        }
    }
}


/**
 * @brief      按键任务
 * @param       argument  任务参数（未使用）
 * @retval      None
 */
void vKeyTask(void *argument) {
    (void)argument;
    emKeyDevNumTdf emKey;

    for (;;) {
        /* 轮询 5 个按键，按设备号 switch 分发 */
        for (emKey = KEY_MAIN; emKey <= KEY_DOWN; emKey++) {
            vKeyHandler(emKey);
        }
        /* 必须让出CPU：本任务优先级(2)高于OLED/传感器(1)，
           空转会饿死低优先级任务，表现为OLED只在按键(蜂鸣器阻塞)时刷新 */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
