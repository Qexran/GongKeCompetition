/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    freertos.c
  * @brief   FreeRTOS application: task creation and startup.
  *
  *          Bare FreeRTOS API (xTaskCreate / vTaskDelay / ...), FreeRTOS V11.3.0.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "freertos_app.h"

#include "app_key.h"
#include "app_oled.h"
#include "app_actuator.h"
#include "app_arm.h"
#include "app_motor.h"
#include "app_sensors.h"
#include "app_comm.h"
#include "app_bt.h"
#include "app_rpi.h"
#include "app_script.h"
#include "app_pid.h"
#include "app_chassis_action.h"

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/**
  * @brief  Creates the FreeRTOS tasks. Called from main() before the scheduler
  *         is started.
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN RTOS_MUTEX */
  /* Add mutexes, semaphores, queues... */

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */

  /* USER CODE END RTOS_QUEUES */

  /* USER CODE BEGIN RTOS_THREADS */
  /* 按类型划分任务，优先级从高到低：通信/电机 > 机械臂 > 按键 > 显示 */

  /* 通信类任务 */
  /* rpi 栈 384 字（2026-10 从 256 提）: vRpiTask 最深链 = astT[3] 快照 + acDbg[32]
   * + snprintf + vRpiSendFrame 的 aucFrame[69], 标准库 vsnprintf 一层就要 400~800B,
   * 1KB 贴上限（CHECK_FOR_STACK_OVERFLOW=2 能兜住, 但别赌）。 */
  xTaskCreate(vRpiTask,     "rpi",     384, NULL, APP_PRIO_COMM,  NULL);
  /* 栈 512 字：$B 协议要行累积/分词/浮点格式化/LIST 逐行组装, 不是原来那个丢帧桩 */
  xTaskCreate(vBtTask,      "bt",      512, NULL, APP_PRIO_COMM,  NULL);
  xTaskCreate(vExtTask,     "ext",     256, NULL, APP_PRIO_COMM,  NULL);
  xTaskCreate(vSensorTask,  "sensor",  256, NULL, APP_PRIO_SENSOR, NULL);
  /* 副陀螺口（USART2，板上丝印 VICE_GYRO）：优先级与主口相同(1) —— 只阻塞等
   * 信号量、每帧几个字节的解析；**不能高过按键(2)**，否则 ATK 那条最长 500ms 的阻塞读
   * 会拖住急停。两个传感器任务都在跑（各跑自己那个口、按 GYRO_MAIN_DEV/GYRO_SUB_DEV
   * 分派设备），由**有效源** gyro_src_eff 决定谁发布姿态（见 app_sensors.c；名字里的
   * M0 是历史名，它跑的不一定是 M0）。 */
  xTaskCreate(vSensorTaskM0, "sensor_m0", 256, NULL, APP_PRIO_SENSOR, NULL);

  /* 底盘电机 */
  xTaskCreate(vMotorTask,   "motor",   256, NULL, APP_PRIO_MOTOR, NULL);

  /* 机械臂 */
  xTaskCreate(vArmTask,     "arm",     256, NULL, APP_PRIO_ARM,   NULL);

  /* 按键 */
  xTaskCreate(vKeyTask,     "key",     256, NULL, APP_PRIO_KEY,   NULL);

  /* 角度 PID 周期任务（50ms 节拍；原 TIM5 中断，周期处理含阻塞发送/延时不能进中断） */
  xTaskCreate(vPidTask,     "pid",     256, NULL, APP_PRIO_PID,   NULL);

  /* OLED 显示（较低优先级）
   * oled 栈 384 字（2026-10 从 256 提）: vOledDrawRowf 的 acBuf[44]+vsnprintf 与
   * OLED.c vOledPrintf 的 %f 链是全工程最深的 printf 用法, 1KB 贴上限。 */
  xTaskCreate(vOledTask,    "oled",    384, NULL, APP_PRIO_OLED,  NULL);

  /* 脚本运行器（栈 512 字：调用链 chassis→EMM 较深，且含浮点运动估时） */
  xTaskCreate(vScriptTask,  "script",  512, NULL, APP_PRIO_SCRIPT, NULL);

  /* 底盘动作组预览播放器（2026-10 加, app_chassis_action.c）。
   * 为什么独立任务而不阻塞在 vBtTask 里: $B,CPLAY 必须非阻塞 —— 车在跑的时候
   * $B,CSTOP 要随时能进来（机械臂 $B,PLAY 那种阻塞式对底盘是没刹车）。
   * 栈 512 字: 播放器含浮点运动估时 + snprintf 遥测（CA: 行 ~60B）, 与脚本任务
   * 同量级; 优先级与脚本同级(3) —— 同是底盘的速度流, 高过它会饿脚本, 低过它
   * 会被 OLED 抢占。路线跑的播放走脚本任务, 本任务只在预览时干活。 */
  xTaskCreate(vChassisActionTask, "chact", 512, NULL, APP_PRIO_SCRIPT, NULL);

  /* 蜂鸣任务（2026-10-27 加, app_actuator.c）: 消费 Beam 蜂鸣请求槽（Pi 0x0C 命令帧 /
   * 固件各失败点）并执行音型, 顺带跑陀螺仪损坏看门狗。优先级最低（1, 与 OLED 同级）
   * —— 被饿只是响得晚, 绝不能抢电机/通信; 栈 256 字: 无 printf、无大缓冲,
   * 最深只有 vBuzzerBeep 的 vTaskDelay 链。 */
  xTaskCreate(vBuzzerTask,  "buzz",   256, NULL, APP_PRIO_OLED,  NULL);

  /* USER CODE END RTOS_THREADS */
}

/**
  * @brief  Called if a task overflows its stack (configCHECK_FOR_STACK_OVERFLOW = 2).
  * @param  xTask:      Handle of the offending task.
  * @param  pcTaskName: Name of the offending task.
  * @retval None
  */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;

  /* Trap here in debugger / add your recovery or logging code. */
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
