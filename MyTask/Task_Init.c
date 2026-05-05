#include "Task_Init.h"

typedef struct
{
    float exp_tor;
    float exp_pos;
    float exp_vel;
    float exp_kp;
    float exp_kd;
} exp_param;

typedef struct {
    GO_MotorHandle_t motor;    /**< 电机句柄（包含 ID、RS485 总线指针等） */
    float pos_offset;          /**< 角度零点偏移量 (rad)，用于校准机械安装误差 */
    float inv_motor;           /**< 电机方向标志：1-正转，-1-反转 */
    float exp_rad;             /**< 期望角度 (rad) */
    float exp_omega;           /**< 期望角速度 (rad/s) */
    float exp_torque;          /**< 期望力矩 (N·m) */
    float Kp;                  /**< 位置环比例增益 */
    float Kd;                  /**< 位置环微分增益 */
} Joint_t;

typedef enum {
    BALL_IDLE = 0,   // 等待触发
    BALL_PREPARE,    // 准备阶段
    BALL_HIT,        // 击球阶段
    BALL_RESET       // 复位阶段
} BallState_t;

/* ---- 全局变量 ---- */
RS485_t rs485bus;
uint8_t dma1_send_buf[sizeof(GOMotor_SendPack_t)];
uint8_t dma1_recv_buf[sizeof(GOMotor_ReceivePack_t)];
Motor3508Ex_t Lift_Motor;
Joint_t let_fly = {.motor = {.motor_id = 0x01, .rs485 = &rs485bus}};
TaskHandle_t Hit_Task_Handle;

float unitree_F = 0;
float unitree_S = 0;
float unitree_T = 0;
float unitree_inv_back  = 12.00f;
float unitree_inv_front = 18.00f;
char  init_done = 0;

int      ret       = 0;
uint32_t error_cnt = 0;
uint32_t success_cnt = 0;

CubicParam_t     cubic;
TrajectoryState_t state;

/* UART7 DMA 接收缓冲区（蓝牙帧协议：0xAA [CMD] 0x55） */
static uint8_t uart7_dma_buf[64];
uint8_t bt_cmd = 0;

/* ---- Task_Init ---- */
void Task_Init(void)
{
    RS485Init(&rs485bus, &huart2, NULL, NULL, dma1_send_buf, dma1_recv_buf);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart7_dma_buf, sizeof(uart7_dma_buf));
    /* 关闭半传输中断，避免帧未完成时触发回调 */
    __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);

    xTaskCreate(Hit_Task,
                "Hit_Task",
                400,
                NULL,
                4,
                &Hit_Task_Handle);
}

void Hit_Task(void *pvParameters)
{
    TickType_t last_wake  = xTaskGetTickCount();
    BallState_t ball_state = BALL_IDLE;
    TickType_t state_start = last_wake;

    let_fly.exp_torque = -0.42f;

    char rad_init_done  = 0;
    char cubic_generated = 0;

    while (1)
    {
        TickType_t now = xTaskGetTickCount();

        if (!rad_init_done)
        {
            GoMotorSend(&let_fly.motor, 0, 0, 0, 0, 0);
            ret = GoMotorRecv(&let_fly.motor);

            unitree_F = let_fly.motor.state.rad;
            unitree_S = unitree_F + unitree_inv_back;   // 准备阶段角度
            unitree_T = unitree_F - unitree_inv_front;  // 击球角度

            rad_init_done = 1;
        }

        switch (ball_state)
        {
            case BALL_IDLE:
                if (init_done) 
                {
                    state_start = now;
                    cubic_generated = 0;
                    ball_state = BALL_PREPARE;
                }
                break;

            case BALL_PREPARE:
                if ((now - state_start) < pdMS_TO_TICKS(500))
                {
                    GoMotorSend(&let_fly.motor, 0, 0, unitree_S, 4, 0.1f);
                }
                else
                {
                    ball_state  = BALL_HIT;
                    state_start = now;
                }
                break;

            case BALL_HIT:
                if (!cubic_generated)
                {
                    Cubic_SetTrajectory(&cubic,
                                        let_fly.motor.state.rad, 0.0f,
                                        unitree_T, 0.0f,
                                        0.1f, HAL_GetTick());
                    cubic_generated = 1;
                }
                if ((now - state_start) < pdMS_TO_TICKS(300))
                {
                    Cubic_GetFullState(&cubic, HAL_GetTick(), &state);
                    GoMotorSend(&let_fly.motor,
                                let_fly.exp_torque,
                                state.vel,
                                state.pos,
                                5,
                                0.1f);
                }
                else
                {
                    ball_state  = BALL_RESET;
                    state_start = now;
                }
                break;

            case BALL_RESET:
                if ((now - state_start) < pdMS_TO_TICKS(1000))
                {
                    GoMotorSend(&let_fly.motor, 0, 0, unitree_F, 3, 0.1f);
                }
                else
                {
                    init_done  = 0;
                    ball_state = BALL_IDLE;
                    GoMotorSend(&let_fly.motor, 0, 0, unitree_F, 0, 0);
                }
                break;

            default:
                ball_state = BALL_IDLE;
                break;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5));
    }
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        RS485SendIRQ_Handler(&rs485bus, huart);
    }
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == UART7)
    {
        /* 校验蓝牙帧：0xAA [CMD] 0x55 */
        if (size >= 3 &&
            uart7_dma_buf[0]        == 0xAA &&
            uart7_dma_buf[size - 1] == 0x55)
        {
            bt_cmd = uart7_dma_buf[1];
            if (bt_cmd == 0x01)
            {
                init_done = 1;
                success_cnt++;
            }
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart7_dma_buf, sizeof(uart7_dma_buf));
        __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
    }
    if (huart->Instance == USART2)
    {
        RS485RecvIRQ_Handler(&rs485bus, huart, size);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        __HAL_UART_CLEAR_FLAG(huart,
                              UART_CLEAR_OREF |
                              UART_CLEAR_FEF  |
                              UART_CLEAR_NEF  |
                              UART_CLEAR_PEF);
        __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
        error_cnt++;
    }
    if (huart->Instance == UART7)
    {
        /* UART7 出错后重新启动 DMA 接收，保证蓝牙通信不中断 */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart7_dma_buf, sizeof(uart7_dma_buf));
        __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
    }
}
