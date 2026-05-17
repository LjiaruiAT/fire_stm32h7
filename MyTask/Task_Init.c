#include "Task_Init.h"
#include <math.h>
#define inv_tor 0.4
#define PI_F 3.14159265358979323846f
#define TWO_PI_F (2.0f * PI_F)
float unitree_F = 0;
float unitree_S = 0;
float unitree_T = 0;
float real_angle = 0;
static float twenty_to_real_pai(float angle)
{
	float temp = angle / 6.369426;
	return temp;
	
	
}
static float NormalizeAngleRad(float angle)
{
    float value = fmodf(angle, TWO_PI_F);
    if (value < 0.0f)
    {
        value += TWO_PI_F;
    }
    return value;
}

static float GravityCompensatedTorque360(float angle_current,
                                         float angle_down,
                                         float torque_max)
{
    float diff = NormalizeAngleRad(angle_current - angle_down);
    if (diff > PI_F)
    {
        diff -= TWO_PI_F;
    }
    else if (diff < -PI_F)
    {
        diff += TWO_PI_F;
    }

    return torque_max * sinf(diff);
}

typedef struct
{
    float exp_tor;
    float exp_pos;
    float exp_vel;
    float exp_kp;
    float exp_kd;
} exp_param;

typedef struct {
    GO_MotorHandle_t motor; 
    float pos_offset;         
    float inv_motor;         
    float exp_rad;           
    float exp_omega;          
    float exp_torque;         
    float Kp;                 
    float Kd;                
} Joint_t;

typedef enum {
    BALL_IDLE = 0,   
    BALL_PREPARE,   
    BALL_HIT,        
    BALL_RESET       
} BallState_t;

RS485_t rs485bus;
uint8_t dma1_send_buf[sizeof(GOMotor_SendPack_t)];
uint8_t dma1_recv_buf[sizeof(GOMotor_ReceivePack_t)];
Motor3508Ex_t Lift_Motor;
Joint_t let_fly = {.motor = {.motor_id = 0x01, .rs485 = &rs485bus}};
TaskHandle_t Hit_Task_Handle;


float balance_rad = 0;
float unitree_inv_back  = 20.00f;
float unitree_inv_front = 18.00f;
float balance_inv = 0.0f;
char  init_done = 0;
float max_vel = -300;
int      ret       = 0;
uint32_t error_cnt = 0;
uint32_t success_cnt = 0;

CubicParam_t     cubic;
TrajectoryState_t state;
float kkp = 0;
float kkd = 0;
static uint8_t uart7_dma_buf[64];
uint8_t bt_cmd = 0;
float test_angle = 0;

/* ---- Task_Init ---- */
void Task_Init(void)
{
    RS485Init(&rs485bus, &huart2, NULL, NULL, dma1_send_buf, dma1_recv_buf);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart7_dma_buf, sizeof(uart7_dma_buf));
    __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);

    xTaskCreate(Hit_Task,
                "Hit_Task",
                400,
                NULL,
                4,
                &Hit_Task_Handle);
}
float ini_rad = 0;
void Hit_Task(void *pvParameters)
{
    TickType_t last_wake  = xTaskGetTickCount();
    BallState_t ball_state = BALL_IDLE;
    TickType_t state_start = last_wake;
let_fly.Kp = 4.0f;
	let_fly.Kd =0.2f;
    let_fly.exp_torque = 0.4f;

    char rad_init_done  = 0;
    char cubic_generated = 0;

    while (1)
    {
        TickType_t now = xTaskGetTickCount();
			
if (init_done == 0)
{
			  GoMotorSend(&let_fly.motor,
                                0,
                                state.vel,
                                test_angle,
                                0,
                                0);
						            ret = GoMotorRecv(&let_fly.motor);

							 ret = GoMotorRecv(&let_fly.motor);
}
        if (!rad_init_done)
        {
            unitree_F = let_fly.motor.state.rad;
            unitree_S = unitree_F + unitree_inv_back;   
            unitree_T = unitree_F - unitree_inv_front; 
            ini_rad =	twenty_to_real_pai(let_fly.motor.state.rad);
            rad_init_done = 1;
        }
				real_angle = twenty_to_real_pai(let_fly.motor.state.rad);
        let_fly.exp_torque = GravityCompensatedTorque360(real_angle,
                                                        ini_rad,
                                                        inv_tor);
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
                    GoMotorSend(&let_fly.motor, let_fly.exp_torque, 0, unitree_S, let_fly.Kp, let_fly.Kd);
									ret = GoMotorRecv(&let_fly.motor);
                }
                else
                {
                    ball_state  = BALL_HIT;
                    state_start = now;
                }
                break;

            case BALL_HIT:
//							if (!cubic_generated)
//							{
//									Cubic_SetTrajectory(&cubic,
//																			let_fly.motor.state.rad, 0.0f,
//																			unitree_T, 0.0f,
//																			0.08f, HAL_GetTick());
//									cubic_generated = 1;
//							}
						
                if ((now - state_start) < pdMS_TO_TICKS(1000))
                {
//                    Cubic_GetFullState(&cubic, HAL_GetTick(), &state);
//                    GoMotorSend(&let_fly.motor,
//                                let_fly.exp_torque,
//                                state.vel,
//                                state.pos,
//                                let_fly.Kp,
//                                let_fly.Kd);
									
									
                                    if (let_fly.motor.state.rad > unitree_T)
									{
										GoMotorSend(&let_fly.motor,let_fly.exp_torque,max_vel,unitree_T,8,0);
																				ret = GoMotorRecv(&let_fly.motor);

									}
									else{
									GoMotorSend(&let_fly.motor,let_fly.exp_torque,0,unitree_T,4,0.1);
																		ret = GoMotorRecv(&let_fly.motor);

									}
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
                    GoMotorSend(&let_fly.motor, let_fly.exp_torque, 0, unitree_F, 3, 0.1f);
									ret = GoMotorRecv(&let_fly.motor);
                }
                else
                {
                    init_done  = 0;
                    ball_state = BALL_IDLE;
                    GoMotorSend(&let_fly.motor, let_fly.exp_torque, 0, unitree_F, 0, 0);
									ret = GoMotorRecv(&let_fly.motor);
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
        HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart7_dma_buf, sizeof(uart7_dma_buf));
        __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
    }
}
