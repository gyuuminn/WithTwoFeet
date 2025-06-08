/*  bsp_uart.c  */
#include "bsp_uart.h"
#include "bsp.h"

#define ENABLE_UART_DMA   1          /* 송신 DMA 사용 여부 ------------------ */

#define UART_HEADER       0xAA       /* 프로토콜 헤더                         */
#define CMD_MODE          0x10       /* 모드 전용 명령 코드                   */

/* ───── 외부 전역(다른 모듈에서 선언) ────────────────────────────── */
extern UART_HandleTypeDef huart1;
extern float setvel;                       /* 주행 속도 목표치            */
extern void Motor_Set_Pwm(uint8_t id,int16_t pwm); /* 모터 PWM 함수               */
/* ───── 내부 전역 ──────────────────────────────────────────────── */
uint8_t RxTemp;                     /* 수신 1바이트 임시 저장      */
volatile uint8_t RxComplete = 0;

static enum {                              /* 수신 상태머신               */
    S_WAIT_HDR,
    S_WAIT_CMD,
    S_WAIT_PAYLOAD
} rx_state = S_WAIT_HDR;

/* ───── 내부 함수 프로토타입 ───────────────────────────────────── */
static void uart_parser(uint8_t byte);

/* =================================================================
 *                    초기화 / 송신 유틸
 * ================================================================= */
void USART1_Init(void)
{
    HAL_UART_Receive_IT(&huart1, &RxTemp, 1);          /* 1바이트 인터럽트 대기 */
}

void USART1_Send_U8(uint8_t ch)
{
    HAL_UART_Transmit(&huart1, &ch, 1, 0xFFFF);
}

void USART1_Send_ArrayU8(uint8_t *buf, uint16_t len)
{
#if ENABLE_UART_DMA
    HAL_UART_Transmit_DMA(&huart1, buf, len);
#else
    while (len--)
        USART1_Send_U8(*buf++);
#endif
}

/* =================================================================
 *                  수신 인터럽트 콜백  (1바이트마다 호출)
 * ================================================================= */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
        return;

    uart_parser(RxTemp);                               /* 수신 바이트 해석 */

    HAL_UART_Receive_IT(&huart1, &RxTemp, 1);          /* 다음 바이트 대기 */
}

/* =================================================================
 *              3-단계 상태머신  (0xAA → CMD → Payload 1B)
 * ================================================================= */
static void uart_parser(uint8_t byte)
{
    static uint8_t cmd;

    switch (rx_state)
    {
    case S_WAIT_HDR:
        if (byte == UART_HEADER)
            rx_state = S_WAIT_CMD;
        break;

    case S_WAIT_CMD:
        cmd = byte;
        rx_state = (cmd == CMD_MODE) ? S_WAIT_PAYLOAD
                                     : S_WAIT_HDR;    /* 알 수 없는 CMD → 리셋 */
        break;

    case S_WAIT_PAYLOAD:
        if (cmd == CMD_MODE) {
            RxComplete = 1;
        }

        rx_state = S_WAIT_HDR;
        break;
    }
}

/* =================================================================
 =================
 *         printf() 를 UART1 으로 리다이렉트 (옵션)
 * ================================================================= */
#ifdef __GNUC__
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}