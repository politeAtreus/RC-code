/*
  STM32F411 - Joystick Sail Controller → XBee Radio Link

  USART1 (XBee Radio):
      TX = PA9  → XBee DIN
      RX = PA10 ← XBee DOUT

  USART2 (ST-Link Serial Monitor via USB):
      TX = PA2
      RX = PA3

  Joystick ADC:
      X-axis = PA0 = ADC_CHANNEL_0   ← used for sail speed command
      Y-axis = PA1 = ADC_CHANNEL_1   ← reserved for future use (e.g. rudder)

  RX Change (v2):
      Replaced blocking HAL_UART_Receive poll with interrupt-driven ring buffer.
      HAL_UART_Receive_IT re-arms after every byte so no bytes are dropped
      during ADC reads or XBee transmits.
*/

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* -------------------- Config -------------------- */

#define ADC_CENTER      2048
#define ADC_MAX         2048.0f
#define DEADBAND        100

/* -------------------- Ring Buffer -------------------- */

#define RX_RING_SIZE    256

static volatile uint8_t  ring_buf[RX_RING_SIZE];
static volatile uint16_t ring_head = 0;   /* written by ISR  */
static volatile uint16_t ring_tail = 0;   /* read  by main   */
static          uint8_t  irq_byte;        /* single-byte ISR target */

/* -------------------- Handles -------------------- */

static UART_HandleTypeDef huart1;
static UART_HandleTypeDef huart2;
static ADC_HandleTypeDef  hadc1;
static uint16_t           adc_center = ADC_CENTER;

static char    xbee_rx_buf[256];   /* increased from 64  */
static uint16_t rx_index = 0;

/* -------------------- Prototypes -------------------- */

static void     XBee_UART_Init(uint32_t baud);
static void     Debug_UART_Init(uint32_t baud);
static void     ADC1_Init_A0_A1(void);
static uint16_t ADC1_ReadChannel(uint32_t channel);
static void     XBee_SendSailSpeed(int16_t angle);
static void     Debug_Printf(const char *fmt, ...);

/* -------------------- Main -------------------- */

int main(void)
{
  HAL_Init();
  XBee_UART_Init(9600);
  Debug_UART_Init(9600);
  ADC1_Init_A0_A1();

  HAL_Delay(200);

  /* calibrate joystick center at startup */
  adc_center = ADC1_ReadChannel(ADC_CHANNEL_0);
  Debug_Printf("Center calibrated: %u\r\n", adc_center);
  Debug_Printf("Sail Controller Ready\r\n");

  /* arm the interrupt — re-armed inside callback after every byte */
  HAL_UART_Receive_IT(&huart1, &irq_byte, 1);

  while (1)
  {
    /* --- read joystick X axis --- */
    uint16_t x_raw = ADC1_ReadChannel(ADC_CHANNEL_0);

    /* --- compute signed offset from calibrated center --- */
    int16_t offset = (int16_t)x_raw - (int16_t)adc_center;

    /* --- apply deadband --- */
    if (offset > -DEADBAND && offset < DEADBAND)
      offset = 0;

    /* --- map offset to sail angle ---
         offset = -ADC_MAX → -45 deg
         offset =  0       → 0 deg
         offset = +ADC_MAX → 45 deg             */
    float angle_f = (offset / ADC_MAX) * 45.0f;

    /* clamp to valid range in case ADC overshoots */
    if (angle_f < -45.0f)   angle_f = -45.0f;
    if (angle_f > 45.0f) angle_f = 45.0f;

    int16_t angle = (int16_t)angle_f;

    /* --- send joystick data to XBee --- */
    XBee_SendSailSpeed(angle);

    /* --- debug joystick output --- */
    Debug_Printf("x_raw=%u  offset=%d  angle=%d\r\n",
                 x_raw, offset, angle);

    /* --- drain ring buffer: process every byte the ISR captured --- */
    while (ring_tail != ring_head)
    {
      uint8_t b = ring_buf[ring_tail];
      ring_tail = (ring_tail + 1) % RX_RING_SIZE;

      if (b == '\n')
      {
        xbee_rx_buf[rx_index] = '\0';
        if (rx_index > 0)
          Debug_Printf("XBee RX: %s\r\n", xbee_rx_buf);
        rx_index = 0;
        memset(xbee_rx_buf, 0, sizeof(xbee_rx_buf));
      }
      else if (b != '\r' && rx_index < sizeof(xbee_rx_buf) - 1)
      {
        xbee_rx_buf[rx_index++] = b;
      }
      else if (rx_index >= sizeof(xbee_rx_buf) - 1)
      {
        /* buffer full without newline — reset to avoid corrupt data */
        rx_index = 0;
        memset(xbee_rx_buf, 0, sizeof(xbee_rx_buf));
        Debug_Printf("WARN: RX buffer overflow, cleared\r\n");
      }
    }

    HAL_Delay(500);
  }
}

/* -------------------- UART RX Interrupt Callback -------------------- */

/*
  Called by HAL after every single byte received on any UART.
  We check that it came from huart1 (XBee), push the byte into the ring
  buffer, then immediately re-arm the interrupt for the next byte.
  No bytes can be dropped regardless of what main() is doing.
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h)
{
  if (h->Instance == USART1)
  {
    ring_buf[ring_head] = irq_byte;
    ring_head = (ring_head + 1) % RX_RING_SIZE;
    HAL_UART_Receive_IT(&huart1, &irq_byte, 1);   /* re-arm immediately */
  }
}

/* -------------------- XBee Message -------------------- */

static void XBee_SendSailSpeed(int16_t angle)
{
  char msg[32];
  int  len = snprintf(msg, sizeof(msg), "{sa:0,ra:%d}\r\n", angle);
  HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)len, HAL_MAX_DELAY);
}

/* -------------------- UART Init -------------------- */

static void XBee_UART_Init(uint32_t baud)
{
  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef io = {0};
  io.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
  io.Mode      = GPIO_MODE_AF_PP;
  io.Pull      = GPIO_NOPULL;
  io.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  io.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &io);

  huart1.Instance          = USART1;
  huart1.Init.BaudRate     = baud;
  huart1.Init.WordLength   = UART_WORDLENGTH_8B;
  huart1.Init.StopBits     = UART_STOPBITS_1;
  huart1.Init.Parity       = UART_PARITY_NONE;
  huart1.Init.Mode         = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart1) != HAL_OK)
    while (1) {}

  /* enable USART1 interrupt in NVIC */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

static void Debug_UART_Init(uint32_t baud)
{
  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef io = {0};
  io.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
  io.Mode      = GPIO_MODE_AF_PP;
  io.Pull      = GPIO_NOPULL;
  io.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  io.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &io);

  huart2.Instance          = USART2;
  huart2.Init.BaudRate     = baud;
  huart2.Init.WordLength   = UART_WORDLENGTH_8B;
  huart2.Init.StopBits     = UART_STOPBITS_1;
  huart2.Init.Parity       = UART_PARITY_NONE;
  huart2.Init.Mode         = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart2) != HAL_OK)
    while (1) {}
}

/* -------------------- ADC -------------------- */

static void ADC1_Init_A0_A1(void)
{
  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef io = {0};
  io.Pin  = GPIO_PIN_0 | GPIO_PIN_1;
  io.Mode = GPIO_MODE_ANALOG;
  io.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &io);

  hadc1.Instance                   = ADC1;
  hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode          = DISABLE;
  hadc1.Init.ContinuousConvMode    = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion       = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
    while (1) {}
}

static uint16_t ADC1_ReadChannel(uint32_t channel)
{
  ADC_ChannelConfTypeDef cfg = {0};
  cfg.Channel      = channel;
  cfg.Rank         = 1;
  cfg.SamplingTime = ADC_SAMPLETIME_144CYCLES;

  if (HAL_ADC_ConfigChannel(&hadc1, &cfg) != HAL_OK)
    return 0;

  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 10);
  uint16_t v = (uint16_t)HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  return v;
}

/* -------------------- Debug Printf -------------------- */

static void Debug_Printf(const char *fmt, ...)
{
  char buf[300];   /* increased from 80 to fit full JSON + prefix */

  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  size_t len = strnlen(buf, sizeof(buf));
  HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
}

/* -------------------- IRQ Handler -------------------- */

/*
  Required: HAL expects this symbol in your project.
  If you are using STM32CubeIDE and already have it in stm32f4xx_it.c,
  remove this block and add  HAL_UART_IRQHandler(&huart1);  there instead.
*/
void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}

/* -------------------- SysTick -------------------- */

void SysTick_Handler(void)
{
  HAL_IncTick();
}