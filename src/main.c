/*
  STM32F411 - Joystick Sail Controller → XBee Radio Link

  USART1 (XBee Radio, 115200 8N1, AP=1, AO=0):
      TX = PA9  → XBee DIN
      RX = PA10 ← XBee DOUT

  USART2 (SailboatGUI via ST-Link USB, 115200 8N1):
      TX = PA2
      RX = PA3
      XBee RX: <telemetry line>
      XBee API: <hex for the entire validated frame, from 7E through checksum>

  Joystick ADC:
      X-axis = PA0 = ADC_CHANNEL_0   ← used for the rudder command
      Y-axis = PA1 = ADC_CHANNEL_1   ← reserved for future use
*/

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* -------------------- Config -------------------- */

#define ADC_CENTER      2048
#define ADC_MAX         2048.0f
#define DEADBAND        100
#define UART_BAUD       115200U
#define JOYSTICK_PERIOD_MS  500U

#define XBEE_MAX_RF_PAYLOAD   256U
#define XBEE_FRAME_HEADER    3U  /* delimiter and 16-bit length */
#define XBEE_FRAME_OVERHEAD  (XBEE_FRAME_HEADER + 1U) /* includes checksum */
#define XBEE_RX_HEADER       12U /* frame type, source addresses, options */
#define XBEE_TX_HEADER       14U /* frame type, ID, destination, radius, options */
#define XBEE_RX_FRAME_SIZE   (XBEE_FRAME_OVERHEAD + XBEE_RX_HEADER + XBEE_MAX_RF_PAYLOAD)
#define XBEE_FRAME_IDLE_MS   100U
#define XBEE_PAYLOAD_IDLE_MS 500U
#define XBEE_AT_TIMEOUT_MS   500U
#define XBEE_DIAGNOSTIC_MS   5000U
#define XBEE_TX_TIMEOUT_MS   20U
#define GUI_TX_TIMEOUT_MS    100U

enum {
  XBEE_START = 0x7E,
  XBEE_AT_REQUEST = 0x08,
  XBEE_TX_REQUEST = 0x10,
  XBEE_AT_RESPONSE = 0x88,
  XBEE_MODEM_STATUS = 0x8A,
  XBEE_RX_PACKET = 0x90
};

typedef enum {
  AT_DH, AT_DL, AT_DB, AT_EA, AT_ER, AT_TR, AT_VR
} XBeeAtCommand;

static const uint8_t at_names[][2] = {
  [AT_DH] = {'D', 'H'}, [AT_DL] = {'D', 'L'}, [AT_DB] = {'D', 'B'},
  [AT_EA] = {'E', 'A'}, [AT_ER] = {'E', 'R'}, [AT_TR] = {'T', 'R'},
  [AT_VR] = {'V', 'R'}
};
static const XBeeAtCommand diagnostics[] = {AT_EA, AT_ER, AT_TR, AT_VR};

/* -------------------- Ring Buffer -------------------- */

#define RX_RING_SIZE    1024U

static volatile uint8_t  ring_buf[RX_RING_SIZE];
static volatile uint16_t ring_head;       /* written by ISR */
static volatile uint16_t ring_tail;       /* written by main */
static volatile bool     ring_gap;        /* UART error, dropped byte, or failed re-arm */
static          uint8_t  irq_byte;        /* single-byte ISR target */

/* -------------------- Handles -------------------- */

static UART_HandleTypeDef huart1;
static UART_HandleTypeDef huart2;
static ADC_HandleTypeDef  hadc1;
static uint16_t           adc_center = ADC_CENTER;

/* -------------------- XBee Protocol State -------------------- */

static uint8_t  rx_frame[XBEE_RX_FRAME_SIZE];
static uint16_t rx_frame_used;
static uint16_t rx_frame_size;
static uint8_t  rx_checksum;
static uint32_t rx_last_byte;

static char     telemetry_line[XBEE_MAX_RF_PAYLOAD];
static uint16_t telemetry_used;
static bool     telemetry_discard;
static uint32_t telemetry_last_payload;

static uint8_t  boat_address[8];          /* big-endian DH followed by DL */
static uint8_t  boat_address_parts;       /* 0, 1, or 2 completed AT reads */
static uint32_t last_address_query;
static uint8_t  next_frame_id;
static struct {
  uint8_t id;
  XBeeAtCommand command;
  uint32_t sent;
} pending_at;
static bool     rssi_pending;
static uint32_t last_diagnostic;
static size_t   diagnostic_index;

/* -------------------- Prototypes -------------------- */

static void     XBee_UART_Init(uint32_t baud);
static void     Debug_UART_Init(uint32_t baud);
static void     ADC1_Init_A0_A1(void);
static uint16_t ADC1_ReadChannel(uint32_t channel);
static void     XBee_ReadFrames(void);
static void     XBee_ServiceAt(uint32_t now);
static void     XBee_SendJoystick(int16_t angle);
static void     Debug_Printf(const char *fmt, ...);

/* -------------------- Main -------------------- */

int main(void)
{
  HAL_Init();
  XBee_UART_Init(UART_BAUD);
  Debug_UART_Init(UART_BAUD);
  ADC1_Init_A0_A1();

  HAL_Delay(200);

  /* calibrate joystick center at startup */
  adc_center = ADC1_ReadChannel(ADC_CHANNEL_0);
  Debug_Printf("Center calibrated: %u\r\n", (unsigned)adc_center);
  Debug_Printf("Sail Controller Ready\r\n");

  /* Clear any overrun accumulated while the joystick was calibrating. */
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  if (HAL_UART_Receive_IT(&huart1, &irq_byte, 1) != HAL_OK)
    while (1) {}

  const uint32_t started = HAL_GetTick();
  uint32_t last_joystick = started - JOYSTICK_PERIOD_MS;
  last_address_query = started - XBEE_AT_TIMEOUT_MS;

  while (1)
  {
    XBee_ReadFrames();
    const uint32_t now = HAL_GetTick();
    XBee_ServiceAt(now);

    /* Keep servicing radio RX between joystick updates. */
    if (now - last_joystick < JOYSTICK_PERIOD_MS)
    {
      if (ring_tail == ring_head)
        __WFI();
      continue;
    }
    last_joystick = now;

    /* --- read joystick X axis --- */
    uint16_t x_raw = ADC1_ReadChannel(ADC_CHANNEL_0);

    /* --- compute signed offset from calibrated center --- */
    int16_t offset = (int16_t)x_raw - (int16_t)adc_center;

    /* --- apply deadband --- */
    if (offset > -DEADBAND && offset < DEADBAND)
      offset = 0;

    /* --- map offset to rudder angle ---
         offset = -ADC_MAX → -45 deg
         offset =  0       → 0 deg
         offset = +ADC_MAX → 45 deg             */
    float angle_f = (offset / ADC_MAX) * 45.0f;

    /* clamp to valid range in case ADC overshoots */
    if (angle_f < -45.0f) angle_f = -45.0f;
    if (angle_f > 45.0f) angle_f = 45.0f;

    int16_t angle = (int16_t)angle_f;

    /* --- send joystick data to XBee --- */
    if (boat_address_parts == 2U)
      XBee_SendJoystick(angle);

    /* --- debug joystick output --- */
    Debug_Printf("x_raw=%u  offset=%d  angle=%d\r\n",
                 (unsigned)x_raw, offset, angle);
  }
}

/* -------------------- UART RX Interrupt Callback -------------------- */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h)
{
  if (h != &huart1)
    return;

  const uint16_t next = (ring_head + 1U) % RX_RING_SIZE;
  if (next == ring_tail) {
    ring_gap = true;
  } else {
    ring_buf[ring_head] = irq_byte;
    ring_head = next;
  }

  if (HAL_UART_Receive_IT(h, &irq_byte, 1) != HAL_OK)
    ring_gap = true;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *h)
{
  if (h == &huart1) {
    __HAL_UART_CLEAR_OREFLAG(h);
    ring_gap = true;
  }
}

/* -------------------- XBee API Transmission -------------------- */

static uint8_t XBee_NextFrameId(void)
{
  if (++next_frame_id == 0U)
    ++next_frame_id;
  return next_frame_id;
}

static bool XBee_SendFrame(uint8_t *wire, uint16_t length)
{
  wire[0] = XBEE_START;
  wire[1] = (uint8_t)(length >> 8);
  wire[2] = (uint8_t)length;

  uint8_t sum = 0U;
  for (uint16_t i = 0U; i < length; ++i)
    sum = (uint8_t)(sum + wire[XBEE_FRAME_HEADER + i]);
  wire[XBEE_FRAME_HEADER + length] = (uint8_t)(0xFFU - sum);

  return HAL_UART_Transmit(&huart1, wire, length + XBEE_FRAME_OVERHEAD,
                           XBEE_TX_TIMEOUT_MS) == HAL_OK;
}

static void XBee_SendJoystick(int16_t angle)
{
  char msg[32];
  const int length = snprintf(msg, sizeof(msg), "{sa:0,ra:%d}\r\n", angle);
  if (length <= 0 || (size_t)length >= sizeof(msg))
    return;

  uint8_t wire[XBEE_FRAME_OVERHEAD + XBEE_TX_HEADER + sizeof(msg)];
  uint8_t *data = &wire[XBEE_FRAME_HEADER];
  data[0] = XBEE_TX_REQUEST;
  data[1] = XBee_NextFrameId();
  memcpy(&data[2], boat_address, sizeof(boat_address));
  data[10] = 0xFFU;
  data[11] = 0xFEU;  /* use the 64-bit destination */
  data[12] = 0U;    /* default broadcast radius */
  data[13] = 0U;    /* use the radio's TO setting */
  memcpy(&data[XBEE_TX_HEADER], msg, (size_t)length);
  XBee_SendFrame(wire, XBEE_TX_HEADER + (uint16_t)length);
}

/* -------------------- Local XBee AT Requests -------------------- */

static void XBee_SendAt(XBeeAtCommand command, uint32_t now)
{
  uint8_t wire[XBEE_FRAME_OVERHEAD + 4U];
  uint8_t *data = &wire[XBEE_FRAME_HEADER];
  data[0] = XBEE_AT_REQUEST;
  data[1] = XBee_NextFrameId();
  memcpy(&data[2], at_names[command], sizeof(at_names[command]));

  if (XBee_SendFrame(wire, 4U)) {
    pending_at.id = data[1];
    pending_at.command = command;
    pending_at.sent = now;
  }
}

static void XBee_ServiceAt(uint32_t now)
{
  if (pending_at.id != 0U && now - pending_at.sent < XBEE_AT_TIMEOUT_MS)
    return;
  pending_at.id = 0U;

  if (boat_address_parts < 2U) {
    /* Read the configured destination before sending any joystick commands. */
    if (now - last_address_query >= XBEE_AT_TIMEOUT_MS) {
      last_address_query = now;
      XBee_SendAt(boat_address_parts == 0U ? AT_DH : AT_DL, now);
    }
  } else if (now - last_diagnostic >= XBEE_DIAGNOSTIC_MS) {
    last_diagnostic = now;
    XBee_SendAt(diagnostics[diagnostic_index], now);
    diagnostic_index = (diagnostic_index + 1U) % (sizeof(diagnostics) / sizeof(diagnostics[0]));
  } else if (rssi_pending) {
    rssi_pending = false;
    XBee_SendAt(AT_DB, now);
  }
}

static void XBee_HandleAtResponse(const uint8_t *data, uint16_t length, uint32_t now)
{
  const XBeeAtCommand command = pending_at.command;
  if (length < 5U || pending_at.id == 0U || data[1] != pending_at.id ||
      memcmp(&data[2], at_names[command], sizeof(at_names[command])) != 0)
    return;
  pending_at.id = 0U;

  /* Other AT replies are forwarded to the GUI as raw frames. */
  if (command != AT_DH && command != AT_DL)
    return;

  const uint16_t value_length = (uint16_t)(length - 5U);
  if (data[4] != 0U || value_length == 0U || value_length > 4U)
    return;

  uint8_t *half = &boat_address[command == AT_DH ? 0U : 4U];
  memset(half, 0, 4U);
  memcpy(half + 4U - value_length, &data[5], value_length);
  if (command == AT_DH) {
    boat_address_parts = 1U;
    last_address_query = now - XBEE_AT_TIMEOUT_MS;
    return;
  }

  /* The boat needs a 64-bit unicast destination. */
  static const uint8_t reserved_prefix[6] = {0};
  boat_address_parts = memcmp(boat_address, reserved_prefix, sizeof(reserved_prefix)) == 0 ? 0U : 2U;
  last_diagnostic = now;
}

/* -------------------- XBee API Reception -------------------- */

static void XBee_ResetTelemetry(void)
{
  telemetry_used = 0U;
  telemetry_discard = false;
}

static void XBee_ForwardTelemetry(const uint8_t *data, uint16_t length, uint32_t now)
{
  if (now - telemetry_last_payload >= XBEE_PAYLOAD_IDLE_MS)
    XBee_ResetTelemetry();
  telemetry_last_payload = now;

  for (uint16_t i = 0U; i < length; ++i) {
    const uint8_t byte = data[i];
    if (byte == '\n') {
      if (!telemetry_discard && telemetry_used != 0U)
        Debug_Printf("XBee RX: %.*s\r\n", (int)telemetry_used, telemetry_line);
      XBee_ResetTelemetry();
    } else if (byte != '\r' && !telemetry_discard) {
      if (byte == '\0' || telemetry_used == sizeof(telemetry_line))
        telemetry_discard = true;
      else
        telemetry_line[telemetry_used++] = (char)byte;
    }
  }
}

#define GUI_API_PREFIX "XBee API: "

static void XBee_ForwardFrame(void)
{
  static const char hex[] = "0123456789ABCDEF";
  static char line[sizeof(GUI_API_PREFIX) - 1U + 2U * XBEE_RX_FRAME_SIZE + 2U] = GUI_API_PREFIX;
  size_t pos = sizeof(GUI_API_PREFIX) - 1U;

  for (uint16_t i = 0U; i < rx_frame_size; ++i) {
    const uint8_t byte = rx_frame[i];
    line[pos++] = hex[byte >> 4];
    line[pos++] = hex[byte & 0x0FU];
  }
  line[pos++] = '\r';
  line[pos++] = '\n';
  HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)pos, GUI_TX_TIMEOUT_MS);
}

static void XBee_HandleFrame(void)
{
  const uint8_t *data = &rx_frame[XBEE_FRAME_HEADER];
  const uint16_t length = (uint16_t)(rx_frame_size - XBEE_FRAME_OVERHEAD);
  const uint32_t now = HAL_GetTick();

  switch (data[0]) {
  case XBEE_RX_PACKET:
    if (length > XBEE_RX_HEADER && boat_address_parts == 2U &&
        memcmp(&data[1], boat_address, sizeof(boat_address)) == 0) {
      XBee_ForwardTelemetry(&data[XBEE_RX_HEADER], (uint16_t)(length - XBEE_RX_HEADER), now);
      rssi_pending = true;
    }
    break;

  case XBEE_AT_RESPONSE:
    XBee_HandleAtResponse(data, length, now);
    break;

  case XBEE_MODEM_STATUS:
    if (length == 2U && data[1] <= 1U) {  /* hardware or watchdog reset */
      boat_address_parts = 0U;
      pending_at.id = 0U;
      last_address_query = now - XBEE_AT_TIMEOUT_MS;
      diagnostic_index = 0U;
      rssi_pending = false;
      XBee_ResetTelemetry();
    }
    break;
  }

  /* This includes TX delivery status (0x8B) and local AT responses (0x88). */
  XBee_ForwardFrame();
}

static void XBee_ResetReceive(void)
{
  rx_frame_used = 0U;
  XBee_ResetTelemetry();
}

static void XBee_ConsumeByte(uint8_t byte)
{
  if (rx_frame_used == 0U && byte != XBEE_START)
    return;

  rx_last_byte = HAL_GetTick();
  rx_frame[rx_frame_used++] = byte;
  if (rx_frame_used == XBEE_FRAME_HEADER) {
    const uint16_t length = (uint16_t)(((uint16_t)rx_frame[1] << 8) | rx_frame[2]);
    if (length == 0U || length > sizeof(rx_frame) - XBEE_FRAME_OVERHEAD) {
      XBee_ResetReceive();
      return;
    }
    rx_frame_size = (uint16_t)(length + XBEE_FRAME_OVERHEAD);
    rx_checksum = 0U;
  } else if (rx_frame_used > XBEE_FRAME_HEADER) {
    /* In AP=1 a delimiter value inside the frame is ordinary data. */
    rx_checksum = (uint8_t)(rx_checksum + byte);
    if (rx_frame_used == rx_frame_size) {
      rx_frame_used = 0U;
      if (rx_checksum == 0xFFU)
        XBee_HandleFrame();
      else
        XBee_ResetTelemetry();
    }
  }
}

static void XBee_ReadFrames(void)
{
  if (ring_gap) {
    HAL_NVIC_DisableIRQ(USART1_IRQn);
    ring_tail = ring_head;
    ring_gap = false;
    if (huart1.RxState == HAL_UART_STATE_READY) {
      __HAL_UART_CLEAR_OREFLAG(&huart1);
      ring_gap = HAL_UART_Receive_IT(&huart1, &irq_byte, 1) != HAL_OK;
    }
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    XBee_ResetReceive();
  }

  if (rx_frame_used != 0U && HAL_GetTick() - rx_last_byte >= XBEE_FRAME_IDLE_MS)
    XBee_ResetReceive();

  /* Take a snapshot so a sustained burst cannot keep us in this loop. */
  const uint16_t head = ring_head;
  while (ring_tail != head && !ring_gap) {
    const uint8_t byte = ring_buf[ring_tail];
    ring_tail = (ring_tail + 1U) % RX_RING_SIZE;
    XBee_ConsumeByte(byte);
  }
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
  char buf[300];

  va_list args;
  va_start(args, fmt);
  int length = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if (length <= 0)
    return;
  if ((size_t)length >= sizeof(buf))
    length = sizeof(buf) - 1U;
  HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)length, GUI_TX_TIMEOUT_MS);
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
