#include "interface.h"

// 状态机状态
typedef enum {
    STATE_IDLE,      // 等待帧头 0x5A
    STATE_LEN,       // 等待长度字节
    STATE_DATA,      // 接收6字节数据
    STATE_CHECKSUM   // 接收校验和
} rx_state_t;

// 私有变量（volatile 因在中断和主循环间共享）
static UART_HandleTypeDef *uart_handle = NULL;          // UART句柄
static volatile rx_state_t state = STATE_IDLE;          // 当前状态
static volatile uint8_t data_buffer[6];                 // 暂存x(2),y(2),area(2)
static volatile uint8_t data_index = 0;                 // 数据缓冲区索引
static volatile uint16_t last_x = 0;                    // 最新有效x
static volatile uint16_t last_y = 0;                    // 最新有效y
static volatile uint16_t last_area = 0;                 // 最新有效面积
static volatile uint8_t data_ready = 0;                 // 新数据标志
static volatile uint8_t rx_byte;                        // 单字节接收缓冲区

// 内部函数
static void parse_byte(uint8_t byte);
static void reset_state(void);

//------------------------------------------------------------------------------
void uart_receiver_init(UART_HandleTypeDef *huart) {
    uart_handle = huart;
    reset_state();
    last_x = 0;
    last_y = 0;
    last_area = 0;
    data_ready = 0;
    rx_byte = 0;
}

//------------------------------------------------------------------------------
void uart_receiver_start(void) {
    if (uart_handle != NULL) {
        // 启动第一次接收（接收一个字节）
        HAL_UART_Receive_IT(uart_handle, (uint8_t*)&rx_byte, 1);
    }
}

//------------------------------------------------------------------------------
// 状态机解析字节
static void parse_byte(uint8_t byte) {
    switch (state) {
        case STATE_IDLE:
            if (byte == 0x5A) {          // 检测到帧头
                state = STATE_LEN;
            }
            break;

        case STATE_LEN:
            if (byte == 0x06) {          // 长度正确
                state = STATE_DATA;
                data_index = 0;
            } else {                      // 长度错误，重新同步
                state = STATE_IDLE;
            }
            break;

        case STATE_DATA:
            if (data_index < sizeof(data_buffer)) {
                data_buffer[data_index++] = byte;
                if (data_index == sizeof(data_buffer)) {
                    state = STATE_CHECKSUM;
                }
            }
            break;

        case STATE_CHECKSUM:
        {
            // 计算校验和 = 帧头 + 长度 + 所有数据
            uint8_t checksum = 0x5A + 0x06;
            for (int i = 0; i < sizeof(data_buffer); i++) {
                checksum += data_buffer[i];
            }
            if (checksum == byte) {       // 校验通过
                // 大端数据转换
                last_x = (data_buffer[0] << 8) | data_buffer[1];
                last_y = (data_buffer[2] << 8) | data_buffer[3];
                last_area = (data_buffer[4] << 8) | data_buffer[5];
                data_ready = 1;            // 标记新数据可用
            }
            // 无论校验成功与否，回到空闲状态准备下一个包
            state = STATE_IDLE;
            break;
        }

        default:
            reset_state();
            break;
    }
}

//------------------------------------------------------------------------------
// 重置状态机（可留作备用，当前未使用）
static void reset_state(void) {
    state = STATE_IDLE;
    data_index = 0;
}

//------------------------------------------------------------------------------
bool uart_receiver_get_data(uint16_t *x, uint16_t *y, uint16_t *area) {
    bool ret = false;
    // 进入临界区，防止中断更新 data_ready 和 last_* 变量
    __disable_irq();
    if (data_ready) {
        if (x != NULL) *x = last_x;
        if (y != NULL) *y = last_y;
        if (area != NULL) *area = last_area;
        data_ready = 0;   // 清除标志
        ret = true;
    }
    __enable_irq();
    return ret;
}

//------------------------------------------------------------------------------
// HAL库UART接收完成回调（在中断中执行）
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == uart_handle) {
        // 解析刚收到的字节
        parse_byte(rx_byte);
        // 继续接收下一个字节
        HAL_UART_Receive_IT(uart_handle, (uint8_t*)&rx_byte, 1);
    }
}
