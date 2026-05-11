#include <linux/can.h> // // struct can_frame
#include <math.h>      // fabs

#include <stddef.h>      // offsetof
#include <stdint.h>      // uint64_t
#include <stdio.h>       // snprintf
#include <stdlib.h>      // malloc
#include <string.h>      // memset
#include <termios.h>     // tcflush
#include <unistd.h>      // pipe
#include "compiler.h"    // __visible
#include "list.h"        // list_add_tail
#include "msgblock.h"    // message_alloc
#include "pollreactor.h" //
#include "pyhelper.h"    // get_monotonic
#include "serialqueue.h" // struct queue_message

#include <stddef.h>
#include <stdint.h>
#include "dspMemOps.h"
#include "serial.h"

#define RECEIVE_WINDOW_USB401 192
#define MCU_TYPE_USB401 2
#define CLOCK_FREQ_USB401 84000000.0

#define SERIAL_BAUD_USB401 0 // 10 / 115200 //0.00001
#define PWM_MAX_USB401 2000000
#define ADC_MAX_USB401 4095
#define STATS_SUMSQ_BASE_USB401 256

extern struct DspMemOps smp_Uart_Ops;
int init_Uart()
{
    smp_Uart_Ops.fd_write = -1;
    smp_Uart_Ops.receive_window = RECEIVE_WINDOW_USB401;
    smp_Uart_Ops.mcu_type = MCU_TYPE_USB401;
    smp_Uart_Ops.clock_freq = CLOCK_FREQ_USB401;
    smp_Uart_Ops.serial_baud = SERIAL_BAUD_USB401;
    smp_Uart_Ops.pwm_max = PWM_MAX_USB401;
    smp_Uart_Ops.adc_max = ADC_MAX_USB401;
    smp_Uart_Ops.stats_sumsq_base = STATS_SUMSQ_BASE_USB401;
    return 0;
}
int re_init_Uart()
{
    return 0;
}

struct DspMemOps smp_Uart_Ops =
    {
        init : init_Uart,
        re_init : re_init_Uart,
        fd_write_type : SQT_UART,

        receive_window : RECEIVE_WINDOW_USB401,
        mcu_type : MCU_TYPE_USB401,
        clock_freq : CLOCK_FREQ_USB401,
        serial_baud : SERIAL_BAUD_USB401,
        pwm_max : PWM_MAX_USB401,
        adc_max : ADC_MAX_USB401,
        stats_sumsq_base : STATS_SUMSQ_BASE_USB401,

    };

struct DspMemOps *GetUartOps()
{
    return &smp_Uart_Ops;
}
