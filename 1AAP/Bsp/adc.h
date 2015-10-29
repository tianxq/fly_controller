#include"stdint.h"

#define AVERAGE_FILTER_BUFFER_SIZE      10 //10
#define CHANNEL_NUM                     7

#define VERT_LEFT        GPIO_Pin_5
#define HORZ_LEFT        GPIO_Pin_6
#define VERT_RIGHT       GPIO_Pin_1
#define HORZ_RIGHT       GPIO_Pin_2
#define CAMERA_PITH      GPIO_Pin_4
#define NC_ADC1          GPIO_Pin_0
#define BATTEY_CHECK     GPIO_Pin_3

static struct _contol_sticks_level{
    uint16_t vert_left;
    uint16_t horz_left;
    uint16_t vert_right;
    uint16_t horz_right;
    uint16_t camera_pith;
    uint16_t nc_ADC1;
		uint16_t battey_check;
}ctrl_sticks_degree;


typedef struct _average_filter{
    uint16_t *rawData;
    uint16_t buffer[AVERAGE_FILTER_BUFFER_SIZE];
    uint8_t  index;
}averaget_filter_t;

extern uint16_t AD_Value[AVERAGE_FILTER_BUFFER_SIZE][CHANNEL_NUM];

void stm32_adc1_init(void);
