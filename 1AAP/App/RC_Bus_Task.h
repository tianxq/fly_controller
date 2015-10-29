/*******************************************************************************
 *  @file:RC_Bus_Profile.h
 */
#ifndef __RC_BUS_PROFILE_H
#define __RC_BUS_PROFILE_H

#ifndef __stdint_h
 #include <stdint.h>
#endif

/*******************************************************************************
 *  Macro
 */
#define RC_PROFILE_PARAM_ADDR                       (FLASH_BASE + FLASH_PAGE_SIZE*127)
/*******************************************************************************
 *  Type
 */
#pragma anon_unions
struct RC_Profile_Cfg_t {
    

    uint16_t ADC_Val_Max[4];
    uint16_t ADC_Val_Min[4];

    union {
        uint8_t chann_reverse;
        struct {
            uint8_t chann1_value_reverse:1;
            uint8_t chann2_value_reverse:1;
            uint8_t chann3_value_reverse:1;
            uint8_t chann4_value_reverse:1;
        };
    };
	
	uint8_t valid_chk;
	uint8_t enable;
	uint8_t reseved;
};



#pragma anon_unions
#pragma pack(1)
typedef union {

    struct {
         uint16_t head;
         uint8_t  type;
         uint8_t  length;
         uint16_t adc[6];

		 uint8_t mode:2;
		 uint8_t gohome:2;
		 uint8_t one_key_fly:2;
		 uint8_t photo:2;
		 uint8_t key_a:2;
		 uint8_t key_b:2;

		 uint8_t chk;
    };

	uint8_t buffer[19];

}rc_info_pkt_t;



/*******************************************************************************
 *  Decalre
 */
extern struct RC_Profile_Cfg_t RC_Profile_Cfg;

void hal_rc_bus_init(void);
void RC_BUS_Strobe_Write(uint8_t *rxbuf, uint8_t len);

#endif //__RC_BUS_PROFILE_H
