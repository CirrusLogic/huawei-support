

#ifndef __CS35LXX_H__
#define __CS35LXX_H__

#include <cutils/log.h>
#include <sys/ioctl.h>


#define CS35LXX_MAGIC_NUMBER	0x32353536

#define SMARTPA_SPK_DAC_VOLUME			_IOWR(CS35LXX_MAGIC_NUMBER, 1,\
							void *)
#define SMARTPA_SPK_POWER_ON			_IOWR(CS35LXX_MAGIC_NUMBER, 2,\
							void *)
#define SMARTPA_SPK_POWER_OFF			_IOWR(CS35LXX_MAGIC_NUMBER, 3,\
							void *)
#define SMARTPA_SPK_BYPASS_DSP		_IOWR(CS35LXX_MAGIC_NUMBER, 4,\
							void *)
#define SMARTPA_SPK_SWITCH_CONFIGURATION	_IOWR(CS35LXX_MAGIC_NUMBER, 5,\
							void *)
#define SMARTPA_SPK_SWITCH_CALIBRATION		_IOWR(CS35LXX_MAGIC_NUMBER, 6,\
							void *)
#define SMARTPA_SPK_GET_R0			_IOWR(CS35LXX_MAGIC_NUMBER, 7,\
							void *)
#define SMARTPA_SPK_GET_F0			_IOWR(CS35LXX_MAGIC_NUMBER, 8,\
							void *)
#define SMARTPA_SPK_GET_CAL_STRUCT		_IOWR(CS35LXX_MAGIC_NUMBER, 9,\
							void *)
#define SMARTPA_SPK_SET_CAL_STRUCT		_IOWR(CS35LXX_MAGIC_NUMBER, 10,\
							void *)
#define SMARTPA_SPK_SET_AMBIENT			_IOWR(CS35LXX_MAGIC_NUMBER, 11,\
							void *)
#define SMARTPA_SPK_SET_R0			    _IOWR(CS35LXX_MAGIC_NUMBER, 12,\
							void *)							
#define SMARTPA_SPK_SWITCH_FIRMWARE		_IOWR(CS35LXX_MAGIC_NUMBER, 13,\
							void *)
#define SMARTPA_SPK_GET_R0_REALTIME		_IOWR(CS35LXX_MAGIC_NUMBER, 14,\
							void *)
#define SMARTPA_SPK_SET_DEFAULT_CALIB   _IOWR(CS35LXX_MAGIC_NUMBER, 15,\
							void *)
#define SMARTPA_SPK_GET_CALIB_STATE	    _IOWR(CS35LXX_MAGIC_NUMBER, 16,\
							void *)
#define SMARTPA_SPK_CALIBRATE			      _IOWR(CS35LXX_MAGIC_NUMBER, 17,\
							void *)
#define SMARTPA_SPK_CALIBRATE_STOP		  _IOWR(CS35LXX_MAGIC_NUMBER, 18,\
							void *)
#define SMARTPA_SPK_START_DIAGNOSTICS	  _IOWR(CS35LXX_MAGIC_NUMBER, 19,\
							void *)
#define SMARTPA_SPK_STOP_DIAGNOSTICS	  _IOWR(CS35LXX_MAGIC_NUMBER, 20,\
							void *)


#define CS_DEVICE "/dev/cs35lxx"
#define DEVICE_ADDRESS "6-0050"
#define MAX_BOX_NUM  6
#define AMBIENT 25
#define CS35LXX_R0_T_UNIFORM 23
#define CS35LXX_R0_K_COEF 0.00383

#define Z_TO_OHM(z) ((z) * 5.85714 / 8192.0)
#define MIN_CAL_Z 5
#define MAX_CAL_Z 9
#define MIN_F0 400
#define MAX_F0 1200
#define RE_DEFAULT     9790//7 OHM (Range 5-9)
#define F0_DEFAULT     1118907//800HZ(Range 400-1200)

enum smartpa_scene {
	MUSIC = 0,
	RINGTONE,
	RINGTONE_HS_SPK,
	VOICE,
	VOIP,
	LOWPOWER,
	MMI_PRI_L,
	ALGO_BYPASS,
	LOW_TEMP,
	CALIB,
	FM,
	SMARTPA_SCENE_MAX,
};
	
	
struct cs35lxx_calib_data {
	uint32_t temp;
	uint32_t rdc;
	uint32_t status;
	uint32_t checksum;
};
	
void cs35lxx_set_firmware_name(const char *vendor_name);
int cs35lxx_init(void);
void cs35lxx_deinit(void);
void cs35lxx_reg_dump(void);
int cs35lxx_speaker_on(unsigned int scene);
int cs35lxx_speaker_off(unsigned int scene);
int cs35lxx_get_re(unsigned int *re_array);
int cs35lxx_get_temp(int *temp_array);
int cs35lxx_get_r0(unsigned int *r0_array);
int cs35lxx_get_f0(unsigned int *f0_array);
void cs35lxx_dsp_bypass(bool enable);
void cs35lxx_calib_start(void);
void cs35lxx_calib_stop(void);
int cs35lxx_calibrate(void);
void cs35lxx_set_box_vendor(unsigned int box_vendor);
int cs35lxx_get_default_calib_state(void);
void cs35lxx_set_default_calib_value(int value);
void cs35lxx_set_calib_value(int re);
	
#endif //__CS35LXX_H__