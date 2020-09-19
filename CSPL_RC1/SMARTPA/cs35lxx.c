

#define LOG_TAG "cs35lxx"


#include "cs35lxx.h"
#include "securec.h"
#include "securectype.h"
#include "smartpakit.h"
#include "oeminfo.h"
#include <math.h>




#define TEMP_MULTIPLE 65536
#define FW_NAME_SIZE   50
#define FW_NAME_BASE  "/vendor/firmware/cs35lxx"


static int dev_node_fd = -1;
static unsigned int g_re_value;
static unsigned int g_box_vendor;
static char g_firmware_name[FW_NAME_SIZE];
static bool is_dsp_bypass = false;
typedef struct calib_oeminfo calib_oeminfo_t;
static calib_oeminfo_t* calib_oeminfo_cs = NULL;

void cs35lxx_set_box_vendor(unsigned int box_vendor)
{
    g_box_vendor = box_vendor;
}
 
void    cs35lxx_set_firmware_name(const char *vendor_name)
{
     
     int ret;

     if(vendor_name == NULL) {
         ALOGE("%s: vendor_name is NULL", __FUNCTION__);
         return;
     }
     ret = snprintf_s(g_firmware_name, FW_NAME_SIZE, FW_NAME_SIZE - 1, "%s_%s.cnt",
                                        FW_NAME_BASE, vendor_name);
    if (ret < 0)
        ALOGE("%s: snprintf_s error", __FUNCTION__);

    ALOGD("%s: g_firmware_name is %s", __FUNCTION__, g_firmware_name);
}



void cs35lxx_dsp_bypass(bool enable)
{

    if (is_dsp_bypass != enable) {
    	is_dsp_bypass = enable;
    	ALOGD("%s: SMARTPA_SPK_BYPASS_DSP succ, enable = %d", __FUNCTION__, enable);
    } else {
        if (is_dsp_bypass)
            ALOGD("%s: Already enabled bypass", __FUNCTION__);
        else
            ALOGD("%s: Already disabled bypass", __FUNCTION__);
    }
}    
    
int cs35lxx_init(void)
{   
    int ret;
    dev_node_fd = open(CS_DEVICE, O_RDWR | O_NONBLOCK);
    if (dev_node_fd < 0) {
        ALOGE("%s: can not open device %s", __FUNCTION__, CS_DEVICE);
        return -1;
    }
		calib_oeminfo_cs = (calib_oeminfo_t *)malloc(sizeof(calib_oeminfo_t));
    ret = rmt_oeminfo_read(OEMINFO_AUDIO_SMARTPA_CALIBRATION,
                               (unsigned int)sizeof(calib_oeminfo_t),
                               (unsigned char *)calib_oeminfo_cs);
    if ((ret < 0) || (calib_oeminfo_cs->calibration_value_f[0] < MIN_CAL_Z) ||
				(calib_oeminfo_cs->calibration_value_f[0] > MAX_CAL_Z)) {
				ALOGE("%s: rmt_oeminfo_read calibration failed or value invalid, set it to default", __FUNCTION__);
				calib_oeminfo_cs->calibration_value[0] = RE_DEFAULT;
				calib_oeminfo_cs->calibration_value_f[0] = Z_TO_OHM(RE_DEFAULT);
    }   
    //ret = ioctl(dev_node_fd, SMARTPA_SPK_SWITCH_FIRMWARE, &g_box_vendor);
    //if (ret < 0) {
    //        ALOGE("%s: SMARTPA_SPK_SWITCH_FIRMWARE failed", __FUNCTION__);
    //        return ret;
    //}        
    //usleep(300000); //firmware load delay
    return 0;
}
    
    
void cs35lxx_deinit(void)
{
    if (dev_node_fd >= 0)
        close (dev_node_fd);
    dev_node_fd = -1;
    free(calib_oeminfo_cs);
    calib_oeminfo_cs = NULL;
}
void cs35lxx_reg_dump(void)
{
    ALOGD("%s", __func__);

    FILE *fp;
    char *filename = (char *) malloc(50);
    char *line = NULL;
    size_t len = 0;
    ssize_t read;

    sprintf(filename, "/sys/kernel/debug/regmap/%s/registers", DEVICE_ADDRESS);
    fp = fopen(filename, "r");
    if (fp == NULL) {
        ALOGE("Failed to open registers");
        //delete(filename);
        return;
    }

    ALOGD("Start of registers dump:");
    while ((read = getline(&line, &len, fp)) != -1) {
        ALOGD("%s", line);
    }
    ALOGD("End of registers dump.");

    fclose(fp);
    fp = NULL;
    //if (line)
    //    delete(line);
    free(filename);
    filename = NULL;
}

void cs35lxx_set_calib_value(int re)
{
    ALOGD("%s: enter", __FUNCTION__);
	int ret = 0 ;
	struct cs35lxx_calib_data calib_data;
	  if (re == 0)
	      re = RE_DEFAULT;
	calib_data.temp = AMBIENT;
	calib_data.rdc = re;
	calib_data.status = 1;
	calib_data.checksum = calib_data.status + calib_data.rdc;
	ret = ioctl(dev_node_fd, SMARTPA_SPK_SET_CAL_STRUCT, &calib_data);
	if (ret < 0) {
	    ALOGE("%s: SMARTPA_SPK_SET_CAL_STRUCT failed (%d)", __FUNCTION__, ret);;
	    return;
	}
	
	ALOGD("%s: Set calibration value, re = %d", __FUNCTION__, re);
}
void cs35lxx_set_default_calib_value(int value)
{
    (void)value;
    ALOGD("%s: enter", __FUNCTION__);
    cs35lxx_set_calib_value(0);
    ALOGD("%s: Set default calibration value", __FUNCTION__);
}
int cs35lxx_speaker_on(unsigned int scene)
{
    int ret;
	int val = (int)is_dsp_bypass;
    ALOGI("%s: power on with box_scene = %u", __FUNCTION__, scene);
    ret = ioctl(dev_node_fd, SMARTPA_SPK_POWER_ON, &scene);
    if (ret < 0) {
            ALOGE("%s: SMARTPA_SPK_POWER_ON failed", __FUNCTION__);
            return ret;
    }    

    if (is_dsp_bypass) {
    	ALOGI("%s: SPK_POWER_ON succ with DSP bypass", __FUNCTION__);
		ret =  ioctl(dev_node_fd, SMARTPA_SPK_BYPASS_DSP, &val);
		if (ret < 0) {
		  ALOGE("%s: SMARTPA_SPK_BYPASS/ENABLE_DSP failed", __FUNCTION__);
		}
    } else {
    //appply calibrated r0 to DSP
    	if (calib_oeminfo_cs) {
		    ALOGI("%s:set calibrated R0 %.6f ohm, = int %d", __FUNCTION__, calib_oeminfo_cs->calibration_value_f[0],
						calib_oeminfo_cs->calibration_value[0]);
		    cs35lxx_set_calib_value(calib_oeminfo_cs->calibration_value[0]);
		    ALOGI("%s: SPK_POWER_ON succ with DSP, and apply calibrated values", __FUNCTION__);
		  } else {
		  	ALOGE("%s: SPK_POWER_ON succ with DSP, be careful oem read calib values", __FUNCTION__);
		  	cs35lxx_set_default_calib_value(0);
		  }
    }
    return ret;
}

int cs35lxx_speaker_off(unsigned int scene)
{
    int ret;
    int val = 0;
    (void)scene;
    ALOGD("enter function %s", __FUNCTION__);
    ret = ioctl(dev_node_fd, SMARTPA_SPK_POWER_OFF, &val);
    if (ret < 0) {
            ALOGE("%s: SMARTPA_SPK_POWER_OFF failed", __FUNCTION__);
            return ret;
    }
    ALOGD("%s: SPK_POWER_OFF succ", __FUNCTION__);
    return ret;
}
    
int cs35lxx_calibrate(void)
{
    ALOGD("%s", __func__);
    return 0;

}
    
    
int cs35lxx_get_re(unsigned int *re_array)
{
    unsigned int re;
    int ret = 0;
    ALOGD("%s: enter", __FUNCTION__);
    //(void) cs35lxx_calibration(g_firmware_name, CS_DEVICE);
    //re = cs35lxx_getRe(0); // 0 is SMartpa INDEX, Range 0-3
    ret = ioctl(dev_node_fd, SMARTPA_SPK_GET_R0, &re);
    if (ret < 0) {
        ALOGE("%s: SMARTPA_SPK_GET_R0 failed (%d)", __FUNCTION__, ret);
        return ret;
    }
    if (re == 0)
        re = RE_DEFAULT;
    re_array[0] = re;
    
    ALOGD("%s: Re = %d", __FUNCTION__, re);
    return 0;    
}

int cs35lxx_get_temp(int *temp_array)
{
    ALOGD("%s: enter", __FUNCTION__);
    int ret = 0 ;
    int t_realtime = 0;
    ret = ioctl(dev_node_fd, SMARTPA_SPK_GET_R0_REALTIME, &t_realtime);
    if (ret < 0) {
        ALOGE("%s: SMARTPA_SPK_GET_R0_REALTIME failed (%d)", __func__, ret);
        return ret;
    }

    ALOGD("\tT_realtime: 0x%x = %d(format) C degree\n", t_realtime, t_realtime/pow(2, 22));
    temp_array[0] = t_realtime/pow(2, 22);

    return ret;
}

int cs35lxx_get_r0(unsigned int *r0_array)
{
    ALOGD("%s: enter", __FUNCTION__);
    //(void) cs35lxx_getRo_req(g_firmware_name, CS_DEVICE);
    //re = cs35lxx_getR0(0); // 0 is SMartpa INDEX, Range 0-3
    //r0_array[0] = (unsigned int)(re * TEMP_MULTIPLE);
    int ret = 0 ;
    unsigned int t_realtime = 0;
	uint32_t r0_cal = 0;
    ret = ioctl(dev_node_fd, SMARTPA_SPK_GET_R0_REALTIME, &t_realtime);
    if (ret < 0) {
        ALOGE("%s: SMARTPA_SPK_GET_R0_REALTIME failed (%d)", __func__, ret);
        return ret;
    }

	ALOGD("%s: Get R0 array value:", __func__);
	ALOGD("\tT_realtime: 0x%x = %d(format)\n", t_realtime, t_realtime/pow(2, 22));
	if (calib_oeminfo_cs->calibration_value[0] > 0)
		r0_cal = calib_oeminfo_cs->calibration_value[0];
	else
		r0_cal = RE_DEFAULT;
	uint32_t r_realtime = (uint32_t)((t_realtime/pow(2, 22) - CS35LXX_R0_T_UNIFORM ) *
					  CS35LXX_R0_K_COEF * r0_cal + r0_cal);

	ALOGD("\tr_realtime: 0x%x\n", r_realtime);
	r0_array[0] = r_realtime;

    return ret;
}
/*
int cs35lxx_get_f0(unsigned int *f0_array)
{
    ALOGD("%s", __func__);
    int ret = 0 ;
    int f0 = 0;
    int ambient = AMBIENT;
    
    ret = ioctl(dev_node_fd, SMARTPA_SPK_SET_AMBIENT, &ambient);
    if (ret < 0) {
        ALOGE("%s: SMARTPA_SPK_SET_AMBIENT failed (%d)", __FUNCTION__, ret);
        return ret;
    }
	//start diagnostic
    ret = ioctl(dev_node_fd, SMARTPA_SPK_START_DIAGNOSTICS, &ret);
    
    if (ret < 0) {
        ALOGE("%s: SMARTPA_SPK_START_DIAGNOSTICS failed (%d)", __FUNCTION__, ret);
    }
	usleep(4000000);//need time to diagnostic f0
    ret = ioctl(dev_node_fd, SMARTPA_SPK_GET_F0, &f0);
    if (ret < 0) {
        ALOGE("%s: SMARTPA_SPK_GET_F0 failed (%d)", __func__, ret);
        return ret;
    }

    f0_array[0] = f0;
    ALOGD("%s: Get F0 value %d (%d)", __func__, f0_array[0], f0);
	//stop diagnostic
    ioctl(dev_node_fd, SMARTPA_SPK_STOP_DIAGNOSTICS, &ret);
    
    return ret;      
}
*/
int cs35lxx_get_f0(unsigned int *f0_array)
{
	f0_array[0] = F0_DEFAULT;
	return 0;
}
void cs35lxx_calib_start(void)
{
    ALOGI("%s: enter", __FUNCTION__);
    
    int ret = 0;
    int ambient = AMBIENT;
    
    ret = ioctl(dev_node_fd, SMARTPA_SPK_SET_AMBIENT, &ambient);
    if (ret < 0) {
        ALOGE("%s: SMARTPA_SPK_SET_AMBIENT failed (%d)", __FUNCTION__, ret);
        return;
    }
    ret = ioctl(dev_node_fd, SMARTPA_SPK_CALIBRATE, &ret);
    
    if (ret < 0) {
        ALOGE("%s: SMARTPA_SPK_CALIBRATE failed (%d)", __FUNCTION__, ret);
    }
    
    return;
}
void cs35lxx_calib_stop(void)
{
    ALOGD("%s: enter", __FUNCTION__);
    int ret = 0;
    ret = ioctl(dev_node_fd, SMARTPA_SPK_CALIBRATE_STOP, &ret);
    return;
}



int cs35lxx_get_default_calib_state(void)
{
    int state = 0;
    int ret;
    ALOGD("%s: enter", __FUNCTION__);
    ret = ioctl(dev_node_fd, SMARTPA_SPK_GET_CALIB_STATE, &state);
    if (ret < 0) {
            ALOGE("%s: SMARTPA_SPK_GET_CALIB_STATE failed, %d", __FUNCTION__, ret);
    }
    return state;
    
}
