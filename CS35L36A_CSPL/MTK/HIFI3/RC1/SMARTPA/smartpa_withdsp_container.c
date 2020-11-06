


#define LOG_TAG "cs35lxx"

#include <stdlib.h>
#include "securec.h"
#include "securectype.h"
#include "smartpa_algo_control.h"
#include "smartpa_with_dsp_interface.h"
#include "cs35lxx.h"



#define PARA_PATH_NUM 4
#define MAX_SMARTPA_NUM 4
#define MAX_DATA_BUF_LEN 256

extern struct name_to_index scene_map[SCENE_NUM + 1];
extern struct name_to_index algo_scene_map[SMARTPA_SCENE_NUM_USB];


static unsigned int g_algo_config;
static char *boxid_table[MAX_BOX_NUM] = {"NONE", "AAC", "GK", "GD", "LC", "LX"};

static void get_config_file(const char* path_suffix, char *path, size_t path_size)
{
    char *path_h[PARA_PATH_NUM] = {"/odm/etc", "/product/etc", "/vendor/etc", "/system/etc"};
    char buf[MAX_FILE_NAME_LEN] = {0};
    int ret;
    int i;
    
    for (i = 0; i < PARA_PATH_NUM; i++) { 
        ret  = snprintf_s(buf, MAX_FILE_NAME_LEN, MAX_FILE_NAME_LEN -1, "%s/%s", path_h[i], path_suffix);
        if (ret < 0) 
            ALOGE("%s: snprintf_s error", __FUNCTION__);
        if (access(buf, 0) == 0) {
            if (strncpy_s(path, path_size, buf, strlen(buf)) != 0)
                ALOGE("%s: strncpy_s error", __FUNCTION__);
            return;
        }
    }
    if (i >= PARA_PATH_NUM)
        *path = '\0';
    
}

static int set_smartpa_with_dsp_algo_scene(unsigned int algo_scene)
{
    ALOGD("%s: enter g_algo_config = %u", __FUNCTION__, algo_scene);
    g_algo_config = algo_scene;
    return 0;
    
}

static void smartpa_algo_bypass(bool enable)
{

    return cs35lxx_dsp_bypass(enable);
    
}

static int select_smartpa_scene(unsigned int reg_scene)
{
    int ret = 0;
    if ((g_algo_config >= SMARTPA_SCENE_NUM_USB) || (reg_scene > SCENE_NUM))
        return -1;
    ALOGD("%s: enter reg_scene = %s g_algo_config = %s", __FUNCTION__, scene_map[reg_scene].name,
                algo_scene_map[g_algo_config].name);
    switch (g_algo_config) {
        case SMARTPA_MUSIC:
        case SMARTPA_MUSIC_MONO:
                if ((reg_scene == POWER_ON_L) || (reg_scene == POWER_ON_L_BT_SCO))
                        cs35lxx_speaker_on(MUSIC);
                else if ((reg_scene == POWER_OFF_L) || (reg_scene == POWER_OFF_L_BT_SCO))
                        cs35lxx_speaker_off(MUSIC);
                else if (reg_scene == MMI_POWER_ON_PRI_L)
                        cs35lxx_speaker_on(MMI_PRI_L);
                else if (reg_scene == MMI_POWER_OFF)    
                        cs35lxx_speaker_off(MMI_PRI_L);
                else if (reg_scene == POWER_ON_FM_L)
                        cs35lxx_speaker_on(FM);
                else if (reg_scene == POWER_OFF_FM_L)    
                        cs35lxx_speaker_off(FM);
                else 
                        ret = -1;
                break;
        case SMARTPA_MMI_SPK:
                if (reg_scene == MMI_POWER_ON_PRI_L)
                        cs35lxx_speaker_on(MMI_PRI_L);
                else if (reg_scene == MMI_POWER_OFF)    
                        cs35lxx_speaker_off(MMI_PRI_L);
                else 
                        ret = -1;
                break;
        case SMARTPA_RINGTONE:
                if (reg_scene == POWER_ON_L)
                        cs35lxx_speaker_on(RINGTONE);
                else if (reg_scene == POWER_OFF_L)    
                        cs35lxx_speaker_off(RINGTONE);
                else 
                        ret = -1;
                break;
        case SMARTPA_RINGTONE_HS_SPK:
                if ((reg_scene == POWER_ON_L) || (reg_scene == POWER_ON_L_BT_SCO))
                        cs35lxx_speaker_on(RINGTONE_HS_SPK);
                else if ((reg_scene == POWER_OFF_L) || (reg_scene == POWER_OFF_L_BT_SCO))
                        cs35lxx_speaker_off(RINGTONE_HS_SPK);
                else 
                        ret = -1;
                break;
        case SMARTPA_INCALL_WB:
                if ((reg_scene == VOICE_POWER_ON_L) || (reg_scene == POWER_ON_L_BT_SCO))
                        cs35lxx_speaker_on(VOICE);
                else if ((reg_scene == VOICE_POWER_OFF_L) || (reg_scene == POWER_OFF_L_BT_SCO))
                        cs35lxx_speaker_off(VOICE);
                else 
                        ret = -1;
                break;
        case SMARTPA_VOIP:
                if ((reg_scene == VOICE_POWER_ON_L) || (reg_scene == POWER_ON_L_BT_SCO))
                        cs35lxx_speaker_on(VOIP);
                else if ((reg_scene == VOICE_POWER_OFF_L) || (reg_scene == POWER_OFF_L_BT_SCO))
                        cs35lxx_speaker_off(VOIP);
                else 
                        ret = -1;
                break;
        case SMARTPA_LOWPOWER:
                if ((reg_scene == POWER_ON_L) || (reg_scene == VOICE_POWER_ON_L))
                        cs35lxx_speaker_on(LOWPOWER);
                else if ((reg_scene == POWER_OFF_L) || (reg_scene == VOICE_POWER_OFF_L))
                        cs35lxx_speaker_off(LOWPOWER);
                else 
                        ret = -1;
                break;
        case SET_ALGO_BYPASS_FOR_PA_WITH_DSP:
        case SMARTPA_SCENE_NULL:
                ALOGE("%s: scene is %u", __func__, g_algo_config);
                smartpa_algo_bypass(true);
                if (reg_scene == POWER_ON_L)
                        cs35lxx_speaker_on(ALGO_BYPASS);
                else if (reg_scene == POWER_OFF_L)
                        cs35lxx_speaker_off(ALGO_BYPASS);
                else if (reg_scene == MMI_POWER_ON_PRI_L)
                        cs35lxx_speaker_on(ALGO_BYPASS);
                else if (reg_scene == MMI_POWER_OFF)
                        cs35lxx_speaker_off(ALGO_BYPASS);
                else
                        ret = -1;
                break;
        default:
                ALOGE("%s: scene %u not support", __func__, g_algo_config);
                ret  = -1;
                break;
    }
    
    if (ret == -1) {
            if ((reg_scene == POWER_ON_L) || (reg_scene == MMI_POWER_ON_PRI_L) ||
                    (reg_scene == VOICE_POWER_ON_L) || (reg_scene == POWER_ON_L_BT_SCO)) {
                        ALOGE("unsupport on scene, use default algo scene");
                        cs35lxx_speaker_on(MUSIC);
                        ret = 0;
            } else if ((reg_scene == POWER_OFF_L) || (reg_scene == MMI_POWER_OFF) ||
                    (reg_scene == VOICE_POWER_OFF_L) || (reg_scene == POWER_OFF_L_BT_SCO)) {
                        ALOGE("unsupport off scene, use default algo scene");
                        cs35lxx_speaker_off(MUSIC);
                        ret = 0;
            }
    }
    return ret;
}

static int smartpa_get_default_calib_value(const char  *box_name)
{
    char buf[MAX_DATA_BUF_LEN] = {0};
    char default_calib_suffix[MAX_FILE_NAME_LEN] = {0};
    char default_calib_file[MAX_FILE_NAME_LEN] = {0};
    
    FILE *fp = NULL;
    
    if(snprintf_s(default_calib_suffix, MAX_FILE_NAME_LEN, MAX_FILE_NAME_LEN-1,
        "audio/cirrus/default_calib_%s", box_name)< 0) {
            ALOGE("%s: snprintf_s error", __FUNCTION__);
            return -1;
    
    }
    
    get_config_file(default_calib_suffix, default_calib_file, sizeof(default_calib_file));
    if(strlen(default_calib_file) == 0) {
        ALOGE("%s: get calibration file %s fail", __FUNCTION__, default_calib_suffix);
        return -1;
    }
    fp = fopen(default_calib_file, "rb");
    if (fp == NULL) {
        ALOGE("%s: open calibration file %s fail", __FUNCTION__, default_calib_file);
        return -1;
    }
    
    if(fgets(buf, sizeof(buf)-1, fp)==NULL){
        ALOGE("%s: fgets value from default calibration file %s fail", __FUNCTION__, default_calib_file);
        fclose(fp);
        return -1;
    }
    
    int value = atoi(buf);//hardcode calibration value, for example 8 ohm 0x2000
    ALOGI("%s: default calibration value is %d", __FUNCTION__, value);
    
    fclose(fp);
    return value;
}

static void smartpa_set_default_calib_value(unsigned int box_vendor)
{
    if(box_vendor >= MAX_BOX_NUM) {
        ALOGE("%s: invalid box_vendor %u", __FUNCTION__, box_vendor);
        return;
    }
    
    if(cs35lxx_get_default_calib_state()!=0) {
        ALOGI("%s: default calibration value has been set", __FUNCTION__);
        return;
    }
    
    int value = smartpa_get_default_calib_value(boxid_table[box_vendor]);
    if(value < = 0) {
    
        ALOGE("%s: invalid default calibration value %d", __FUNCTION__, value);
        return;
    }
    cs35lxx_set_default_calib_value(value);
}



static int smartpa_init(char *cnt_file_name)
{
    int ret;
    unsigned int i;
    unsigned int box_vendor = MAX_BOX_NUM;
    
    if(cnt_file_name != NULL) {
        char delims[] = "_";
        char *tail_string = NULL;
        char *temp_string = strdup(cnt_file_name);
        
        ALOGD("%s: cnt_file_name = %s", __FUNCTION__, cnt_file_name);
        tail_string = strtok(temp_string, delims);
        tail_string = strtok(NULL, delims);
        free(temp_string);
        temp_string = NULL;
        
        for(i=0; i<MAX_BOX_NUM;i++) {
            if (strcmp(boxid_table[i], tail_string) == 0) {
                //for withdsp pa
                //cs35lxx_set_firmware_name(boxid_table[i]);
                cs35lxx_set_box_vendor(i);
                box_vendor = i;
                break;
            }
        }
        
        ALOGD("%s: boxid name = %s, boxid = %u", __FUNCTION__, tail_string, i);
        
    } else {
        
        ALOGD("%s: cnt_file_name = null ", __FUNCTION__);
    }
    ret = cs35lxx_init();
    if (ret == 0) {
        smartpa_set_default_calib_value(box_vendor);
        cs35lxx_speaker_off(MUSIC);
    }
    return ret;
}



static void smartpa_deinit(void)
{

    return cs35lxx_deinit();
}


static int smartpa_calibrate(void)
{
    int ret;
    ret = cs35lxx_calibrate();
    if (ret < 0) {
        ALOGE("%s: cs35lxx_calibrate failed %d", __FUNCTION__, ret);
    }
    return 0;
}
static int smartpa_get_r0(unsigned int *r0_array)
{
    if (r0_array == NULL)
            return -1;
    return cs35lxx_get_r0(r0_array);
}

static int smartpa_get_re(unsigned int *re_array)
{
    if (re_array == NULL)
            return -1;
    return cs35lxx_get_re(re_array);
}

static int smartpa_get_temperature(int *temp_array)
{
    if (temp_array == NULL)
            return -1;
    return cs35lxx_get_temp(temp_array);
}

static int smartpa_get_f0(unsigned int *f0_array)
{
    if (f0_array == NULL)
            return -1;
    return cs35lxx_get_f0(f0_array);
}

static void smartpa_reg_dump(void)
{
    cs35lxx_reg_dump();
    return;
}



static void smartpa_calib_start(void)
{
    return cs35lxx_calib_start();
}
static void smartpa_calib_stop(void)
{
    return cs35lxx_calib_stop();
}

static void smartpa_set_calib_value(void *param, unsigned int param_len)
{
    unsigned int re[4] = {0};//re has 4 values
    int ret;
    
    ret = memcpy_s(re, sizeof(re), param, param_len);
    if(ret != EOK)
        ALOGE("%s: memcpy_s error", __FUNCTION__);
    
    cs35lxx_set_calib_value(re[0]);
}

static int smartpa_parser_coef_range(char *product_name, double *coef_range)
{
    int i;
    (void)product_name;
    for(i = 0; i < 4; i++) {
        coef_range[i * 6 + 0] = MIN_CAL_Z;
        coef_range[i * 6 + 1] = MAX_CAL_Z;
        coef_range[i * 6 + 2] = 0;
        coef_range[i * 6 + 3] = 0;
        coef_range[i * 6 + 4] = MIN_F0;
        coef_range[i * 6 + 5] = MAX_F0;
    }    
    return 0;
}

int smartpa_lib_open(struct smartpa_with_dsp_lib *dsp_lib)
{
    if (dsp_lib == NULL) {
        ALOGE("%s: dsp_lib is null", __FUNCTION__);
        return -1;
    }

    dsp_lib->smartpa_init = smartpa_init;
    dsp_lib->smartpa_deinit = smartpa_deinit;
    dsp_lib->smartpa_calibrate = smartpa_calibrate;
    dsp_lib->smartpa_get_f0 = smartpa_get_f0;
    dsp_lib->smartpa_get_r0 =smartpa_get_r0;
    dsp_lib->smartpa_get_re = smartpa_get_re;
    dsp_lib->smartpa_reg_dump = smartpa_reg_dump;
    dsp_lib->smartpa_select_scene = select_smartpa_scene;
    dsp_lib->set_smartpa_with_dsp_algo_scene = set_smartpa_with_dsp_algo_scene;
    dsp_lib->parser_coef_range = smartpa_parser_coef_range;
    dsp_lib->smartpa_calib_start = smartpa_calib_start;
    dsp_lib->smartpa_calib_stop = smartpa_calib_stop;
    dsp_lib->smartpa_with_dsp_algp_bypass = smartpa_algo_bypass;
    dsp_lib->smartpa_set_calib_value = smartpa_set_calib_value;
    dsp_lib->smartpa_get_temperature = smartpa_get_temperature;

    ALOGD("%s: end succ", __FUNCTION__);
    return 0;
}
