/*
 * linux/sound/cs35l36.h  --  CS35L36 Platform data
 *
 * Copyright 2020 Cirrus Logic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __CS35L36_H
#define __CS35L36_H

struct cs35l36_vpbr_cfg {
	bool is_present;
	bool vpbr_en;
	int vpbr_thld;
	int vpbr_atk_rate;
	int vpbr_atk_vol;
	int vpbr_max_attn;
	int vpbr_wait;
	int vpbr_rel_rate;
	int vpbr_mute_en;
};

struct cs35l36_platform_data {
	bool multi_amp_mode;
	bool dcm_mode;
	bool amp_pcm_inv;
	bool imon_pol_inv;
	bool vmon_pol_inv;
	int boost_ind;
	int bst_vctl;
	int bst_vctl_sel;
	int bst_ipk;
	bool extern_boost;
	int temp_warn_thld;
	int irq_drv_sel;
	int irq_gpio_sel;
	struct cs35l36_vpbr_cfg vpbr_config;
};

#endif /* __CS35L36_H */
