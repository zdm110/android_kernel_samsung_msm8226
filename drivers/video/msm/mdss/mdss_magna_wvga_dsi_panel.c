/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/qpnp/pin.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/pwm.h>
#include <linux/err.h>
#if defined (CONFIG_LCD_CLASS_DEVICE)
	#include <linux/lcd.h>
#endif
#include "mdss_fb.h"
#include "mdnie_lite_tuning.h"
#include "mdss_dsi.h"
#include "mdss_magna_panel.h"
#include "dlog.h"

#if defined(CONFIG_LCD_CONNECTION_CHECK)
static int lcd_connected_status;
static char *panel_vendor = NULL;
#endif
#define DT_CMD_HDR 6
#if defined(CONFIG_ESD_ERR_FG_RECOVERY)
struct work_struct  err_fg_work;
static int err_fg_gpio;
static int esd_count;
static int err_fg_working;
#define ESD_DEBUG 1
#endif
DEFINE_LED_TRIGGER(bl_led_trigger);

static struct mdss_samsung_driver_data msd;
static struct mdss_panel_data mpd;

extern int poweroff_charging;
int recovery_boot_mode;
static int g_gamma_lux = 180;
static int boot_on;

#define LCD_ELVSS_DELTA_300CD (0x00)
#define LCD_ELVSS_DELTA_200CD (0x06)
#define LCD_ELVSS_DELTA_160CD (0x0A)
#define LCD_ELVSS_DELTA_100CD (0x0F)

#define LCD_ELVSS_RESULT_LIMIT (0x1F)

static int GET_ELVSS_ID[] = {
	LCD_ELVSS_DELTA_100CD,/* 0 = 30_dimming,*/
	LCD_ELVSS_DELTA_100CD,/* 1 = 40*/
	LCD_ELVSS_DELTA_100CD,/* 2 = 50*/
	LCD_ELVSS_DELTA_100CD,/* 3 = 60,*/
	LCD_ELVSS_DELTA_100CD,/* 4 = 70,*/
	LCD_ELVSS_DELTA_100CD,/* 5 = 80,*/
	LCD_ELVSS_DELTA_100CD,/* 6 = 90,*/
	LCD_ELVSS_DELTA_100CD,/* 7 = 100,*/
	LCD_ELVSS_DELTA_160CD,/* 8 = 110,*/
	LCD_ELVSS_DELTA_160CD,/* 9 = 120,*/
	LCD_ELVSS_DELTA_160CD,/* 10 = 130,*/
	LCD_ELVSS_DELTA_160CD,/* 11 = 140,*/
	LCD_ELVSS_DELTA_160CD,/* 12 = 150,*/
	LCD_ELVSS_DELTA_160CD,/* 13 = 160,*/
	LCD_ELVSS_DELTA_200CD,/* 14 = 170,*/
	LCD_ELVSS_DELTA_200CD,/* 15 = 180,*/
	LCD_ELVSS_DELTA_200CD,/* 16 = 190,*/
	LCD_ELVSS_DELTA_200CD,/* 17 = 200,*/
	LCD_ELVSS_DELTA_300CD,/* 18 = 210,*/
	LCD_ELVSS_DELTA_300CD,/* 19 = 220,*/
	LCD_ELVSS_DELTA_300CD,/* 20 = 230,*/
	LCD_ELVSS_DELTA_300CD,/* 21 = 240,*/
	LCD_ELVSS_DELTA_300CD,/* 22 = 250,*/
	LCD_ELVSS_DELTA_300CD/* 23 = 300,*/
};

#define LCD_ELVSS_DEFAULT_100CD (0x15)
#define LCD_ELVSS_DEFAULT_160CD (0x0F)
#define LCD_ELVSS_DEFAULT_200CD (0x0B)
#define LCD_ELVSS_DEFAULT_300CD (0x05)
static int GET_DEFAULT_ELVSS_ID[] = {
	LCD_ELVSS_DEFAULT_100CD,/* 0 = 30_dimming,*/
	LCD_ELVSS_DEFAULT_100CD,/* 1 = 40*/
	LCD_ELVSS_DEFAULT_100CD,/* 2 = 50*/
	LCD_ELVSS_DEFAULT_100CD,/* 3 = 60,*/
	LCD_ELVSS_DEFAULT_100CD,/* 4 = 70,*/
	LCD_ELVSS_DEFAULT_100CD,/* 5 = 80,*/
	LCD_ELVSS_DEFAULT_100CD,/* 6 = 90,*/
	LCD_ELVSS_DEFAULT_100CD,/* 7 = 100,*/
	LCD_ELVSS_DEFAULT_160CD,/* 8 = 110,*/
	LCD_ELVSS_DEFAULT_160CD,/* 9 = 120,*/
	LCD_ELVSS_DEFAULT_160CD,/* 10 = 130,*/
	LCD_ELVSS_DEFAULT_160CD,/* 11 = 140,*/
	LCD_ELVSS_DEFAULT_160CD,/* 12 = 150,*/
	LCD_ELVSS_DEFAULT_160CD,/* 13 = 160,*/
	LCD_ELVSS_DEFAULT_200CD,/* 14 = 170,*/
	LCD_ELVSS_DEFAULT_200CD,/* 15 = 180,*/
	LCD_ELVSS_DEFAULT_200CD,/* 16 = 190,*/
	LCD_ELVSS_DEFAULT_200CD,/* 17 = 200,*/
	LCD_ELVSS_DEFAULT_300CD,/* 18 = 210,*/
	LCD_ELVSS_DEFAULT_300CD,/* 19 = 220,*/
	LCD_ELVSS_DEFAULT_300CD,/* 20 = 230,*/
	LCD_ELVSS_DEFAULT_300CD,/* 21 = 240,*/
	LCD_ELVSS_DEFAULT_300CD,/* 22 = 250,*/
	LCD_ELVSS_DEFAULT_300CD/* 23 = 300,*/
};


static char acl_on[] = {
	0xC0,
	0x01
};

static char acl_off[] = {
	0xC0,
	0x00
};

static char elvss_cond_set[] = {
	0xB2,
	0x06, 0x06, 0x06, 0x06
};

static char elvss_on[] = {
	0xB1,
	0x0B, 0x16
};

static char gamma_flag_set[] = {
	0xFB,
	0x00, 0x5A
};
/* GAMMA SET FROM SMD */
static char gamma_set_cmd1[] = {
	0xF9,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00
};
static char gamma_flag_set_2[] = {
	0xFB,
	0x00, 0xa5,
};

static char ACL_COND_SET_40[] = {
	0xC1,
	0x4D, 0x96, 0x1D, 0x00, 0x00,
	0x01, 0xDF, 0x00, 0x00, 0x03,
	0x1F, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x01, 0x06, 0x11, 0x1A,
	0x20, 0x25, 0x29, 0x2D, 0x30,
	0x33, 0x35
};

static int lux_tbl_acl[] = {
	  30, 40, 50, 60, 70, 80,
	  90, 100, 110, 120, 130, 140,
	  150, 160, 170, 180, 190, 200,
	  210, 220, 230, 240, 250, 300
};

static char GAMMA_SmartDimming_COND_SET[] = {
	0xF9,
	0x00, 0xa6, 0xbf, 0xab, 0xd0,
	0xc1, 0xc1, 0x55, 0x00, 0xc1,
	0xbe, 0xa9, 0xce, 0xbe, 0xc7,
	0x55, 0x00, 0xc7, 0xbe, 0xa8,
	0xcf, 0xbf, 0xd3, 0x55
};

static char ACL_COND_SET_50[] = {
	0xC1,
	0x4D, 0x96, 0x1D, 0x00, 0x00,
	0x01, 0xDF, 0x00, 0x00, 0x03,
	0x1F, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x01, 0x08, 0x16, 0x22,
	0x2B, 0x31, 0x37, 0x3B, 0x3F,
	0x43, 0x46
};


static struct dsi_cmd_desc samsung_panel_gamma_update_cmds[3] = {
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
	sizeof(gamma_flag_set)}, gamma_flag_set},
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
	sizeof(gamma_set_cmd1)}, gamma_set_cmd1},
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
	sizeof(gamma_flag_set_2)}, gamma_flag_set_2},

};

static struct dsi_cmd_desc samsung_panel_elvss_update_cmds[] = {
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(elvss_on)}, elvss_on},
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
	 sizeof(elvss_cond_set)}, elvss_cond_set},
};

static struct dsi_cmd_desc samsung_panel_acl_on_cmds[] = {
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(acl_on)}, acl_on},
};

static struct dsi_cmd_desc samsung_panel_acl_off_cmds[] = {
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(acl_off)}, acl_off},
};
static struct dsi_cmd_desc samsung_panel_acl_update_cmds[] = {
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
	 sizeof(ACL_COND_SET_40)}, ACL_COND_SET_40},
};

static struct dsi_cmd_desc DSI_CMD_ACL_40 = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
	sizeof(ACL_COND_SET_40)}, ACL_COND_SET_40 };
static struct dsi_cmd_desc DSI_CMD_ACL_50 = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
	sizeof(ACL_COND_SET_50)}, ACL_COND_SET_50 };



static struct dsi_cmd_desc_LCD lcd_acl_table[] = {
	{0, "30", NULL},
	{0, "40", NULL},
	{40, "50", &DSI_CMD_ACL_40},
	{40, "60", &DSI_CMD_ACL_40},
	{40, "70", &DSI_CMD_ACL_40},
	{40, "80", &DSI_CMD_ACL_40},
	{40, "90", &DSI_CMD_ACL_40},
	{40, "100", &DSI_CMD_ACL_40},
	{40, "110", &DSI_CMD_ACL_40},
	{40, "120", &DSI_CMD_ACL_40},
	{40, "130", &DSI_CMD_ACL_40},
	{40, "140", &DSI_CMD_ACL_40},
	{40, "150", &DSI_CMD_ACL_40},
	{40, "160", &DSI_CMD_ACL_40},
	{40, "170", &DSI_CMD_ACL_40},
	{40, "180", &DSI_CMD_ACL_40},
	{40, "190", &DSI_CMD_ACL_40},
	{40, "200", &DSI_CMD_ACL_40},
	{40, "210", &DSI_CMD_ACL_40},
	{40, "220", &DSI_CMD_ACL_40},
	{40, "230", &DSI_CMD_ACL_40},
	{40, "240", &DSI_CMD_ACL_40},
	{40, "250", &DSI_CMD_ACL_40},
	{50, "300", &DSI_CMD_ACL_50},
};


static int get_candela_index(int bl_level)
{
	int backlightlevel;

	/* brightness setting from platform is from 0 to 255
	 * But in this driver, brightness is only supported from 0 to 24 */

	switch (bl_level) {
	case 0 ... 39:
		backlightlevel = GAMMA_30CD; /* 0*/
		break;
	case 40 ... 49:
		backlightlevel = GAMMA_40CD; /* 1 */
		break;
	case 50 ... 59:
		backlightlevel = GAMMA_50CD; /* 2 */
		break;
	case 60 ... 69:
		backlightlevel = GAMMA_60CD; /* 3 */
		break;
	case 70 ... 79:
		backlightlevel = GAMMA_70CD; /* 4 */
		break;
	case 80 ... 89:
		backlightlevel = GAMMA_80CD; /* 5 */
		break;
	case 90 ... 99:
		backlightlevel = GAMMA_90CD; /* 6 */
		break;
	case 100 ... 109:
		backlightlevel = GAMMA_100CD; /* 7 */
		break;
	case 110 ... 119:
		backlightlevel = GAMMA_110CD; /* 8 */
		break;
	case 120 ... 129:
		backlightlevel = GAMMA_120CD; /* 9 */
		break;
	case 130 ... 139:
		backlightlevel = GAMMA_130CD; /* 10 */
		break;
	case 140 ... 149:
		backlightlevel = GAMMA_140CD; /* 11 */
		break;
	case 150 ... 159:
		backlightlevel = GAMMA_150CD; /* 12 */
		break;
	case 160 ... 169:
		backlightlevel = GAMMA_160CD; /* 13 */
		break;
	case 170 ... 179:
		backlightlevel = GAMMA_170CD; /* 14 */
		break;
	case 180 ... 189:
		backlightlevel = GAMMA_180CD; /* 15 */
		break;
	case 190 ... 199:
		backlightlevel = GAMMA_190CD; /* 16 */
		break;
	case 200 ... 209:
		backlightlevel = GAMMA_200CD; /* 17 */
		break;
	case 210 ... 219:
		backlightlevel = GAMMA_210CD; /* 18 */
		break;
	case 220 ... 229:
		backlightlevel = GAMMA_220CD; /* 10 */
		break;
	case 230 ... 239:
		backlightlevel = GAMMA_230CD; /* 20 */
		break;
	case 240 ... 249:
		backlightlevel = GAMMA_240CD; /* 21 */
		break;
	case 250 ... 254:
		backlightlevel = GAMMA_250CD; /* 22 */
		break;
	case 255:
		if (msd.dstat.auto_brightness == 0)
			backlightlevel = GAMMA_250CD; /* 22 */
		else
			backlightlevel = GAMMA_300CD; /* 23 */
		break;
	default:
		backlightlevel = GAMMA_40CD; /* 1 */
		break;
	}
	return backlightlevel;
}


static int __init current_boot_mode(char *mode)
{
	/*
	*	1 is recovery booting
	*	0 is normal booting
	*/

	if (strncmp(mode, "1", 1) == 0)
		recovery_boot_mode = 1;
	else
		recovery_boot_mode = 0;

	pr_debug("%s %s", __func__, recovery_boot_mode == 1 ?
						"recovery" : "normal");
	return 1;
}
__setup("androidboot.boot_recovery=", current_boot_mode);


static int set_gamma_level(int bl_level, int gamma_mode)
{
	int cd;
	int *lux_tbl = lux_tbl_acl;

	cd = get_candela_index(bl_level);
	if (mpd.lcd_current_cd_idx == cd)
		return -EPERM;
	else
	    mpd.lcd_current_cd_idx = cd;

	if (gamma_mode == GAMMA_SMART) {

          //SMART Dimming gamma_lux;  
		char pBuffer[256];
		int i;
		int gamma_lux;

		gamma_lux = lux_tbl[cd];

		if (gamma_lux > SmartDimming_CANDELA_UPPER_LIMIT)
			gamma_lux = SmartDimming_CANDELA_UPPER_LIMIT;

		mpd.smart_ea8868.brightness_level = gamma_lux;

		if (gamma_lux != 30)
			g_gamma_lux = gamma_lux;

		pr_info("lcd :current lux(%d) g_gamma_lux(%d)\n",
					gamma_lux, g_gamma_lux);

		for (i = SmartDimming_GammaUpdate_Pos;
		     i < sizeof(GAMMA_SmartDimming_COND_SET); i++)
			GAMMA_SmartDimming_COND_SET[i] = 0;

		generate_gamma(&mpd.smart_ea8868,
						GAMMA_SmartDimming_COND_SET +
						SmartDimming_GammaUpdate_Pos,
								GEN_GAMMA_MAX);

		samsung_panel_gamma_update_cmds[1].dchdr.dlen =
		    sizeof(GAMMA_SmartDimming_COND_SET);
		samsung_panel_gamma_update_cmds[1].payload =
		    GAMMA_SmartDimming_COND_SET;
		pBuffer[0] = 0;
		for (i = 0; i < sizeof(GAMMA_SmartDimming_COND_SET); i++) {
			snprintf(pBuffer + strnlen(pBuffer, 256), 256, " %02x",
				 GAMMA_SmartDimming_COND_SET[i]);
		}
		pr_info("SD: %03d %s\n", gamma_lux, pBuffer);
		pr_info("bl_level:%d,cd:%d:Candela:%d\n", bl_level, cd, gamma_lux);
		}

	return 0;
}

static int set_acl_on_level(int bl_level)
{
	int cd;
	cd = get_candela_index(bl_level);
	if (!lcd_acl_table[cd].lux)
		return 1;

	if (lcd_acl_table[cd].lux) {
		samsung_panel_acl_update_cmds[0].dchdr.dlen =
		    lcd_acl_table[cd].cmd->dchdr.dlen;
		samsung_panel_acl_update_cmds[0].payload =
		    lcd_acl_table[cd].cmd->payload;
	}
	return 0;
}


void mdss_dsi_panel_pwm_cfg(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int ret;

	if (!gpio_is_valid(ctrl->pwm_pmic_gpio)) {
		pr_err("%s: pwm_pmic_gpio=%d Invalid\n", __func__,
				ctrl->pwm_pmic_gpio);
		ctrl->pwm_pmic_gpio = -1;
		return;
	}

	ret = gpio_request(ctrl->pwm_pmic_gpio, "disp_pwm");
	if (ret) {
		pr_err("%s: pwm_pmic_gpio=%d request failed\n", __func__,
				ctrl->pwm_pmic_gpio);
		ctrl->pwm_pmic_gpio = -1;
		return;
	}

	ctrl->pwm_bl = pwm_request(ctrl->pwm_lpg_chan, "lcd-bklt");
	if (ctrl->pwm_bl == NULL || IS_ERR(ctrl->pwm_bl)) {
		pr_err("%s: lpg_chan=%d pwm request failed", __func__,
				ctrl->pwm_lpg_chan);
		gpio_free(ctrl->pwm_pmic_gpio);
		ctrl->pwm_pmic_gpio = -1;
	}
}

static void mdss_dsi_panel_bklt_pwm(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	int ret;
	u32 duty;

	if (ctrl->pwm_bl == NULL) {
		pr_err("%s: no PWM\n", __func__);
		return;
	}

	duty = level * ctrl->pwm_period;
	duty /= ctrl->bklt_max;

	pr_debug("%s: bklt_ctrl=%d pwm_period=%d pwm_gpio=%d pwm_lpg_chan=%d\n",
			__func__, ctrl->bklt_ctrl, ctrl->pwm_period,
				ctrl->pwm_pmic_gpio, ctrl->pwm_lpg_chan);

	pr_debug("%s: ndx=%d level=%d duty=%d\n", __func__,
					ctrl->ndx, level, duty);

	ret = pwm_config(ctrl->pwm_bl, duty, ctrl->pwm_period);
	if (ret) {
		pr_err("%s: pwm_config() failed err=%d.\n", __func__, ret);
		return;
	}

	ret = pwm_enable(ctrl->pwm_bl);
	if (ret)
		pr_err("%s: pwm_enable() failed err=%d\n", __func__, ret);
}

void mdss_dsi_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_cmd_desc *cmds, int cnt,int flag)
{
	struct dcs_cmd_req cmdreq;
#if defined(CONFIG_LCD_CONNECTION_CHECK)
	if (is_lcd_attached() == 0)
	{
		printk("%s: LCD not connected!\n",__func__);
		return;
	}
#endif

	memset(&cmdreq, 0, sizeof(cmdreq));

	if (flag & CMD_REQ_SINGLE_TX) {
			cmdreq.flags = CMD_REQ_SINGLE_TX | CMD_CLK_CTRL | CMD_REQ_COMMIT;
		}else
			cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;

	cmdreq.cmds = cmds;
	cmdreq.cmds_cnt = cnt;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	
		
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	
}

static void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds)
{
	struct dcs_cmd_req cmdreq;
#if defined(CONFIG_LCD_CONNECTION_CHECK)
	if (is_lcd_attached() == 0)
	{
		printk("%s: LCD not connected!\n",__func__);
		return;
	}
#endif
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = pcmds->cmds;
	cmdreq.cmds_cnt = pcmds->cmd_cnt;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}


static int mipi_samsung_disp_send_cmd(enum mipi_samsung_cmd_list cmd, struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct dsi_cmd_desc *cmd_desc;
	int cmd_size = 0;
	int flag;

#ifdef CMD_DEBUG
	int i,j;
#endif
	
	switch (cmd) {
	case PANEL_READY_TO_ON:
		break;
	case PANEL_READY_TO_OFF:
		break;
	case PANEL_ON:
		break;
	case PANEL_OFF:
		break;
	case PANEL_LATE_ON:
		break;
	case PANEL_EARLY_OFF:
		break;
	case PANEL_GAMMA_UPDATE:
		flag = CMD_REQ_SINGLE_TX;
		cmd_desc = mpd.gamma_update.cmd;
		cmd_size = mpd.gamma_update.size;
		break;
	case MTP_READ_ENABLE:
		break;
#ifdef USE_ACL
	case PANEL_ACL_ON:
		flag = CMD_REQ_SINGLE_TX;
		cmd_desc = mpd.acl_on.cmd;
		cmd_size = mpd.acl_on.size;
		mpd.ldi_acl_stat = true;
		break;
	case PANEL_ACL_OFF:
		flag = CMD_REQ_SINGLE_TX;
		cmd_desc = mpd.acl_off.cmd;
		cmd_size = mpd.acl_off.size;
		mpd.ldi_acl_stat = false;
		break;
	case PANEL_ACL_UPDATE:
		flag = CMD_REQ_SINGLE_TX;
		cmd_desc = mpd.acl_update.cmd;
		cmd_size = mpd.acl_update.size;
		break;
#endif
#ifdef USE_ELVSS
	case PANEL_ELVSS_UPDATE:  
		flag = CMD_REQ_SINGLE_TX;
		mpd.set_elvss(msd.bl_level);
		cmd_desc = mpd.elvss_update.cmd;
		cmd_size = mpd.elvss_update.size;
		break;
#endif
	default:
		goto unknown_command;
		;
	}

	if (!cmd_size)
		goto unknown_command;

#ifdef CMD_DEBUG
		for (i = 0; i < cmd_size; i++)
		{
			for (j = 0; j < cmd_desc[i].dchdr.dlen; j++)
				pr_info("%x ",cmd_desc[i].payload[j]);
			pr_info("\n");
		}
#endif

	mdss_dsi_cmds_send(ctrl, cmd_desc, cmd_size, flag);


	return 0;

unknown_command:

	return 0;
}


int get_gamma_lux(void)
{
	return g_gamma_lux;
}

void reset_gamma_level(void)
{
	pr_info("reset_gamma_level\n");
	mpd.lcd_current_cd_idx = -1;
}

 int set_elvss_level(int bl_level)
{
	unsigned char calc_elvss;
	int cd;
	int id3=0;// = mipi_pd.manufacture_id & 0xFF;
	int id2=0;// = (mipi_pd.manufacture_id>>8) & 0xFF;

	cd = get_candela_index(bl_level);

	if (1)//id2 == 0x4A)
		calc_elvss = id3 + GET_ELVSS_ID[cd];
	else
		calc_elvss = GET_DEFAULT_ELVSS_ID[cd];

	pr_debug("%s: ID2=%x, ID3=%x, calc_elvss = %x\n", __func__, id2, id3,
		calc_elvss);

	if (calc_elvss > LCD_ELVSS_RESULT_LIMIT)
		calc_elvss = LCD_ELVSS_RESULT_LIMIT;

	elvss_cond_set[1] = calc_elvss;
	elvss_cond_set[2] = calc_elvss;
	elvss_cond_set[3] = calc_elvss;
	elvss_cond_set[4] = calc_elvss;

	return 0;
}


unsigned char mdss_dsi_panel_pwm_scaling(int level)
{
	unsigned char scaled_level;

	if (level >= MAX_BRIGHTNESS_LEVEL)
		scaled_level  = BL_MAX_BRIGHTNESS_LEVEL;
	else if (level >= MID_BRIGHTNESS_LEVEL) {
		scaled_level  = (level - MID_BRIGHTNESS_LEVEL) *
		(BL_MAX_BRIGHTNESS_LEVEL - BL_MID_BRIGHTNESS_LEVEL) / (MAX_BRIGHTNESS_LEVEL-MID_BRIGHTNESS_LEVEL) + BL_MID_BRIGHTNESS_LEVEL;
	} else if (level >= DIM_BRIGHTNESS_LEVEL) {
		scaled_level  = (level - DIM_BRIGHTNESS_LEVEL) *
		(BL_MID_BRIGHTNESS_LEVEL - BL_DIM_BRIGHTNESS_LEVEL) / (MID_BRIGHTNESS_LEVEL-DIM_BRIGHTNESS_LEVEL) + BL_DIM_BRIGHTNESS_LEVEL;
	} else if (level >= LOW_BRIGHTNESS_LEVEL) {
		scaled_level  = (level - LOW_BRIGHTNESS_LEVEL) *
		(BL_DIM_BRIGHTNESS_LEVEL - BL_LOW_BRIGHTNESS_LEVEL) / (DIM_BRIGHTNESS_LEVEL-LOW_BRIGHTNESS_LEVEL) + BL_LOW_BRIGHTNESS_LEVEL;
	}  else{
		if(poweroff_charging == 1)
			scaled_level  = level*BL_LOW_BRIGHTNESS_LEVEL/LOW_BRIGHTNESS_LEVEL;
		else
			scaled_level  = BL_MIN_BRIGHTNESS;
	}

	pr_info("level = [%d]: scaled_level = [%d]   autobrightness level:%d\n",level,scaled_level,msd.dstat.auto_brightness);

	return scaled_level;
}
/*
static char led_pwm1[2] = {0x51, 0xFF};
static struct dsi_cmd_desc backlight_cmd = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(led_pwm1)},
	led_pwm1
};

static void mdss_dsi_panel_bklt_dcs(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	struct dcs_cmd_req cmdreq;

	pr_debug("%s: level=%d\n", __func__, level);
#if defined(CONFIG_LCD_CONNECTION_CHECK)
	if (is_lcd_attached() == 0)
	{
		printk("%s: LCD not connected!\n",__func__);
		return;
	}
#endif

	led_pwm1[1] = mdss_dsi_panel_pwm_scaling(level);

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &backlight_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}*/

#if defined (CONFIG_LCD_CLASS_DEVICE) 
static char lcd_cabc[2] = {0x55, 0x0};	/* CABC COMMAND : default disabled */
static struct dsi_cmd_desc cabc_cmd= {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(lcd_cabc)},
	lcd_cabc
};

static void mdss_dsi_panel_cabc_dcs(struct mdss_dsi_ctrl_pdata *ctrl, int siop_status)
{
	
	struct dcs_cmd_req cmdreq;

	pr_debug("%s: cabc=%d\n", __func__, siop_status);
#if defined(CONFIG_LCD_CONNECTION_CHECK)
	if (is_lcd_attached() == 0)
	{
		printk("%s: LCD not connected!\n",__func__);
		return;
	}
#endif
	lcd_cabc[1] = (unsigned char)siop_status;
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &cabc_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}
#endif
int mdss_dsi_magna_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	int rc=0;
	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return 0;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return 0;
	}

	pr_info("%s: enable = %d\n", __func__, enable);
	if (enable) {
		rc = gpio_tlmm_config(GPIO_CFG(ctrl_pdata->disp_en_gpio_1_8v, 0,
					GPIO_CFG_OUTPUT,GPIO_CFG_NO_PULL,GPIO_CFG_8MA),
					GPIO_CFG_ENABLE);
		if (rc)
			pr_err("enabling disp_en_gpio_1_8v[%d] failed, rc=%d\n", ctrl_pdata->disp_en_gpio_1_8v, rc);
		
		rc = gpio_tlmm_config(GPIO_CFG(ctrl_pdata->rst_gpio, 0,
					GPIO_CFG_OUTPUT,GPIO_CFG_PULL_UP,GPIO_CFG_8MA),
					GPIO_CFG_ENABLE);
		if (rc)
			pr_err("disabling rst_gpio failed, rc=%d\n",rc);
		mdelay(20);
		gpio_set_value(ctrl_pdata->disp_en_gpio_1_8v, 1);
		mdelay(5);
		gpio_set_value((ctrl_pdata->rst_gpio), 1);
		msleep(20);
		gpio_set_value((ctrl_pdata->rst_gpio), 0);
		msleep(1);
		gpio_set_value((ctrl_pdata->rst_gpio), 1);
		msleep(20);
		if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
			gpio_set_value((ctrl_pdata->disp_en_gpio), 1);
		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}
	} else {
		
		if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
			gpio_set_value((ctrl_pdata->disp_en_gpio), 0);

		rc = gpio_tlmm_config(GPIO_CFG(ctrl_pdata->disp_en_gpio_1_8v, 0,
					GPIO_CFG_OUTPUT,GPIO_CFG_PULL_DOWN,GPIO_CFG_2MA),
					GPIO_CFG_ENABLE);
		if (rc)
			pr_err("disabling disp_en_gpio_1_8v failed, rc=%d\n",rc);

		rc = gpio_tlmm_config(GPIO_CFG(ctrl_pdata->rst_gpio, 0,
					GPIO_CFG_OUTPUT,GPIO_CFG_PULL_DOWN,GPIO_CFG_2MA),
					GPIO_CFG_ENABLE);
		if (rc)
			pr_err("disabling rst_gpio failed, rc=%d\n",rc);

		gpio_set_value(ctrl_pdata->disp_en_gpio_1_8v, 0);
		gpio_set_value((ctrl_pdata->rst_gpio), 0);
	}
	return 0;
}

static void mdss_dsi_panel_bl_ctrl(struct mdss_panel_data *pdata,
							u32 bl_level)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

#if defined(CONFIG_LCD_CONNECTION_CHECK)
	if (is_lcd_attached() == 0)
	{
		printk("%s: LCD not connected!\n",__func__);
		return;
	}
#endif

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	if( msd.mfd->panel_power_on == false){
		pr_err("%s: panel power off no bl ctrl\n", __func__);
		return;
	}
#if defined(CONFIG_ESD_ERR_FG_RECOVERY)
	if (err_fg_working) {
		pr_info("[LCD] %s : esd is working!! return.. \n", __func__);
		return;
	}
#endif
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	switch (ctrl_pdata->bklt_ctrl) {
	case BL_WLED:
		led_trigger_event(bl_led_trigger, bl_level);
		break;
	case BL_PWM:
		mdss_dsi_panel_bklt_pwm(ctrl_pdata, bl_level);
		break;
	case BL_DCS_CMD:
#if defined(CONFIG_MIPI_SAMSUNG_ESD_REFRESH)
		if (msd.esd_refresh == true)
			goto end;
#endif
		printk("mipi_samsung_disp_backlight %d\n", bl_level);
		if (!msd.mfd->panel_power_on || msd.mfd->resume_state == MIPI_SUSPEND_STATE)			
			goto end;

		if (set_gamma_level(bl_level, GAMMA_SMART) < 0)
			goto end;
		

		mipi_samsung_disp_send_cmd(PANEL_GAMMA_UPDATE,ctrl_pdata);

#ifdef USE_ELVSS
		if (mpd.set_elvss){
			msd.bl_level = bl_level;
			mipi_samsung_disp_send_cmd(PANEL_ELVSS_UPDATE,ctrl_pdata);
		}
#endif

#ifdef USE_ACL
		if (mpd.set_acl && msd.dstat.acl_on && mpd.set_acl(bl_level))
			mipi_samsung_disp_send_cmd(PANEL_ACL_OFF,ctrl_pdata);

		if (mpd.set_acl && msd.dstat.acl_on && !mpd.set_acl(bl_level)) {
			mipi_samsung_disp_send_cmd(PANEL_ACL_ON,ctrl_pdata);
			mipi_samsung_disp_send_cmd(PANEL_ACL_UPDATE,ctrl_pdata);
		}
#endif

		break;
	default:
		pr_err("%s: Unknown bl_ctrl configuration\n",
			__func__);
		break;
	}
end:
	return;
}

#if defined(CONFIG_ESD_ERR_FG_RECOVERY)
static irqreturn_t err_fg_irq_handler(int irq, void *handle)
{
	printk("%s : handler start\n", __func__);
	disable_irq_nosync(err_fg_gpio);
	schedule_work(&err_fg_work);
	printk("%s : handler end\n", __func__);

	return IRQ_HANDLED;
}
static void err_fg_work_func(struct work_struct *work)
{
	int bl_backup;
	struct mdss_panel_data *pdata = msd.mpd;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if(msd.mfd == NULL){
		pr_err("%s: mfd not initialized Skip ESD recovery\n", __func__);
		return;
	}
	bl_backup=msd.mfd->bl_level;
	if( msd.mfd->panel_power_on == false){
		pr_err("%s: Display off Skip ESD recovery\n", __func__);
		return;
	}
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
						panel_data);

	printk("%s : start", __func__);
	err_fg_working = 1;
	gpio_direction_output(ctrl_pdata->bl_on_gpio, 0);
	msd.mfd->fbi->fbops->fb_blank(FB_BLANK_POWERDOWN, msd.mfd->fbi);
	msd.mfd->fbi->fbops->fb_blank(FB_BLANK_UNBLANK, msd.mfd->fbi);
	esd_count++;
	err_fg_working = 0;
	msd.mfd->bl_level=bl_backup;
	mdss_dsi_panel_bl_ctrl(pdata,msd.mfd->bl_level);
	printk("%s : end", __func__);
	return;
}
#ifdef ESD_DEBUG
static ssize_t mipi_samsung_esd_check_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int rc;

	rc = snprintf((char *)buf, 20, "esd count:%d \n",esd_count);

	return rc;
}
static ssize_t mipi_samsung_esd_check_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct msm_fb_data_type *mfd;
	mfd = platform_get_drvdata(msd.msm_pdev);

	err_fg_irq_handler(0, mfd);
	return 1;
}

static DEVICE_ATTR(esd_check, S_IRUGO , mipi_samsung_esd_check_show,\
			 mipi_samsung_esd_check_store);
#endif
#endif


static int mdss_dsi_panel_on(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
#if defined(CONFIG_ESD_ERR_FG_RECOVERY)
	static int first_boot = 1;
#endif
	msd.mfd = (struct msm_fb_data_type *)registered_fb[0]->par;
	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}
	msd.mpd = pdata;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	
	pr_debug("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	if (!msd.dstat.is_smart_dim_loaded) {

		smart_dimming_init(&(mpd.smart_ea8868));
		msd.dstat.is_smart_dim_loaded = true;
		msd.dstat.gamma_mode = GAMMA_SMART;
	}

	if (mpd.gamma_initial && boot_on == 0) {
		mpd.smart_ea8868.brightness_level = 180;
		generate_gamma(&mpd.smart_ea8868,
			&(mpd.gamma_initial[1]), GAMMA_SET_MAX);

		if (recovery_boot_mode == 0)
			boot_on = 1;
	} else {
		mpd.smart_ea8868.brightness_level = get_gamma_lux();
		generate_gamma(&mpd.smart_ea8868,
			&(mpd.gamma_initial[1]), GAMMA_SET_MAX);
		reset_gamma_level();
	}


	if (ctrl->on_cmds.cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->on_cmds);

	msd.mfd->resume_state = MIPI_RESUME_STATE;
#if defined(CONFIG_MDNIE_LITE_TUNING)
	is_negative_on();
#endif
#if defined(CONFIG_ESD_ERR_FG_RECOVERY)
	if(first_boot) {
		int rc;
		
		INIT_WORK(&err_fg_work, err_fg_work_func);

		rc = request_threaded_irq(err_fg_gpio, NULL, err_fg_irq_handler, 
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, "esd_detect", NULL);
		if (rc) {
			pr_err("%s : Failed to request_irq. :ret=%d", __func__, rc);
		}
		first_boot = 0;
	}
	
	enable_irq(err_fg_gpio);
#endif
	pr_debug("%s:-\n", __func__);
	return 0;
}

static int mdss_dsi_panel_off(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	msd.mfd = (struct msm_fb_data_type *)registered_fb[0]->par;
	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
#if defined(CONFIG_ESD_ERR_FG_RECOVERY)
	if (!err_fg_working) {
		disable_irq_nosync(err_fg_gpio);
		cancel_work_sync(&err_fg_work);
	}
#endif
	pr_debug("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	if (ctrl->off_cmds.cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->off_cmds);

	mpd.ldi_acl_stat = false;
	msd.mfd->resume_state = MIPI_SUSPEND_STATE;
	pr_debug("%s:-\n", __func__);
	return 0;
}


static int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key)
{
	const char *data;
	int blen = 0, len;
	char *buf, *bp;
	struct dsi_ctrl_hdr *dchdr;
	int i, cnt;

	data = of_get_property(np, cmd_key, &blen);
	if (!data) {
		pr_err("%s: failed, key=%s\n", __func__, cmd_key);
		return -ENOMEM;
	}

	buf = kzalloc(sizeof(char) * blen, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy(buf, data, blen);

	/* scan dcs commands */
	bp = buf;
	len = blen;
	cnt = 0;
	while (len > sizeof(*dchdr)) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		dchdr->dlen = ntohs(dchdr->dlen);
		if (dchdr->dlen > len) {
			pr_err("%s: dtsi cmd=%x error, len=%d",
				__func__, dchdr->dtype, dchdr->dlen);
			return -ENOMEM;
		}
		bp += sizeof(*dchdr);
		len -= sizeof(*dchdr);
		bp += dchdr->dlen;
		len -= dchdr->dlen;
		cnt++;
	}

	if (len != 0) {
		pr_err("%s: dcs_cmd=%x len=%d error!",
				__func__, buf[0], blen);
		kfree(buf);
		return -ENOMEM;
	}

	pcmds->cmds = kzalloc(cnt * sizeof(struct dsi_cmd_desc),
						GFP_KERNEL);
	if (!pcmds->cmds){
		kfree(buf);
		return -ENOMEM;
	}

	pcmds->cmd_cnt = cnt;
	pcmds->buf = buf;
	pcmds->blen = blen;

	bp = buf;
	len = blen;
	for (i = 0; i < cnt; i++) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		len -= sizeof(*dchdr);
		bp += sizeof(*dchdr);
		pcmds->cmds[i].dchdr = *dchdr;
		pcmds->cmds[i].payload = bp;
		bp += dchdr->dlen;
		len -= dchdr->dlen;
	}


	data = of_get_property(np, link_key, NULL);
	if (!strncmp(data, "dsi_hs_mode", 11))
		pcmds->link_state = DSI_HS_MODE;
	else
		pcmds->link_state = DSI_LP_MODE;
	pr_debug("%s: dcs_cmd=%x len=%d, cmd_cnt=%d link_state=%d\n", __func__,
		pcmds->buf[0], pcmds->blen, pcmds->cmd_cnt, pcmds->link_state);

	return 0;
}
static int mdss_panel_dt_get_dst_fmt(u32 bpp, char mipi_mode, u32 pixel_packing,
				char *dst_format)
{
	int rc = 0;
	switch (bpp) {
	case 3:
		*dst_format = DSI_CMD_DST_FORMAT_RGB111;
		break;
	case 8:
		*dst_format = DSI_CMD_DST_FORMAT_RGB332;
		break;
	case 12:
		*dst_format = DSI_CMD_DST_FORMAT_RGB444;
		break;
	case 16:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB565;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB565;
			break;
		default:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB565;
			break;
		}
		break;
	case 18:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			if (pixel_packing == 0)
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666;
			else
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666_LOOSE;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB666;
			break;
		default:
			if (pixel_packing == 0)
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666;
			else
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666_LOOSE;
			break;
		}
		break;
	case 24:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB888;
			break;
		default:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
			break;
		}
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}


static int mdss_dsi_parse_fbc_params(struct device_node *np,
				struct mdss_panel_info *panel_info)
{
	int rc, fbc_enabled = 0;
	u32 tmp;

	fbc_enabled = of_property_read_bool(np,	"qcom,mdss-dsi-fbc-enable");
	if (fbc_enabled) {
		pr_debug("%s:%d FBC panel enabled.\n", __func__, __LINE__);
		panel_info->fbc.enabled = 1;
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-bpp", &tmp);
		panel_info->fbc.target_bpp =	(!rc ? tmp : panel_info->bpp);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-packing",
				&tmp);
		panel_info->fbc.comp_mode = (!rc ? tmp : 0);
		panel_info->fbc.qerr_enable = of_property_read_bool(np,
			"qcom,mdss-dsi-fbc-quant-error");
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-bias", &tmp);
		panel_info->fbc.cd_bias = (!rc ? tmp : 0);
		panel_info->fbc.pat_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-pat-mode");
		panel_info->fbc.vlc_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-vlc-mode");
		panel_info->fbc.bflc_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-bflc-mode");
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-h-line-budget",
				&tmp);
		panel_info->fbc.line_x_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-budget-ctrl",
				&tmp);
		panel_info->fbc.block_x_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-block-budget",
				&tmp);
		panel_info->fbc.block_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossless-threshold", &tmp);
		panel_info->fbc.lossless_mode_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossy-threshold", &tmp);
		panel_info->fbc.lossy_mode_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-rgb-threshold",
				&tmp);
		panel_info->fbc.lossy_rgb_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossy-mode-idx", &tmp);
		panel_info->fbc.lossy_mode_idx = (!rc ? tmp : 0);
	} else {
		pr_debug("%s:%d Panel does not support FBC.\n",
				__func__, __LINE__);
		panel_info->fbc.enabled = 0;
		panel_info->fbc.target_bpp =
			panel_info->bpp;
	}
	return 0;
}
static int mdss_panel_parse_dt(struct device_node *np,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	u32	tmp;
	int rc, i, len, res[2];
	const char *data;
	static const char *pdest;
	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-width", &tmp);
	if (rc) {
		pr_err("%s:%d, panel width not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}

	pinfo->xres = (!rc ? tmp : 640);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-height", &tmp);
	if (rc) {
		pr_err("%s:%d, panel height not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	pinfo->yres = (!rc ? tmp : 480);
	rc = of_property_read_u32_array(np, "qcom,mdss-pan-size", res, 2);
	if (rc == 0) {
		pinfo->physical_width= res[0];
		pinfo->physical_height= res[1];
	}

	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-left-border", &tmp);
	pinfo->lcdc.xres_pad = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-right-border", &tmp);
	if (!rc)
		pinfo->lcdc.xres_pad += tmp;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-top-border", &tmp);
	pinfo->lcdc.yres_pad = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-bottom-border", &tmp);
	if (!rc)
		pinfo->lcdc.yres_pad += tmp;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bpp", &tmp);
	if (rc) {
		pr_err("%s:%d, bpp not specified\n", __func__, __LINE__);
		return -EINVAL;
	}
	pinfo->bpp = (!rc ? tmp : 24);
	pinfo->mipi.mode = DSI_VIDEO_MODE;
	data = of_get_property(np, "qcom,mdss-dsi-panel-type", NULL);
	if (data && !strncmp(data, "dsi_cmd_mode", 12))
		pinfo->mipi.mode = DSI_CMD_MODE;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-pixel-packing", &tmp);
	tmp = (!rc ? tmp : 0);
	rc = mdss_panel_dt_get_dst_fmt(pinfo->bpp,
		pinfo->mipi.mode, tmp,
		&(pinfo->mipi.dst_format));
	if (rc) {
		pr_debug("%s: problem determining dst format. Set Default\n",
			__func__);
		pinfo->mipi.dst_format =
			DSI_VIDEO_DST_FORMAT_RGB888;
	}
	
	pdest = of_get_property(np,
			"qcom,mdss-dsi-panel-destination", NULL);
	if (strlen(pdest) != 9) {
		pr_err("%s: Unknown pdest specified\n", __func__);
		return -EINVAL;
	}
	if (!strncmp(pdest, "display_1", 9))
		pinfo->pdest = DISPLAY_1;
	else if (!strncmp(pdest, "display_2", 9))
		pinfo->pdest = DISPLAY_2;
	else {
		pr_debug("%s: pdest not specified. Set Default\n",
							__func__);
		pinfo->pdest = DISPLAY_1;
	}

	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-front-porch", &tmp);
	pinfo->lcdc.h_front_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-back-porch", &tmp);
	pinfo->lcdc.h_back_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-pulse-width", &tmp);
	pinfo->lcdc.h_pulse_width = (!rc ? tmp : 2);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-sync-skew", &tmp);
	pinfo->lcdc.hsync_skew = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-back-porch", &tmp);
	pinfo->lcdc.v_back_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-front-porch", &tmp);
	pinfo->lcdc.v_front_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-pulse-width", &tmp);
	pinfo->lcdc.v_pulse_width = (!rc ? tmp : 2);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-underflow-color", &tmp);
	pinfo->lcdc.underflow_clr = (!rc ? tmp : 0xff);

	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-border-color", &tmp);
	pinfo->lcdc.border_clr = (!rc ? tmp : 0);
	pinfo->bklt_ctrl = UNKNOWN_CTRL;
	data = of_get_property(np, "qcom,mdss-dsi-bl-pmic-control-type", NULL);
	if (data) {
		if (!strncmp(data, "bl_ctrl_wled", 12)) {
			led_trigger_register_simple("bkl-trigger",
				&bl_led_trigger);
			pr_debug("%s: SUCCESS-> WLED TRIGGER register\n",
				__func__);
			ctrl_pdata->bklt_ctrl = BL_WLED;
		} else if (!strncmp(data, "bl_ctrl_pwm", 11)) {
			ctrl_pdata->bklt_ctrl = BL_PWM;
			rc = of_property_read_u32(np,
				"qcom,mdss-dsi-bl-pmic-pwm-frequency", &tmp);
			if (rc) {
				pr_err("%s:%d, Error, panel pwm_period\n",		
					__func__, __LINE__);
			return -EINVAL;
			}
			ctrl_pdata->pwm_period = tmp;
			rc = of_property_read_u32(np,
					"qcom,mdss-dsi-bl-pmic-bank-select", &tmp);
			if (rc) {
				pr_err("%s:%d, Error, dsi lpg channel\n",
 						__func__, __LINE__);
				return -EINVAL;
			}
			ctrl_pdata->pwm_lpg_chan = tmp;
			tmp = of_get_named_gpio(np,
				"qcom,mdss-dsi-pwm-gpio", 0);
			ctrl_pdata->pwm_pmic_gpio = tmp;
		} else if (!strncmp(data, "bl_ctrl_dcs", 11)) {
			ctrl_pdata->bklt_ctrl = BL_DCS_CMD;
 		}
	}
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bl-min-level", &tmp);
	pinfo->bl_min = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bl-max-level", &tmp);
	pinfo->bl_max = (!rc ? tmp : 255);
	ctrl_pdata->bklt_max = pinfo->bl_max;

	rc = of_property_read_u32(np, "qcom,mdss-dsi-interleave-mode", &tmp);
	pinfo->mipi.interleave_mode = (!rc ? tmp : 0);
 
	pinfo->mipi.vsync_enable = of_property_read_bool(np,
		"qcom,mdss-dsi-te-check-enable");
	pinfo->mipi.hw_vsync_mode = of_property_read_bool(np,
		"qcom,mdss-dsi-te-using-te-pin");
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-h-sync-pulse", &tmp);
	pinfo->mipi.pulse_mode_hsa_he = (!rc ? tmp : false);

	pinfo->mipi.hfp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hfp-power-mode");
	pinfo->mipi.hsa_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hsa-power-mode");
	pinfo->mipi.hbp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hbp-power-mode");
	pinfo->mipi.bllp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-bllp-power-mode");
	pinfo->mipi.eof_bllp_power_stop = of_property_read_bool(
		np, "qcom,mdss-dsi-bllp-eof-power-mode");
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-traffic-mode", &tmp);
	pinfo->mipi.traffic_mode =
			(!rc ? tmp : DSI_NON_BURST_SYNCH_PULSE);

	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-dcs-command", &tmp);
	pinfo->mipi.insert_dcs_cmd =
			(!rc ? tmp : 1);

	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-v-sync-continue-lines", &tmp);
	pinfo->mipi.wr_mem_continue =
			(!rc ? tmp : 0x3c);

	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-v-sync-rd-ptr-irq-line", &tmp);
	pinfo->mipi.wr_mem_start =
			(!rc ? tmp : 0x2c);

	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-pin-select", &tmp);
	pinfo->mipi.te_sel =
			(!rc ? tmp : 1);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-virtual-channel-id", &tmp);
	pinfo->mipi.vc = (!rc ? tmp : 0);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-color-order", &tmp);
	pinfo->mipi.rgb_swap = (!rc ? tmp : DSI_RGB_SWAP_RGB);
	
	rc = of_property_read_u32(np, "qcom,mdss-force-clk-lane-hs", &tmp);
	pinfo->mipi.force_clk_lane_hs = (!rc ? tmp : 0);

	pinfo->mipi.data_lane0 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-0-state");
	pinfo->mipi.data_lane1 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-1-state");
	pinfo->mipi.data_lane2 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-2-state");
	pinfo->mipi.data_lane3 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-3-state");

	rc = of_property_read_u32(np, "qcom,mdss-dsi-lane-map", &tmp);
	pinfo->mipi.dlane_swap = (!rc ? tmp : 0);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-t-clk-pre", &tmp);
	pinfo->mipi.t_clk_pre = (!rc ? tmp : 0x24);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-t-clk-post", &tmp);
	pinfo->mipi.t_clk_post = (!rc ? tmp : 0x03);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-stream", &tmp);
	pinfo->mipi.stream = (!rc ? tmp : 0);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-mdp-trigger", &tmp);
	pinfo->mipi.mdp_trigger =
			(!rc ? tmp : DSI_CMD_TRIGGER_SW);
	if (pinfo->mipi.mdp_trigger > 6) {
		pr_err("%s:%d, Invalid mdp trigger. Forcing to sw trigger",
						 __func__, __LINE__);
		pinfo->mipi.mdp_trigger =
					DSI_CMD_TRIGGER_SW;
	}

	rc = of_property_read_u32(np, "qcom,mdss-dsi-dma-trigger", &tmp);
	pinfo->mipi.dma_trigger =
			(!rc ? tmp : DSI_CMD_TRIGGER_SW);
	if (pinfo->mipi.dma_trigger > 6) {
		pr_err("%s:%d, Invalid dma trigger. Forcing to sw trigger",
						 __func__, __LINE__);
		pinfo->mipi.dma_trigger =
					DSI_CMD_TRIGGER_SW;
	}

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-frame-rate", &tmp);
	pinfo->mipi.frame_rate = (!rc ? tmp : 60);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-clock-rate", &tmp);
	pinfo->clk_rate = (!rc ? tmp : 0);

	data = of_get_property(np, "qcom,mdss-dsi-panel-timings", &len);
	if ((!data) || (len != 12)) {
		pr_err("%s:%d, Unable to read Phy timing settings",
		       __func__, __LINE__);
		goto error;
	}
	for (i = 0; i < len; i++)
		pinfo->mipi.dsi_phy_db.timing[i] = data[i];

	mdss_dsi_parse_fbc_params(np, pinfo);
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->on_cmds,
		"qcom,mdss-dsi-on-command", "qcom,mdss-dsi-on-command-state");
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->off_cmds,
		"qcom,mdss-dsi-off-command", "qcom,mdss-dsi-off-command-state");
	return 0;
error:
	return -EINVAL;
}

#if defined(CONFIG_LCD_CLASS_DEVICE)

static ssize_t mdss_magna_disp_get_power(struct device *dev,
			struct device_attribute *attr, char *buf)
{

	pr_info("mipi_samsung_disp_get_power(0)\n");

	return 0;
}

static ssize_t mdss_magna_disp_set_power(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int power;
	if (sscanf(buf, "%u", &power) != 1)
		return -EINVAL;

	pr_info("mipi_samsung_disp_set_power:%d\n",power);

	return size;
}

static DEVICE_ATTR(lcd_power, S_IRUGO | S_IWUSR | S_IWGRP,
			mdss_magna_disp_get_power,
			mdss_magna_disp_set_power);

static ssize_t mdss_siop_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int rc;

	rc = snprintf(buf, 2, "%d\n",msd.dstat.siop_status);
	pr_info("%s :[MDSS_MAGNA] CABC: %d\n", __func__, msd.dstat.siop_status);
	return rc;
}
static ssize_t mdss_siop_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	
	if (sysfs_streq(buf, "1") && !msd.dstat.siop_status)
		msd.dstat.siop_status = true;
	else if (sysfs_streq(buf, "0") && msd.dstat.siop_status)
		msd.dstat.siop_status = false;
	else
		pr_info("%s: Invalid argument!!", __func__);
	
	return size;

}

static DEVICE_ATTR(siop_enable, S_IRUGO | S_IWUSR | S_IWGRP,
			mdss_siop_enable_show,
			mdss_siop_enable_store);


static ssize_t mipi_samsung_disp_acl_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int rc;

	rc = snprintf((char *)buf, sizeof(buf), "%d\n", msd.dstat.acl_on);
	printk("acl status: %d\n", *buf);

	return rc;
}



static ssize_t mipi_samsung_disp_acl_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct msm_fb_data_type *mfd;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
        struct mdss_panel_data *pdata = msd.mpd;
	
	mfd = msd.mfd;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
							panel_data);



	if (!msd.mfd->panel_power_on) {
		printk("%s: panel is off state. updating state value.\n",__func__);
		if (sysfs_streq(buf, "1") && !msd.dstat.acl_on)
			msd.dstat.acl_on = true;
		else if (sysfs_streq(buf, "0") && msd.dstat.acl_on)
			msd.dstat.acl_on = false;
		else
			pr_info("%s: Invalid argument!!", __func__);
	} else {

		if (sysfs_streq(buf, "1") && !msd.dstat.acl_on) {
			if (set_acl_on_level(msd.mfd->bl_level)){

//			mdss_dsi_panel_acl_ctrl(pdata,PANEL_ACL_OFF);
			mipi_samsung_disp_send_cmd(PANEL_ACL_OFF,ctrl_pdata);


			}else {
//			mdss_dsi_panel_acl_ctrl(pdata,PANEL_ACL_ON);
//			mdss_dsi_panel_acl_ctrl(pdata,PANEL_ACL_UPDATE);
			mipi_samsung_disp_send_cmd(PANEL_ACL_ON,ctrl_pdata);
			mipi_samsung_disp_send_cmd(PANEL_ACL_UPDATE,ctrl_pdata);



			}
			msd.dstat.acl_on = true;
		} else if (sysfs_streq(buf, "0") && msd.dstat.acl_on) {
//			mdss_dsi_panel_acl_ctrl(pdata,PANEL_ACL_OFF);
			mipi_samsung_disp_send_cmd(PANEL_ACL_OFF,ctrl_pdata);
			msd.dstat.acl_on = false;
		} else {
			pr_info("%s: Invalid argument!!", __func__);
		}
	}

	printk("mipi_samsung_disp_acl_store acl = %d\n",msd.dstat.acl_on);

	return size;
}


static DEVICE_ATTR(power_reduce, S_IRUGO | S_IWUSR | S_IWGRP,
			mipi_samsung_disp_acl_show,
			mipi_samsung_disp_acl_store);


static ssize_t mipi_samsung_disp_gamma_mode_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int rc;

	rc = snprintf((char *)buf, sizeof(buf), "%d\n", msd.dstat.gamma_mode);
	pr_info("gamma_mode: %d\n", *buf);

	return rc;
}

static ssize_t mipi_samsung_disp_gamma_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	if (sysfs_streq(buf, "1") && !msd.dstat.gamma_mode) {
		/* 1.9 gamma */
		msd.dstat.gamma_mode = GAMMA_1_9;
	} else if (sysfs_streq(buf, "0") && msd.dstat.gamma_mode) {
		/* 2.2 gamma */
		msd.dstat.gamma_mode = GAMMA_2_2;
	} else {
		pr_info("%s: Invalid argument!!", __func__);
	}

	return size;
}

static DEVICE_ATTR(gamma_mode, S_IRUGO | S_IWUSR | S_IWGRP,
			mipi_samsung_disp_gamma_mode_show,
			mipi_samsung_disp_gamma_mode_store);


static struct lcd_ops mdss_magna_disp_props = {

	.get_power = NULL,
	.set_power = NULL,

};

static ssize_t mdss_magna_auto_brightness_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int rc;

	rc = snprintf(buf, sizeof(buf), "%d\n",
					msd.dstat.auto_brightness);
	pr_info("%s : auto_brightness : %d\n", __func__, msd.dstat.auto_brightness);

	return rc;
}

static ssize_t mdss_magna_auto_brightness_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	static unsigned char prev_auto_brightness;
	struct mdss_panel_data *pdata = msd.mpd;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
						panel_data);
	if (sysfs_streq(buf, "0"))
		msd.dstat.auto_brightness = 0;
	else if (sysfs_streq(buf, "1"))
		msd.dstat.auto_brightness = 1;
	else if (sysfs_streq(buf, "2"))
		msd.dstat.auto_brightness = 2;
	else if (sysfs_streq(buf, "3"))
		msd.dstat.auto_brightness = 3;
	else if (sysfs_streq(buf, "4"))
		msd.dstat.auto_brightness = 4;
	else if (sysfs_streq(buf, "5"))
		msd.dstat.auto_brightness = 5;
	else if (sysfs_streq(buf, "6"))
		msd.dstat.auto_brightness = 6;
	else
		pr_info("%s: Invalid argument!!", __func__);

	if(prev_auto_brightness == msd.dstat.auto_brightness)
		return size;

	mdelay(1);

	if((msd.dstat.auto_brightness >=5 )|| (msd.dstat.auto_brightness == 0 ))
		msd.dstat.siop_status = false;
	else
		msd.dstat.siop_status = true;

	if( msd.mfd->panel_power_on == false){
		pr_err("%s: panel power off no bl ctrl\n", __func__);
		return size;
	}

	mdss_dsi_panel_cabc_dcs(ctrl_pdata, msd.dstat.siop_status);
	prev_auto_brightness = msd.dstat.auto_brightness;
	pr_info("%s %d %d\n", __func__, msd.dstat.auto_brightness, msd.dstat.siop_status);
	return size;
}

static DEVICE_ATTR(auto_brightness, S_IRUGO | S_IWUSR | S_IWGRP,
			mdss_magna_auto_brightness_show,
			mdss_magna_auto_brightness_store);

static ssize_t mdss_disp_lcdtype_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	char temp[20];

	snprintf(temp, 20, "SHA_LS052K3SY01");
	strncat(buf, temp, 20);
	return strnlen(buf, 20);
}
static DEVICE_ATTR(lcd_type, S_IRUGO, mdss_disp_lcdtype_show, NULL);
unsigned int mdss_dsi_show_cabc(void )
{
	return msd.dstat.siop_status;
}

void mdss_dsi_store_cabc(unsigned int cabc)
{
	struct mdss_panel_data *pdata = msd.mpd;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	if( msd.mfd->panel_power_on == false){
		pr_err("%s: panel power off no bl ctrl\n", __func__);
		return;
	}
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
						panel_data);
	if(msd.dstat.auto_brightness)
		return;

	msd.dstat.siop_status=cabc;
	mdss_dsi_panel_cabc_dcs(ctrl_pdata,msd.dstat.siop_status);
	pr_info("%s :[MDSS_MAGNA] CABC: %d\n", __func__,msd.dstat.siop_status);

}	
#endif


#if defined(CONFIG_LCD_CONNECTION_CHECK)
int is_lcd_attached(void)
{
	return lcd_connected_status;
	}
EXPORT_SYMBOL(is_lcd_attached);

static int __init lcd_attached_status(char *state)
{
	/*
	*	1 is lcd attached
	*	0 is lcd detached
	*/

	if (strncmp(state, "1", 1) == 0)
		lcd_connected_status = 1;
	else
		lcd_connected_status = 0;

	pr_info("%s %s", __func__, lcd_connected_status == 1 ?
				"lcd_attached" : "lcd_detached");
	return 1;
}
__setup("lcd_attached=", lcd_attached_status);

static int __init detect_lcd_panel_vendor(char* read_id)
{
	/*
	*	0x53A890 --> HIMAX
	*/
	int lcd_id = simple_strtol(read_id, NULL, 16);
	if(lcd_id == 0x53A890)
		panel_vendor = "HIMAX";
	else{
		pr_info("%s: manufacture id read may be faulty id[0x%x]\n", __func__, lcd_id);
		return 1;
	}
	pr_info("%s: detected panel vendor --> %s [0x%x]\n", __func__, panel_vendor, lcd_id);
	return 1;
}
__setup("lcd_id=0x", detect_lcd_panel_vendor);

#endif
int mdss_dsi_panel_init(struct device_node *node,
	struct mdss_dsi_ctrl_pdata *ctrl_pdata,
	bool cmd_cfg_cont_splash)
{
	int rc = 0;
	static const char *panel_name;
	bool cont_splash_enabled;
#if defined(CONFIG_ESD_ERR_FG_RECOVERY)
	int disp_esd_gpio;
#endif
#if defined(CONFIG_LCD_CLASS_DEVICE)
	struct lcd_device *lcd_device;

#if defined(CONFIG_BACKLIGHT_CLASS_DEVICE)
	struct backlight_device *bd = NULL;
#endif
#endif
#if defined(CONFIG_LCD_CLASS_DEVICE)
	struct device_node *np = NULL;
	struct platform_device *pdev = NULL;
	np = of_parse_phandle(node,
			"qcom,mdss-dsi-panel-controller", 0);
	if (!np) {
		pr_err("%s: Dsi controller node not initialized\n", __func__);
		return -EPROBE_DEFER;
	}

	pdev = of_find_device_by_node(np);
#endif
#if defined(CONFIG_FB_MSM_MDSS_KTD3102_BACKLIGHT)
	mutex_init(&bg_lock);
#endif

#if defined(CONFIG_LCD_CONNECTION_CHECK)
	printk("%s: LCD attached status: %d !\n",
				__func__, is_lcd_attached());
#endif
#ifdef DDI_VIDEO_ENHANCE_TUNING
	mutex_init(&msd.lock);
#endif
	if (!node) {
		pr_err("%s: no panel node\n", __func__);
		return -ENODEV;
	}

	pr_debug("%s:%d\n", __func__, __LINE__);
	panel_name = of_get_property(node, "qcom,mdss-dsi-panel-name", NULL);
	if (!panel_name)
		pr_info("%s:%d, Panel name not specified\n",
						__func__, __LINE__);
	else
		pr_info("%s: Panel Name = %s\n", __func__, panel_name);


	rc = mdss_panel_parse_dt(node, ctrl_pdata);
	if (rc) {
		pr_err("%s:%d panel dt parse failed\n", __func__, __LINE__);
		return rc;
	}

	if (cmd_cfg_cont_splash)
		cont_splash_enabled = of_property_read_bool(node,
				"qcom,cont-splash-enabled");
	else
		cont_splash_enabled = false;
	if (!cont_splash_enabled) {
		pr_info("%s:%d Continuous splash flag not found.\n",
				__func__, __LINE__);
		ctrl_pdata->panel_data.panel_info.cont_splash_enabled = 0;
	} else {
		pr_info("%s:%d Continuous splash flag enabled.\n",
				__func__, __LINE__);
		ctrl_pdata->panel_data.panel_info.cont_splash_enabled = 1;
	}

#if defined(CONFIG_LCD_CONNECTION_CHECK)
	if (is_lcd_attached() == 0)
	{
		printk("%s: LCD not connected.... Disabling Continous Splash!\n",__func__);
		ctrl_pdata->panel_data.panel_info.cont_splash_enabled = 0;
	}
#endif

	ctrl_pdata->on = mdss_dsi_panel_on;
	ctrl_pdata->off = mdss_dsi_panel_off;
	ctrl_pdata->panel_reset = mdss_dsi_magna_panel_reset;
	ctrl_pdata->panel_data.set_backlight = mdss_dsi_panel_bl_ctrl;

	msd.dstat.acl_on = false;
	mpd.gamma_update.cmd= samsung_panel_gamma_update_cmds;
	mpd.gamma_update.size =  ARRAY_SIZE(samsung_panel_gamma_update_cmds);
							 
	mpd.elvss_update.cmd = samsung_panel_elvss_update_cmds;
	mpd.elvss_update.size =	 ARRAY_SIZE(samsung_panel_elvss_update_cmds);
				 
	mpd.acl_on.cmd = samsung_panel_acl_on_cmds;
	mpd.acl_on.size =	ARRAY_SIZE(samsung_panel_acl_on_cmds);
					
	mpd.acl_off.cmd = samsung_panel_acl_off_cmds;
	mpd.acl_off.size =  ARRAY_SIZE(samsung_panel_acl_off_cmds);
					
	mpd.acl_update.cmd = samsung_panel_acl_update_cmds;
	mpd.acl_update.size = ARRAY_SIZE(samsung_panel_acl_update_cmds);
				
	mpd.gamma_initial = gamma_set_cmd1;
	mpd.lcd_current_cd_idx = -1;
	mpd.set_gamma = set_gamma_level;
	mpd.set_elvss = set_elvss_level;
	msd.dstat.gamma_mode = GAMMA_SMART;

#if defined(CONFIG_LCD_CLASS_DEVICE)
	lcd_device = lcd_device_register("panel", &pdev->dev, NULL,
					&mdss_magna_disp_props);

	if (IS_ERR(lcd_device)) {
		rc = PTR_ERR(lcd_device);
		printk(KERN_ERR "lcd : failed to register device\n");
		return rc;
	}

	sysfs_remove_file(&lcd_device->dev.kobj,&dev_attr_lcd_power.attr);

	rc = sysfs_create_file(&lcd_device->dev.kobj,&dev_attr_lcd_power.attr);

	if (rc) {
		pr_info("sysfs create fail-%s\n",dev_attr_lcd_power.attr.name);
	
	}
	rc = sysfs_create_file(&lcd_device->dev.kobj,
					&dev_attr_lcd_type.attr);
	if (rc) {
		pr_info("sysfs create fail-%s\n",
				dev_attr_lcd_type.attr.name);
	}

	
	rc= sysfs_create_file(&lcd_device->dev.kobj,
					&dev_attr_siop_enable.attr);
	if (rc) {
		pr_info("sysfs create fail-%s\n",
				dev_attr_siop_enable.attr.name);
	}

	rc = sysfs_create_file(&lcd_device->dev.kobj,
					&dev_attr_power_reduce.attr);
	if (rc) {
		pr_info("sysfs create fail-%s\n",
				dev_attr_power_reduce.attr.name);
	}

	rc = sysfs_create_file(&lcd_device->dev.kobj,
					&dev_attr_gamma_mode.attr);
	if (rc) {
		pr_info("sysfs create fail-%s\n",
				dev_attr_gamma_mode.attr.name);
	}


	
#if defined(CONFIG_BACKLIGHT_CLASS_DEVICE)
	bd = backlight_device_register("panel", &lcd_device->dev,
			NULL, NULL, NULL);
	if (IS_ERR(bd)) {
		rc = PTR_ERR(bd);
		pr_info("backlight : failed to register device\n");
		return rc;
	}
	rc= sysfs_create_file(&bd->dev.kobj,
					&dev_attr_auto_brightness.attr);
	if (rc) {
		pr_info("sysfs create fail-%s\n",
				dev_attr_auto_brightness.attr.name);
	}


	
#endif
#endif
#if defined(CONFIG_MDNIE_LITE_TUNING)
		pr_info("[%s] CONFIG_MDNIE_LITE_TUNING ok ! initclass called!\n",__func__);
		init_mdnie_class();
		mdnie_lite_tuning_init(&msd);
#endif
#if defined(CONFIG_ESD_ERR_FG_RECOVERY)
#ifdef ESD_DEBUG
	rc = sysfs_create_file(&lcd_device->dev.kobj,
							&dev_attr_esd_check.attr);
	if (rc) {
		pr_info("sysfs create fail-%s\n",
				dev_attr_esd_check.attr.name);
	}
#endif
	msd.msm_pdev = pdev;

	disp_esd_gpio =of_get_named_gpio(np,"qcom,oled-esd-gpio", 0);
	err_fg_gpio = gpio_to_irq(disp_esd_gpio);
	rc = gpio_request(disp_esd_gpio, "err_fg");
	if (rc) {
		pr_err("request gpio GPIO_ESD failed, ret=%d\n",rc);
		gpio_free(disp_esd_gpio);
		return rc;
	}
	gpio_tlmm_config(GPIO_CFG(disp_esd_gpio,  0, GPIO_CFG_INPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	rc = gpio_direction_input(disp_esd_gpio);
	if (unlikely(rc < 0)) {
		pr_err("%s: failed to set gpio %d as input (%d)\n",
			__func__, disp_esd_gpio, rc);
	}
#endif
	return 0;
}
