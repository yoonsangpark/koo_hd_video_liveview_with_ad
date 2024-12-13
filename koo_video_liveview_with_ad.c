/**
	@brief Sample code of video liveview with ccir sensor.\n

	@file video_liveview_with_ad.c

	@author kuro su

	@ingroup mhdal

	@note This file is modified from video_liveview.c.

	Copyright Novatek Microelectronics Corp. 2018.  All rights reserved.
*/

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include "hdal.h"
#include "hd_debug.h"
#include "vendor_videocapture.h"

// platform dependent
#if defined(__LINUX)
#include <pthread.h>			//for pthread API
#define MAIN(argc, argv) 		int main(int argc, char** argv)
#define GETCHAR()				getchar()
#else
#include <FreeRTOS_POSIX.h>
#include <FreeRTOS_POSIX/pthread.h> //for pthread API
#include <kwrap/util.h>		//for sleep API
#define sleep(x)    			vos_util_delay_ms(1000*(x))
#define msleep(x)    			vos_util_delay_ms(x)
#define usleep(x)   			vos_util_delay_us(x)
#include <kwrap/examsys.h> 	//for MAIN(), GETCHAR() API
#define MAIN(argc, argv) 		EXAMFUNC_ENTRY(hd_video_liveview_with_ad, argc, argv)
#define GETCHAR()				NVT_EXAMSYS_GETCHAR()
#endif

#define DEBUG_MENU 		1

#define CHKPNT			printf("\033[37mCHK: %s, %s: %d\033[0m\r\n",__FILE__,__func__,__LINE__)
#define DBGH(x)			printf("\033[0;35m%s=0x%08X\033[0m\r\n", #x, x)
#define DBGD(x)			printf("\033[0;35m%s=%d\033[0m\r\n", #x, x)

///////////////////////////////////////////////////////////////////////////////

//header
#define DBGINFO_BUFSIZE()	(0x200)

//RAW
#define VDO_RAW_BUFSIZE(w, h, pxlfmt)   (ALIGN_CEIL_4((w) * HD_VIDEO_PXLFMT_BPP(pxlfmt) / 8) * (h))
//NRX: RAW compress: Only support 12bit mode
#define RAW_COMPRESS_RATIO ((7/12)*100)
#define VDO_NRX_BUFSIZE(w, h)           (ALIGN_CEIL_4(ALIGN_CEIL_64(w) * 12 / 8 * RAW_COMPRESS_RATIO / 100 * (h)))
//CA for AWB
#define VDO_CA_BUF_SIZE(win_num_w, win_num_h) ALIGN_CEIL_4((win_num_w * win_num_h << 3) << 1)
//LA for AE
#define VDO_LA_BUF_SIZE(win_num_w, win_num_h) ALIGN_CEIL_4((win_num_w * win_num_h << 1) << 1)

//YUV
#define VDO_YUV_BUFSIZE(w, h, pxlfmt)	(ALIGN_CEIL_4((w) * HD_VIDEO_PXLFMT_BPP(pxlfmt) / 8) * (h))
//NVX: YUV compress
#define YUV_COMPRESS_RATIO 75
#define VDO_NVX_BUFSIZE(w, h, pxlfmt)	(VDO_YUV_BUFSIZE(w, h, pxlfmt) * YUV_COMPRESS_RATIO / 100)

///////////////////////////////////////////////////////////////////////////////

#define SEN_OUT_FMT		HD_VIDEO_PXLFMT_YUV422
#define CAP_OUT_FMT		HD_VIDEO_PXLFMT_YUV420
#define CA_WIN_NUM_W		32
#define CA_WIN_NUM_H		32
#define LA_WIN_NUM_W		32
#define LA_WIN_NUM_H		32
#define VA_WIN_NUM_W		16
#define VA_WIN_NUM_H		16
#define YOUT_WIN_NUM_W	128
#define YOUT_WIN_NUM_H	128
#define ETH_8BIT_SEL		0 //0: 2bit out, 1:8 bit out
#define ETH_OUT_SEL		1 //0: full, 1: subsample 1/2


typedef enum {
	AD_MODULE_TYPE_UNKNOWN = 0,
	AD_MODULE_TYPE_NVP6124B,
	AD_MODULE_TYPE_TP9950,			
	AD_MODULE_TYPE_TC358743,
	AD_MODULE_TYPE_TC358840,
	AD_MODULE_TYPE_TP2854,
	AD_MODULE_TYPE_TP2855,
	AD_MODULE_TYPE_NVP6188,
	AD_MODULE_TYPE_TP2860,	
	AD_MODULE_TYPE_TP2863,	
	AD_MODULE_TYPE_TP9963,	
	AD_MODULE_TYPE_MAX,
} AD_MODULE_TYPE;

typedef struct {
	VENDOR_VIDEOCAP_CCIR_FMT_SEL ccir_fmt;
	HD_COMMON_VIDEO_IN_TYPE bus_type;
} AD_MODULE_INFO;

typedef struct _VIDEO_LIVEVIEW {

	// (1)
	HD_VIDEOCAP_SYSCAPS cap_syscaps;
	HD_PATH_ID cap_ctrl;
	HD_PATH_ID cap_path;

	HD_DIM  cap_dim;
	HD_DIM  proc_max_dim;

	// (2)
	HD_VIDEOPROC_SYSCAPS proc_syscaps;
	HD_PATH_ID proc_ctrl;
	HD_PATH_ID proc_path;

	HD_DIM  out_max_dim;
	HD_DIM  out_dim;

	// (3)
	/*HD_VIDEOOUT_SYSCAPS out_syscaps;
	HD_PATH_ID out_ctrl;
	HD_PATH_ID out_path;

    HD_VIDEOOUT_HDMI_ID hdmi_id;*/

	// AD
	AD_MODULE_TYPE module_type;
	UINT32 chip_id;
	UINT32 vin_id;
	UINT32 vcap_id;
	UINT32 ccir_mux_index;
	UINT32 mipi_data_lane;
	BOOL det_en;
	UINT32 vdo_size_w;
	UINT32 vdo_size_h;
	UINT32 vdo_fps;
	BOOL vdo_interlace;

	AD_MODULE_INFO *module_info;
	UINT32 vproc_id;
	UINT32 vcap_id_bit;
	BOOL start;
	BOOL resume;
	VENDOR_VIDEOCAP_GET_PLUG_INFO plug_info;
	VENDOR_VIDEOCAP_CCIR_INFO ccir_info;
} VIDEO_LIVEVIEW;

#define AP_VERSION 			"0.01.007"
#define STREAM_MAX 			8

static AD_MODULE_INFO g_ad_module_info[AD_MODULE_TYPE_MAX] = {
	[AD_MODULE_TYPE_NVP6124B] = {
		.ccir_fmt = VENDOR_VIDEOCAP_CCIR_FMT_SEL_CCIR656,
		.bus_type = HD_COMMON_VIDEO_IN_P_AHD,
	},

	[AD_MODULE_TYPE_TP9950] = {
		.ccir_fmt = VENDOR_VIDEOCAP_CCIR_FMT_SEL_CCIR656,
		.bus_type = HD_COMMON_VIDEO_IN_P_AHD,
	},

	[AD_MODULE_TYPE_TC358743] = {
		.ccir_fmt = VENDOR_VIDEOCAP_CCIR_FMT_SEL_CCIR601,
		.bus_type = HD_COMMON_VIDEO_IN_MIPI_CSI,
	},

	[AD_MODULE_TYPE_TC358840] = {
		.ccir_fmt = VENDOR_VIDEOCAP_CCIR_FMT_SEL_CCIR601,
		.bus_type = HD_COMMON_VIDEO_IN_MIPI_CSI,
	},

	[AD_MODULE_TYPE_TP2854] = {
		.ccir_fmt = VENDOR_VIDEOCAP_CCIR_FMT_SEL_CCIR601,
		.bus_type = HD_COMMON_VIDEO_IN_MIPI_CSI,
	},

	[AD_MODULE_TYPE_TP2855] = {
		.ccir_fmt = VENDOR_VIDEOCAP_CCIR_FMT_SEL_CCIR601,
		.bus_type = HD_COMMON_VIDEO_IN_MIPI_CSI,
	},

	[AD_MODULE_TYPE_NVP6188] = {
		.ccir_fmt = VENDOR_VIDEOCAP_CCIR_FMT_SEL_CCIR601,
		.bus_type = HD_COMMON_VIDEO_IN_MIPI_CSI,
	},
	[AD_MODULE_TYPE_TP2860] = {
		.ccir_fmt = VENDOR_VIDEOCAP_CCIR_FMT_SEL_CCIR656,
		.bus_type = HD_COMMON_VIDEO_IN_P_AHD,
	},	

// 	typedef enum {
// 	VENDOR_VIDEOCAP_CCIR_FMT_SEL_CCIR601 = 0,
// 	VENDOR_VIDEOCAP_CCIR_FMT_SEL_CCIR656,
// 	VENDOR_VIDEOCAP_CCIR_FMT_SEL_CCIR709,
// 	VENDOR_VIDEOCAP_CCIR_FMT_SEL_CCIR1120,
// 	VENDOR_VIDEOCAP_CCIR_FMT_SEL_MAX_NUM,
// 	ENUM_DUMMY4WORD(VENDOR_VIDEOCAP_CCIR_FMT_SEL)
// } VENDOR_VIDEOCAP_CCIR_FMT_SEL;

	[AD_MODULE_TYPE_TP2863] = {
		.ccir_fmt = VENDOR_VIDEOCAP_CCIR_FMT_SEL_CCIR601,
		.bus_type = HD_COMMON_VIDEO_IN_MIPI_CSI,
	},	
	[AD_MODULE_TYPE_TP9963] = {
		.ccir_fmt = VENDOR_VIDEOCAP_CCIR_FMT_SEL_CCIR601,
		.bus_type = HD_COMMON_VIDEO_IN_MIPI_CSI,
	},	
};

static VIDEO_LIVEVIEW g_stream[STREAM_MAX] = {0};
static UINT32 g_vout_id = 0;
static UINT32 g_vout_open_cnt = 0, g_vout_start_cnt = 0;
static HD_VIDEOOUT_SYSCAPS g_out_syscaps = {0};
static HD_PATH_ID g_out_ctrl = 0;
static HD_PATH_ID g_out_path = 0;
static HD_VIDEOOUT_HDMI_ID g_hdmi_id = HD_VIDEOOUT_HDMI_1920X1080I60;
static pthread_t g_sen_det_plug_hdl;
static BOOL g_sen_det_plug_exit = 0;

// allocate fix max size
#define VDO_MEM_SIZE_W	1920
#define VDO_MEM_SIZE_H	1080

///////////////////////////////////////////////////////////////////////////////


static HD_RESULT mem_init(UINT32 stream_num)
{
	HD_RESULT              ret;
	HD_COMMON_MEM_INIT_CONFIG mem_cfg = {0};

	// config common pool (cap)
	mem_cfg.pool_info[0].type = HD_COMMON_MEM_COMMON_POOL;
	mem_cfg.pool_info[0].blk_size = DBGINFO_BUFSIZE()+VDO_YUV_BUFSIZE(VDO_MEM_SIZE_W, VDO_MEM_SIZE_H, CAP_OUT_FMT)
        													+VDO_CA_BUF_SIZE(CA_WIN_NUM_W, CA_WIN_NUM_H)
        													+VDO_LA_BUF_SIZE(LA_WIN_NUM_W, LA_WIN_NUM_H);
	mem_cfg.pool_info[0].blk_cnt = 2*stream_num;
	mem_cfg.pool_info[0].ddr_id = DDR_ID0;
	// config common pool (main)
	mem_cfg.pool_info[1].type = HD_COMMON_MEM_COMMON_POOL;
	mem_cfg.pool_info[1].blk_size = DBGINFO_BUFSIZE()+VDO_YUV_BUFSIZE(VDO_MEM_SIZE_W, VDO_MEM_SIZE_H, HD_VIDEO_PXLFMT_YUV420);
	mem_cfg.pool_info[1].blk_cnt = 3*stream_num;
	mem_cfg.pool_info[1].ddr_id = DDR_ID0;

	ret = hd_common_mem_init(&mem_cfg);
	return ret;
}

static HD_RESULT mem_exit(void)
{
	HD_RESULT ret = HD_OK;
	hd_common_mem_uninit();
	return ret;
}

///////////////////////////////////////////////////////////////////////////////

static HD_RESULT get_cap_caps(UINT32 stream_id)
{
	HD_RESULT ret = HD_OK;
	VIDEO_LIVEVIEW *p_stream = &g_stream[stream_id];
	HD_PATH_ID video_cap_ctrl = p_stream->cap_ctrl;
	HD_VIDEOCAP_SYSCAPS *p_video_cap_syscaps = &p_stream->cap_syscaps;

	hd_videocap_get(video_cap_ctrl, HD_VIDEOCAP_PARAM_SYSCAPS, p_video_cap_syscaps);
	return ret;
}

static HD_RESULT get_cap_sysinfo(UINT32 stream_id)
{
	HD_RESULT ret = HD_OK;
	VIDEO_LIVEVIEW *p_stream = &g_stream[stream_id];
	HD_PATH_ID video_cap_ctrl = p_stream->cap_ctrl;
	HD_VIDEOCAP_SYSINFO sys_info = {0};

	hd_videocap_get(video_cap_ctrl, HD_VIDEOCAP_PARAM_SYSINFO, &sys_info);
	printf("sys_info.devid =0x%X, cur_fps[0]=%d/%d, vd_count=%llu, output_started=%d, cur_dim(%dx%d)\r\n",
		sys_info.dev_id, GET_HI_UINT16(sys_info.cur_fps[0]), GET_LO_UINT16(sys_info.cur_fps[0]), sys_info.vd_count, sys_info.output_started, sys_info.cur_dim.w, sys_info.cur_dim.h);
	return ret;
}

static HD_RESULT set_cap_cfg(UINT32 stream_id)
{
	HD_RESULT ret = HD_OK;
	VIDEO_LIVEVIEW *p_stream = &g_stream[stream_id];
	HD_PATH_ID *p_video_cap_ctrl = &p_stream->cap_ctrl;
	HD_VIDEOCAP_DRV_CONFIG cap_cfg = {0};
	HD_PATH_ID video_cap_ctrl = 0;
	HD_VIDEOCAP_CTRL iq_ctl = {0};
	UINT32 ad_map;

	if (p_stream->module_type == AD_MODULE_TYPE_NVP6124B) {
		printf("cap cfg nvp6124b");
		snprintf(cap_cfg.sen_cfg.sen_dev.driver_name, HD_VIDEOCAP_SEN_NAME_LEN-1, "nvt_sen_ad_nvp6124b");
		cap_cfg.sen_cfg.sen_dev.if_type = p_stream->module_info->bus_type;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.sensor_pinmux =  0x08 | 0x800;  // PIN_SENSOR2_CFG_CCIR8BITS | PIN_SENSOR2_CFG_SN3_MCLK
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.cmd_if_pinmux = 0x40; // PIN_I2C_CFG_CH3

		cap_cfg.sen_cfg.sen_dev.pin_cfg.ccir_msblsb_switch = FALSE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.ccir_vd_hd_pin = TRUE;
	}
	else if (p_stream->module_type == AD_MODULE_TYPE_TP9950) {
		printf("cap cfg TP9950");
		snprintf(cap_cfg.sen_cfg.sen_dev.driver_name, HD_VIDEOCAP_SEN_NAME_LEN-1, "nvt_sen_ad_tp9950");
		cap_cfg.sen_cfg.sen_dev.if_type = p_stream->module_info->bus_type;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.sensor_pinmux =  0x08 | 0x800;  // PIN_SENSOR2_CFG_CCIR8BITS | PIN_SENSOR2_CFG_SN3_MCLK
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.cmd_if_pinmux = 0x40; // PIN_I2C_CFG_CH3

		cap_cfg.sen_cfg.sen_dev.pin_cfg.ccir_msblsb_switch = FALSE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.ccir_vd_hd_pin = TRUE;
	}
	else if (p_stream->module_type == AD_MODULE_TYPE_TP2860) {
		printf("cap cfg TP2860");
		snprintf(cap_cfg.sen_cfg.sen_dev.driver_name, HD_VIDEOCAP_SEN_NAME_LEN-1, "nvt_sen_ad_tp2860");
		cap_cfg.sen_cfg.sen_dev.if_type = p_stream->module_info->bus_type;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.sensor_pinmux =  0x08 | 0x800;  // PIN_SENSOR2_CFG_CCIR8BITS | PIN_SENSOR2_CFG_SN3_MCLK
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.cmd_if_pinmux = 0x40; // PIN_I2C_CFG_CH3

		cap_cfg.sen_cfg.sen_dev.pin_cfg.ccir_msblsb_switch = FALSE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.ccir_vd_hd_pin = TRUE;
	} else if (p_stream->module_type == AD_MODULE_TYPE_TC358743) {
		printf("cap cfg tc358743");
		snprintf(cap_cfg.sen_cfg.sen_dev.driver_name, HD_VIDEOCAP_SEN_NAME_LEN-1, "nvt_sen_ad_tc358743");
		cap_cfg.sen_cfg.sen_dev.if_type = p_stream->module_info->bus_type;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.sensor_pinmux =  0x20 | 0x200; //PIN_SENSOR_CFG_MIPI | PIN_SENSOR_CFG_MCLK
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.serial_if_pinmux = 0x1 | 0x100 | 0x200 | 0x400 | 0x800;//PIN_MIPI_LVDS_CFG_CLK0 | PIN_MIPI_LVDS_CFG_DAT0 | PIN_MIPI_LVDS_CFG_DAT1 | PIN_MIPI_LVDS_CFG_DAT2 | PIN_MIPI_LVDS_CFG_DAT3
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.cmd_if_pinmux = 0x10;//PIN_I2C_CFG_CH2
		cap_cfg.sen_cfg.sen_dev.pin_cfg.clk_lane_sel = HD_VIDEOCAP_SEN_CLANE_SEL_CSI0_USE_C0;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[0] = 0;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[1] = 1;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[2] = 2;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[3] = 3;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[4] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[5] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[6] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[7] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.ccir_vd_hd_pin = TRUE;
	} else if (p_stream->module_type == AD_MODULE_TYPE_TC358840) {
		printf("cap cfg tc358840");
		snprintf(cap_cfg.sen_cfg.sen_dev.driver_name, HD_VIDEOCAP_SEN_NAME_LEN-1, "nvt_sen_ad_tc358840");
		cap_cfg.sen_cfg.sen_dev.if_type = p_stream->module_info->bus_type;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.sensor_pinmux =  0x20 | 0x200; //PIN_SENSOR_CFG_MIPI | PIN_SENSOR_CFG_MCLK
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.serial_if_pinmux = 0x1 | 0x100 | 0x200 | 0x400 | 0x800;//PIN_MIPI_LVDS_CFG_CLK0 | PIN_MIPI_LVDS_CFG_DAT0 | PIN_MIPI_LVDS_CFG_DAT1 | PIN_MIPI_LVDS_CFG_DAT2 | PIN_MIPI_LVDS_CFG_DAT3
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.cmd_if_pinmux = 0x10;//PIN_I2C_CFG_CH2
		cap_cfg.sen_cfg.sen_dev.pin_cfg.clk_lane_sel = HD_VIDEOCAP_SEN_CLANE_SEL_CSI0_USE_C0;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[0] = 0;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[1] = 1;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[2] = 2;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[3] = 3;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[4] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[5] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[6] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[7] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.ccir_vd_hd_pin = TRUE;
	} else if (p_stream->module_type == AD_MODULE_TYPE_TP2854) {
		printf("cap cfg tp2854");
		snprintf(cap_cfg.sen_cfg.sen_dev.driver_name, HD_VIDEOCAP_SEN_NAME_LEN-1, "nvt_sen_ad_tp2854");
		cap_cfg.sen_cfg.sen_dev.if_type = p_stream->module_info->bus_type;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.sensor_pinmux =  0x20 | 0x200; //PIN_SENSOR_CFG_MIPI | PIN_SENSOR_CFG_MCLK
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.serial_if_pinmux = 0x1 | 0x100 | 0x200 | 0x400 | 0x800;//PIN_MIPI_LVDS_CFG_CLK0 | PIN_MIPI_LVDS_CFG_DAT0 | PIN_MIPI_LVDS_CFG_DAT1 | PIN_MIPI_LVDS_CFG_DAT2 | PIN_MIPI_LVDS_CFG_DAT3
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.cmd_if_pinmux = 0x10;//PIN_I2C_CFG_CH2
		cap_cfg.sen_cfg.sen_dev.pin_cfg.clk_lane_sel = HD_VIDEOCAP_SEN_CLANE_SEL_CSI0_USE_C0;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[0] = 0;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[1] = 1;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[2] = 2;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[3] = 3;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[4] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[5] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[6] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[7] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.ccir_vd_hd_pin = TRUE;
	} else if (p_stream->module_type == AD_MODULE_TYPE_TP2855) {
		printf("cap cfg tp2855");
		snprintf(cap_cfg.sen_cfg.sen_dev.driver_name, HD_VIDEOCAP_SEN_NAME_LEN-1, "nvt_sen_ad_tp2855");
		cap_cfg.sen_cfg.sen_dev.if_type = p_stream->module_info->bus_type;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.sensor_pinmux =  0x20 | 0x200; //PIN_SENSOR_CFG_MIPI | PIN_SENSOR_CFG_MCLK
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.serial_if_pinmux = 0x1 | 0x100 | 0x200 | 0x400 | 0x800;//PIN_MIPI_LVDS_CFG_CLK0 | PIN_MIPI_LVDS_CFG_DAT0 | PIN_MIPI_LVDS_CFG_DAT1 | PIN_MIPI_LVDS_CFG_DAT2 | PIN_MIPI_LVDS_CFG_DAT3
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.cmd_if_pinmux = 0x10;//PIN_I2C_CFG_CH2
		cap_cfg.sen_cfg.sen_dev.pin_cfg.clk_lane_sel = HD_VIDEOCAP_SEN_CLANE_SEL_CSI0_USE_C0;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[0] = 0;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[1] = 1;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[2] = 2;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[3] = 3;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[4] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[5] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[6] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[7] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.ccir_vd_hd_pin = TRUE;
	} else if (p_stream->module_type == AD_MODULE_TYPE_NVP6188) {
		printf("cap cfg nvp6188");
		snprintf(cap_cfg.sen_cfg.sen_dev.driver_name, HD_VIDEOCAP_SEN_NAME_LEN-1, "nvt_sen_ad_nvp6188");
		cap_cfg.sen_cfg.sen_dev.if_type = p_stream->module_info->bus_type;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.sensor_pinmux =  0x20 | 0x200; //PIN_SENSOR_CFG_MIPI | PIN_SENSOR_CFG_MCLK
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.serial_if_pinmux = 0x1 | 0x100 | 0x200 | 0x400 | 0x800;//PIN_MIPI_LVDS_CFG_CLK0 | PIN_MIPI_LVDS_CFG_DAT0 | PIN_MIPI_LVDS_CFG_DAT1 | PIN_MIPI_LVDS_CFG_DAT2 | PIN_MIPI_LVDS_CFG_DAT3
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.cmd_if_pinmux = 0x01;//PIN_I2C_CFG_CH1
		cap_cfg.sen_cfg.sen_dev.pin_cfg.clk_lane_sel = HD_VIDEOCAP_SEN_CLANE_SEL_CSI0_USE_C0;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[0] = 0;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[1] = 1;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[2] = 2;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[3] = 3;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[4] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[5] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[6] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[7] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.ccir_vd_hd_pin = TRUE;
	} else if (p_stream->module_type == AD_MODULE_TYPE_TP2863) {
		printf("cap cfg tp2863");
		snprintf(cap_cfg.sen_cfg.sen_dev.driver_name, HD_VIDEOCAP_SEN_NAME_LEN-1, "nvt_sen_ad_tp2863");
		cap_cfg.sen_cfg.sen_dev.if_type = p_stream->module_info->bus_type;
// PIN_MIPI_LVDS_CFG_CLK0 = 0x1,       ///< CLK lane 0. Enable HSI_CK0N/HSI_CK0P for LVDS/CSI/CSI2.
	// PIN_MIPI_LVDS_CFG_CLK1 = 0x2,       ///< CLK lane 1. Enable HSI_CK1N/HSI_CK1P for LVDS/CSI/CSI2.
	// PIN_MIPI_LVDS_CFG_DAT0 = 0x100,     ///< DATA lane 0. Enable HSI_D0N/HSI_D0P for LVDS/CSI/CSI2.
	// PIN_MIPI_LVDS_CFG_DAT1 = 0x200,     ///< DATA lane 1. Enable HSI_D1N/HSI_D1P for LVDS/CSI/CSI2.
	// PIN_MIPI_LVDS_CFG_DAT2 = 0x400,     ///< DATA lane 2. Enable HSI_D2N/HSI_D2P for LVDS/CSI/CSI2.
	// PIN_MIPI_LVDS_CFG_DAT3 = 0x800,     ///< DATA lane 3. Enable HSI_D3N/HSI_D3P for LVDS/CSI/CSI2.
#if 1 
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.sensor_pinmux =  0x20 | 0x200; //PIN_SENSOR_CFG_MIPI | PIN_SENSOR_CFG_MCLK
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.serial_if_pinmux = 0xF02;//PIN_MIPI_LVDS_CFG_CLK0 | PIN_MIPI_LVDS_CFG_DAT0 | PIN_MIPI_LVDS_CFG_DAT1 | PIN_MIPI_LVDS_CFG_DAT2 | PIN_MIPI_LVDS_CFG_DAT3		
		// cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.serial_if_pinmux = 0xF01;//PIN_MIPI_LVDS_CFG_CLK0 | PIN_MIPI_LVDS_CFG_DAT0 | PIN_MIPI_LVDS_CFG_DAT1 | PIN_MIPI_LVDS_CFG_DAT2 | PIN_MIPI_LVDS_CFG_DAT3		
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.cmd_if_pinmux = 0x01;//PIN_I2C_CFG_CH1
		cap_cfg.sen_cfg.sen_dev.pin_cfg.clk_lane_sel = HD_VIDEOCAP_SEN_CLANE_SEL_CSI0_USE_C0;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[0] = 0;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[1] = 1;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[2] = 2;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[3] = 3;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[4] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[5] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[6] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[7] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.ccir_vd_hd_pin = TRUE;		
#else
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.sensor_pinmux =  0x20 | 0x200; //PIN_SENSOR_CFG_MIPI | PIN_SENSOR_CFG_MCLK
	// cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.serial_if_pinmux =  0xC02;//PIN_MIPI_LVDS_CFG_CLK2 | PIN_MIPI_LVDS_CFG_DAT2 | PIN_MIPI_LVDS_CFG_DAT3	
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.serial_if_pinmux = 0x2 | 0x400 | 0x800;//PIN_MIPI_LVDS_CFG_CLK0 | PIN_MIPI_LVDS_CFG_DAT0 | PIN_MIPI_LVDS_CFG_DAT1 | PIN_MIPI_LVDS_CFG_DAT2 | PIN_MIPI_LVDS_CFG_DAT3
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.cmd_if_pinmux = 0x10;//PIN_I2C_CFG_CH2
		cap_cfg.sen_cfg.sen_dev.pin_cfg.clk_lane_sel = HD_VIDEOCAP_SEN_CLANE_SEL_CSI1_USE_C1;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[0] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[1] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[2] = 2;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[3] = 3;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[4] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[5] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[6] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[7] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.ccir_vd_hd_pin = TRUE;		
#endif 
	} else if (p_stream->module_type == AD_MODULE_TYPE_TP9963) {
		printf("cap cfg tp9963");
		snprintf(cap_cfg.sen_cfg.sen_dev.driver_name, HD_VIDEOCAP_SEN_NAME_LEN-1, "nvt_sen_ad_tp9963");
		cap_cfg.sen_cfg.sen_dev.if_type = p_stream->module_info->bus_type;
// PIN_MIPI_LVDS_CFG_CLK0 = 0x1,       ///< CLK lane 0. Enable HSI_CK0N/HSI_CK0P for LVDS/CSI/CSI2.
	// PIN_MIPI_LVDS_CFG_CLK1 = 0x2,       ///< CLK lane 1. Enable HSI_CK1N/HSI_CK1P for LVDS/CSI/CSI2.
	// PIN_MIPI_LVDS_CFG_DAT0 = 0x100,     ///< DATA lane 0. Enable HSI_D0N/HSI_D0P for LVDS/CSI/CSI2.
	// PIN_MIPI_LVDS_CFG_DAT1 = 0x200,     ///< DATA lane 1. Enable HSI_D1N/HSI_D1P for LVDS/CSI/CSI2.
	// PIN_MIPI_LVDS_CFG_DAT2 = 0x400,     ///< DATA lane 2. Enable HSI_D2N/HSI_D2P for LVDS/CSI/CSI2.
	// PIN_MIPI_LVDS_CFG_DAT3 = 0x800,     ///< DATA lane 3. Enable HSI_D3N/HSI_D3P for LVDS/CSI/CSI2.

		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.sensor_pinmux =  0x20 | 0x200; //PIN_SENSOR_CFG_MIPI | PIN_SENSOR_CFG_MCLK
	// cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.serial_if_pinmux =  0xC02;//PIN_MIPI_LVDS_CFG_CLK2 | PIN_MIPI_LVDS_CFG_DAT2 | PIN_MIPI_LVDS_CFG_DAT3	
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.serial_if_pinmux = 0x2 | 0x400 | 0x800;//PIN_MIPI_LVDS_CFG_CLK0 | PIN_MIPI_LVDS_CFG_DAT0 | PIN_MIPI_LVDS_CFG_DAT1 | PIN_MIPI_LVDS_CFG_DAT2 | PIN_MIPI_LVDS_CFG_DAT3
		cap_cfg.sen_cfg.sen_dev.pin_cfg.pinmux.cmd_if_pinmux = 0x10;//PIN_I2C_CFG_CH2
		cap_cfg.sen_cfg.sen_dev.pin_cfg.clk_lane_sel = HD_VIDEOCAP_SEN_CLANE_SEL_CSI1_USE_C1;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[0] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[1] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[2] = 2;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[3] = 3;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[4] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[5] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[6] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.sen_2_serial_pin_map[7] = HD_VIDEOCAP_SEN_IGNORE;
		cap_cfg.sen_cfg.sen_dev.pin_cfg.ccir_vd_hd_pin = TRUE;		
	}

	ret = hd_videocap_open(0, HD_VIDEOCAP_CTRL(p_stream->vcap_id), &video_cap_ctrl); //open this for device control
	
	if (ret != HD_OK) {
		return ret;
	}

	ad_map = VENDOR_VIDEOCAP_AD_MAP(p_stream->chip_id, p_stream->vin_id, p_stream->vcap_id_bit);
	vendor_videocap_set(video_cap_ctrl, VENDOR_VIDEOCAP_PARAM_AD_MAP, &ad_map);
	ret |= hd_videocap_set(video_cap_ctrl, HD_VIDEOCAP_PARAM_DRV_CONFIG, &cap_cfg);
	iq_ctl.func = 0;
	ret |= hd_videocap_set(video_cap_ctrl, HD_VIDEOCAP_PARAM_CTRL, &iq_ctl);

	*p_video_cap_ctrl = video_cap_ctrl;
	
	return ret;
}

static HD_RESULT set_cap_param(UINT32 stream_id)
{
	HD_RESULT ret = HD_OK;
	VIDEO_LIVEVIEW *p_stream = &g_stream[stream_id];
	HD_PATH_ID video_cap_path = p_stream->cap_path;
	HD_DIM *p_dim = &p_stream->cap_dim;

	{//select sensor mode, manually or automatically
		HD_VIDEOCAP_IN video_in_param = {0};

		video_in_param.sen_mode = HD_VIDEOCAP_SEN_MODE_AUTO; //auto select sensor mode by the parameter of HD_VIDEOCAP_PARAM_OUT
		video_in_param.frc = HD_VIDEO_FRC_RATIO(p_stream->vdo_fps, 1);
		video_in_param.dim.w = p_dim->w;
		video_in_param.dim.h = p_dim->h;
		video_in_param.pxlfmt = SEN_OUT_FMT;
		video_in_param.out_frame_num = HD_VIDEOCAP_SEN_FRAME_NUM_1;
		ret = hd_videocap_set(video_cap_path, HD_VIDEOCAP_PARAM_IN, &video_in_param);
		//printf("set_cap_param MODE=%d\r\n", ret);
		if (ret != HD_OK) {
			return ret;
		}
		ret = vendor_videocap_set(video_cap_path, VENDOR_VIDEOCAP_PARAM_CCIR_INFO, &p_stream->ccir_info);
		//printf("set_cap_param MODE=%d\r\n", ret);
		if (ret != HD_OK) {
			return ret;
		}
	}
	#if 1 //no crop, full frame
	{
		HD_VIDEOCAP_CROP video_crop_param = {0};

		video_crop_param.mode = HD_CROP_OFF;
		ret = hd_videocap_set(video_cap_path, HD_VIDEOCAP_PARAM_OUT_CROP, &video_crop_param);
		//printf("set_cap_param CROP NONE=%d\r\n", ret);
	}
	#else //HD_CROP_ON
	{
		HD_VIDEOCAP_CROP video_crop_param = {0};

		video_crop_param.mode = HD_CROP_ON;
		video_crop_param.win.rect.x = 0;
		video_crop_param.win.rect.y = 0;
		video_crop_param.win.rect.w = 1920/2;
		video_crop_param.win.rect.h= 1080/2;
		video_crop_param.align.w = 4;
		video_crop_param.align.h = 4;
		ret = hd_videocap_set(video_cap_path, HD_VIDEOCAP_PARAM_OUT_CROP, &video_crop_param);
		//printf("set_cap_param CROP ON=%d\r\n", ret);
	}
	#endif
	{
		HD_VIDEOCAP_OUT video_out_param = {0};

		//without setting dim for no scaling, using original sensor out size
		video_out_param.pxlfmt = CAP_OUT_FMT;
		video_out_param.dir = HD_VIDEO_DIR_NONE;
		ret = hd_videocap_set(video_cap_path, HD_VIDEOCAP_PARAM_OUT, &video_out_param);
		//printf("set_cap_param OUT=%d\r\n", ret);
	}
	{ // Set MIPI Data Lane
		ret = vendor_videocap_set(video_cap_path, VENDOR_VIDEOCAP_PARAM_DATA_LANE, &p_stream->mipi_data_lane);
	}

	return ret;
}

///////////////////////////////////////////////////////////////////////////////

static HD_RESULT set_proc_cfg(UINT32 stream_id)
{
	HD_RESULT ret = HD_OK;
	VIDEO_LIVEVIEW *p_stream = &g_stream[stream_id];
	HD_PATH_ID *p_video_proc_ctrl = &p_stream->proc_ctrl;
	HD_DIM* p_max_dim = &p_stream->proc_max_dim;
	HD_VIDEOPROC_DEV_CONFIG video_cfg_param = {0};
	HD_VIDEOPROC_CTRL video_ctrl_param = {0};
	HD_PATH_ID video_proc_ctrl = 0;

	ret = hd_videoproc_open(0, HD_VIDEOPROC_CTRL(p_stream->vproc_id), &video_proc_ctrl); //open this for device control
	if (ret != HD_OK)
		return ret;

	if (p_max_dim != NULL ) {
		video_cfg_param.pipe = HD_VIDEOPROC_PIPE_YUVALL;
		video_cfg_param.isp_id = p_stream->vproc_id;
		video_cfg_param.ctrl_max.func = 0;
		video_cfg_param.in_max.func = 0;
		video_cfg_param.in_max.dim.w = p_max_dim->w;
		video_cfg_param.in_max.dim.h = p_max_dim->h;
		video_cfg_param.in_max.pxlfmt = CAP_OUT_FMT;
		video_cfg_param.in_max.frc = HD_VIDEO_FRC_RATIO(1,1);
		ret = hd_videoproc_set(video_proc_ctrl, HD_VIDEOPROC_PARAM_DEV_CONFIG, &video_cfg_param);
		if (ret != HD_OK) {
			return HD_ERR_NG;
		}
	}

	video_ctrl_param.func = 0;
	ret = hd_videoproc_set(video_proc_ctrl, HD_VIDEOPROC_PARAM_CTRL, &video_ctrl_param);

	*p_video_proc_ctrl = video_proc_ctrl;

	return ret;
}

static HD_RESULT set_proc_param(UINT32 stream_id)
{
	HD_RESULT ret = HD_OK;
	VIDEO_LIVEVIEW *p_stream = &g_stream[stream_id];
	HD_PATH_ID video_proc_path = p_stream->proc_path;
	HD_DIM* p_dim = &p_stream->out_max_dim;

	//if (p_dim != NULL) { //if videoproc is already binding to dest module, not require to setting this!
		HD_VIDEOPROC_OUT video_out_param = {0};
		video_out_param.func = 0;
		video_out_param.dim.w = p_dim->w;
		video_out_param.dim.h = p_dim->h;
		video_out_param.pxlfmt = HD_VIDEO_PXLFMT_YUV420;
		video_out_param.dir = HD_VIDEO_DIR_NONE;
		video_out_param.frc = HD_VIDEO_FRC_RATIO(1,1);
		ret = hd_videoproc_set(video_proc_path, HD_VIDEOPROC_PARAM_OUT, &video_out_param);
	//}

	return ret;
}

///////////////////////////////////////////////////////////////////////////////

static HD_RESULT set_out_cfg(UINT32 stream_id)
{
	HD_RESULT ret = HD_OK;
	//VIDEO_LIVEVIEW *p_stream = &g_stream[stream_id];
	HD_PATH_ID *p_video_out_ctrl = &g_out_ctrl;
	UINT32 out_type = 1;
	HD_VIDEOOUT_HDMI_ID hdmi_id = g_hdmi_id;
	HD_VIDEOOUT_MODE videoout_mode = {0};
	HD_PATH_ID video_out_ctrl = 0;

	ret = hd_videoout_open(0, HD_VIDEOOUT_CTRL(0), &video_out_ctrl); //open this for device control
	if (ret != HD_OK) {
		return ret;
	}

	switch(out_type){
	case 0:
		videoout_mode.output_type = HD_COMMON_VIDEO_OUT_CVBS;
		videoout_mode.input_dim = HD_VIDEOOUT_IN_AUTO;
		videoout_mode.output_mode.cvbs= HD_VIDEOOUT_CVBS_NTSC;
	break;
	case 1:
		videoout_mode.output_type = HD_COMMON_VIDEO_OUT_LCD;
		videoout_mode.input_dim = HD_VIDEOOUT_IN_AUTO;
		videoout_mode.output_mode.lcd = HD_VIDEOOUT_LCD_0;
	break;
	case 2:
		videoout_mode.output_type = HD_COMMON_VIDEO_OUT_HDMI;
		videoout_mode.input_dim = HD_VIDEOOUT_IN_AUTO;
		videoout_mode.output_mode.hdmi= hdmi_id;
	break;
	default:
		printf("not support out_type\r\n");
	break;
	}
	ret = hd_videoout_set(video_out_ctrl, HD_VIDEOOUT_PARAM_MODE, &videoout_mode);

	*p_video_out_ctrl=video_out_ctrl ;
	return ret;
}

static HD_RESULT get_out_caps(UINT32 stream_id)
{
	HD_RESULT ret = HD_OK;
	//VIDEO_LIVEVIEW *p_stream = &g_stream[stream_id];
	HD_PATH_ID video_out_ctrl = g_out_ctrl;
	HD_VIDEOOUT_SYSCAPS *p_video_out_syscaps = &g_out_syscaps;
    HD_DEVCOUNT video_out_dev = {0};

	ret = hd_videoout_get(video_out_ctrl, HD_VIDEOOUT_PARAM_DEVCOUNT, &video_out_dev);
	if (ret != HD_OK) {
		return ret;
	}

	ret = hd_videoout_get(video_out_ctrl, HD_VIDEOOUT_PARAM_SYSCAPS, p_video_out_syscaps);
	if (ret != HD_OK) {
		return ret;
	}
	return ret;
}

static HD_RESULT set_out_param(UINT32 stream_id)
{
	HD_RESULT ret = HD_OK;
	VIDEO_LIVEVIEW *p_stream = &g_stream[stream_id];
	HD_PATH_ID video_out_path = g_out_path;
	HD_DIM *p_dim = &p_stream->out_dim;
	HD_VIDEOOUT_IN video_out_param={0};

	video_out_param.dim.w = p_dim->w;
	video_out_param.dim.h = p_dim->h;
	video_out_param.pxlfmt = HD_VIDEO_PXLFMT_YUV420;
	video_out_param.dir = HD_VIDEO_DIR_NONE;
	ret = hd_videoout_set(video_out_path, HD_VIDEOOUT_PARAM_IN, &video_out_param);
	if (ret != HD_OK) {
		return ret;
	}
	memset((void *)&video_out_param,0,sizeof(HD_VIDEOOUT_IN));
	ret = hd_videoout_get(video_out_path, HD_VIDEOOUT_PARAM_IN, &video_out_param);
	if (ret != HD_OK) {
		return ret;
	}
	printf("##video_out_param w:%d,h:%d %x %x\r\n", video_out_param.dim.w, video_out_param.dim.h, video_out_param.pxlfmt, video_out_param.dir);

	return ret;
}

static HD_RESULT init_module(void)
{
	HD_RESULT ret;
	if ((ret = hd_videocap_init()) != HD_OK)
		return ret;
	if ((ret = hd_videoproc_init()) != HD_OK)
		return ret;
	if ((ret = hd_videoout_init()) != HD_OK)
		return ret;
	return HD_OK;
}

static HD_RESULT open_module(UINT32 stream_id)
{
	HD_RESULT ret;
	VIDEO_LIVEVIEW *p_stream = &g_stream[stream_id];

	// set videocap config
	ret = set_cap_cfg(stream_id);
	if (ret != HD_OK) {
		printf("set cap-cfg fail=%d\n", ret);
		return HD_ERR_NG;
	}

	// set videoproc config
	ret = set_proc_cfg(stream_id);
	if (ret != HD_OK) {
		printf("set proc-cfg fail=%d\n", ret);
		return HD_ERR_NG;
	}

	// set videoout config
	ret = set_out_cfg(stream_id);
	if (ret != HD_OK) {
		printf("set out-cfg fail=%d\n", ret);
		return HD_ERR_NG;
	}

	if ((ret = hd_videocap_open(HD_VIDEOCAP_IN(p_stream->vcap_id, 0), HD_VIDEOCAP_OUT(p_stream->vcap_id, 0), &p_stream->cap_path)) != HD_OK)
		return ret;
	if ((ret = hd_videoproc_open(HD_VIDEOPROC_IN(p_stream->vproc_id, 0), HD_VIDEOPROC_OUT(p_stream->vproc_id, 0), &p_stream->proc_path)) != HD_OK)
		return ret;
	if (!g_vout_open_cnt && (ret = hd_videoout_open(HD_VIDEOOUT_0_IN_0, HD_VIDEOOUT_0_OUT_0, &g_out_path)) != HD_OK)
		return ret;

	g_vout_open_cnt++;

	return HD_OK;
}

static HD_RESULT close_module(UINT32 stream_id)
{
	HD_RESULT ret;
	VIDEO_LIVEVIEW *p_stream = &g_stream[stream_id];

	if (g_vout_open_cnt) {
		g_vout_open_cnt--;
	}

	if ((ret = hd_videocap_close(p_stream->cap_path)) != HD_OK)
		return ret;
	if ((ret = hd_videoproc_close(p_stream->proc_path)) != HD_OK)
		return ret;
	if (!g_vout_open_cnt && (ret = hd_videoout_close(g_out_path)) != HD_OK)
		return ret;

	return HD_OK;
}

static HD_RESULT exit_module(void)
{
	HD_RESULT ret;
	if ((ret = hd_videocap_uninit()) != HD_OK)
		return ret;
	if ((ret = hd_videoproc_uninit()) != HD_OK)
		return ret;
	if ((ret = hd_videoout_uninit()) != HD_OK)
		return ret;
	return HD_OK;
}

static HD_RESULT stream_start(UINT32 stream_id)
{
	HD_RESULT ret = HD_OK;
	VIDEO_LIVEVIEW *p_stream = &g_stream[stream_id];

	if (p_stream->start) {
		return HD_OK;
	}

	printf("stream%u, Start!\n", stream_id);

	// open video_liveview modules
	p_stream->proc_max_dim.w = VDO_MEM_SIZE_W; //assign by user
	p_stream->proc_max_dim.h = VDO_MEM_SIZE_H; //assign by user
	ret = open_module(stream_id);
	if (ret != HD_OK) {
		printf("open fail=%d\n", ret);
		goto exit;
	}

	// get videocap capability
	ret = get_cap_caps(stream_id);
	if (ret != HD_OK) {
		printf("get cap-caps fail=%d\n", ret);
		goto exit;
	}

	// get videoout capability
	ret = get_out_caps(stream_id);
	if (ret != HD_OK) {
		printf("get out-caps fail=%d\n", ret);
		goto exit;
	}
	p_stream->out_max_dim = g_out_syscaps.output_dim;

	// set videocap parameter
	p_stream->cap_dim.w = p_stream->vdo_size_w; //assign by user
	p_stream->cap_dim.h = p_stream->vdo_size_h; //assign by user
	ret = set_cap_param(stream_id);
	if (ret != HD_OK) {
		printf("set cap fail=%d\n", ret);
		goto exit;
	}

	// set videoproc parameter
	ret = set_proc_param(stream_id);
	if (ret != HD_OK) {
		printf("set proc fail=%d\n", ret);
		goto exit;
	}

	// set videoout parameter
	p_stream->out_dim.w = p_stream->out_max_dim.w; //using device max dim.w
	p_stream->out_dim.h = p_stream->out_max_dim.h; //using device max dim.h
	ret = set_out_param(stream_id);
	if (ret != HD_OK) {
		printf("set out fail=%d\n", ret);
		goto exit;
	}

	p_stream->start = TRUE;

exit:
	return ret;
}

static HD_RESULT stream_resume(UINT32 stream_id)
{
	HD_RESULT ret = HD_OK;
	VIDEO_LIVEVIEW *p_stream = &g_stream[stream_id];

	if (p_stream->resume) {
		return HD_OK;
	}

	printf("stream%u, Resume!\n", stream_id);

	// bind video_liveview modules
	hd_videocap_bind(HD_VIDEOCAP_OUT(p_stream->vcap_id, 0), HD_VIDEOPROC_IN(p_stream->vproc_id, 0));
	hd_videoproc_bind(HD_VIDEOPROC_OUT(g_vout_id, 0), HD_VIDEOOUT_0_IN_0);

	// start video_liveview modules
	hd_videocap_start(p_stream->cap_path);
	hd_videoproc_start(p_stream->proc_path);
	if (!g_vout_start_cnt) {
		hd_videoout_start(g_out_path);
	}

	g_vout_start_cnt++;

	p_stream->resume = TRUE;

	return ret;
}

static HD_RESULT stream_pause(UINT32 stream_id)
{
	HD_RESULT ret = HD_OK;
	VIDEO_LIVEVIEW *p_stream = &g_stream[stream_id];

	if (!p_stream->resume) {
		return HD_OK;
	}

	printf("stream%u, Pause!\n", stream_id);

	if (g_vout_start_cnt) {
		g_vout_start_cnt--;
	}

	// stop video_liveview modules
	hd_videocap_stop(p_stream->cap_path);
	hd_videoproc_stop(p_stream->proc_path);
	if (!g_vout_start_cnt) {
		hd_videoout_stop(g_out_path);
	}

	// unbind video_liveview modules
	hd_videocap_unbind(HD_VIDEOCAP_OUT(p_stream->vcap_id, 0));
	hd_videoproc_unbind(HD_VIDEOPROC_OUT(g_vout_id, 0));

	p_stream->resume = FALSE;

	return ret;
}

static HD_RESULT stream_stop(UINT32 stream_id)
{
	HD_RESULT ret = HD_OK;
	VIDEO_LIVEVIEW *p_stream = &g_stream[stream_id];

	if (!p_stream->start) {
		return HD_OK;
	}

	printf("stream%u, Stop!\n", stream_id);

	// close video_liveview modules
	ret = close_module(stream_id);
	if (ret != HD_OK) {
		printf("close fail=%d\n", ret);
	}

	p_stream->start = FALSE;

	return ret;
}

static void *sen_det_plug_tsk(void *arg)
{
#define max(a, b) (((a) > (b)) ? (a) : (b))
#define min(a, b) (((a) < (b)) ? (a) : (b))
	VIDEO_LIVEVIEW *p_stream;
	BOOL plugged = FALSE;
	VENDOR_VIDEOCAP_GET_PLUG_INFO plug_info = {0};
	VENDOR_VIDEOCAP_CCIR_INFO ccir_info = {0};
	static UINT32 vout_id = 0;
	UINT32 i_stream, vout_id_tmp;

	while (!g_sen_det_plug_exit) {

		// check video format change
		for (i_stream = 0; i_stream < STREAM_MAX; i_stream++) {

			p_stream = &g_stream[i_stream];

			if (p_stream->module_type == AD_MODULE_TYPE_UNKNOWN) {
				continue;
			}

			if (p_stream->det_en) { // enable detection function

				if (vendor_videocap_get(p_stream->cap_ctrl, VENDOR_VIDEOCAP_PARAM_GET_PLUG, &plugged) == HD_OK && plugged != FALSE) {
					printf("id%u, Sensor Plugged\r\n", i_stream);

					if (vendor_videocap_get(p_stream->cap_ctrl, VENDOR_VIDEOCAP_PARAM_GET_PLUG_INFO, &plug_info) == HD_OK) {

						ccir_info.interlace = plug_info.interlace;
						ccir_info.field_sel = plug_info.interlace ? VENDOR_VIDEOCAP_FIELD_EN_0 : VENDOR_VIDEOCAP_FIELD_DISABLE;
						ccir_info.fmt = p_stream->module_info->ccir_fmt;
						ccir_info.mux_data_index = p_stream->ccir_mux_index;

						printf("id%u, Size %u x %u @ %s%u.%02u\r\n", i_stream, plug_info.size.w, plug_info.size.h, ccir_info.interlace ? "i" : "p", plug_info.fps/100, plug_info.fps%100);

						if (memcmp(&plug_info.size, &p_stream->plug_info.size, sizeof(plug_info.size)) ||
							(max(plug_info.fps, p_stream->plug_info.fps) - min(plug_info.fps, p_stream->plug_info.fps) > 50) ||
							memcmp(&ccir_info, &p_stream->ccir_info, sizeof(ccir_info))) {
							printf("id%u, Size Change Detected. %ux%u@%s%u.%u => %ux%u@%s%u.%02u\r\n", i_stream,
								p_stream->plug_info.size.w, p_stream->plug_info.size.h, p_stream->ccir_info.interlace ? "i" : "p", p_stream->plug_info.fps/100, p_stream->plug_info.fps%100,
								plug_info.size.w, plug_info.size.h, ccir_info.interlace ? "i" : "p", plug_info.fps/100, plug_info.fps%100);

							p_stream->plug_info = plug_info;
							p_stream->ccir_info = ccir_info;
							goto do_restart;

						} else {
							p_stream->plug_info = plug_info;
							p_stream->ccir_info = ccir_info;
							goto do_start;
						}

					} else {
						printf("id%u, Get resolution fail\r\n", i_stream);
						goto do_stop;
					}

				} else {
					printf("id%u, Sensor Un-plugged\r\n", i_stream);
					goto do_stop;
				}

			} else { // disable detection function
				plug_info.size.w = p_stream->vdo_size_w;
				plug_info.size.h = p_stream->vdo_size_h;
				plug_info.fps = p_stream->vdo_fps * 100;
				plug_info.interlace = p_stream->vdo_interlace;
				printf("id%u, Fix plugin %u x %u @ %s%u.%02u\r\n", i_stream, plug_info.size.w, plug_info.size.h, plug_info.interlace ? "i" : "p", plug_info.fps/100, plug_info.fps%100);
			}

do_start:
			if (!p_stream->resume) { // force trigger sensor change mode when video loss before
				stream_stop(i_stream);
				stream_start(i_stream);
			}
			stream_resume(i_stream);
			continue;

do_stop:
			stream_pause(i_stream);
			continue;

do_restart:
			stream_pause(i_stream);
			stream_stop(i_stream);

			// change resolution
			p_stream->vdo_size_w = plug_info.size.w;
			p_stream->vdo_size_h = plug_info.size.h;
			p_stream->vdo_fps = plug_info.fps / 100;
			p_stream->vdo_interlace = plug_info.interlace;

			stream_start(i_stream);
			stream_resume(i_stream);
		}

		// check vout_id setting change
		if (vout_id != g_vout_id) {
			printf("change vout_id %u -> %u\r\n", vout_id, g_vout_id);

			vout_id_tmp = g_vout_id;
			g_vout_id = vout_id;
			for (i_stream = 0; i_stream < STREAM_MAX; i_stream++) {
				p_stream = &g_stream[i_stream];
				if (p_stream->module_type == AD_MODULE_TYPE_UNKNOWN) {
					continue;
				}
				hd_videocap_stop(p_stream->cap_path);
				hd_videoproc_stop(p_stream->proc_path);
			}
			hd_videoout_stop(g_out_path);
			hd_videoproc_unbind(HD_VIDEOPROC_OUT(g_vout_id, 0));
			g_vout_id = vout_id_tmp;
			hd_videoproc_bind(HD_VIDEOPROC_OUT(g_vout_id, 0), HD_VIDEOOUT_0_IN_0);
			for (i_stream = 0; i_stream < STREAM_MAX; i_stream++) {
				p_stream = &g_stream[i_stream];
				if (p_stream->module_type == AD_MODULE_TYPE_UNKNOWN) {
					continue;
				}
				set_proc_param(i_stream);
			}
			for (i_stream = 0; i_stream < STREAM_MAX; i_stream++) {
				p_stream = &g_stream[i_stream];
				if (p_stream->module_type == AD_MODULE_TYPE_UNKNOWN) {
					continue;
				}
				hd_videocap_start(p_stream->cap_path);
				hd_videoproc_start(p_stream->proc_path);
			}
			hd_videoout_start(g_out_path);
			vout_id = g_vout_id;
		}

		usleep(1000*1000);
	}

	return 0;
}

static UINT32 get_input(int argc, char** argv)
{
	enum {
		IN_ARG_MODULE_TYPE = 0,
		IN_ARG_CHIP_ID,
		IN_ARG_VIN_ID,
		IN_ARG_VCAP_ID,
		IN_ARG_CCIR_MUX_INDEX,
		IN_ARG_MIPI_DATA_LANE,
		IN_ARG_DET_EN,
		IN_ARG_VDO_SIZE_W,
		IN_ARG_VDO_SIZE_H,
		IN_ARG_VDO_FPS,
		IN_ARG_VDO_INTERLACE,

		IN_ARG_NUM
	};

	char module_type_str[][16] = {
		[AD_MODULE_TYPE_UNKNOWN] = "unknown",
		[AD_MODULE_TYPE_NVP6124B] = "nvp6124b",
		[AD_MODULE_TYPE_TP9950] = "tp9950",
		[AD_MODULE_TYPE_TC358743] = "tc358743",
		[AD_MODULE_TYPE_TC358840] = "tc358840",
		[AD_MODULE_TYPE_TP2854] = "tp2854",
		[AD_MODULE_TYPE_TP2855] = "tp2855",
		[AD_MODULE_TYPE_NVP6188] = "nvp6188",
		[AD_MODULE_TYPE_TP2860] = "tp2860",		
		[AD_MODULE_TYPE_TP2863] = "tp2863",		
		[AD_MODULE_TYPE_TP9963] = "tp9963",		
	};
	char ccir_fmt_str[][8] = {
		[VENDOR_VIDEOCAP_CCIR_FMT_SEL_CCIR601] = "601",
		[VENDOR_VIDEOCAP_CCIR_FMT_SEL_CCIR656] = "656",
	};
	VIDEO_LIVEVIEW *p_stream, *p_stream_tmp;
	UINT32 i, j, stream_num;

	if (argc == 0 || (argc % IN_ARG_NUM)) {
		printf("Wrong input arg num %d\r\n", argc);
		printf("module chip vin vcap mux_id lane_num det w h fps interlace\r\n");
		printf("module - ");
		for (i = 1; i < AD_MODULE_TYPE_MAX; i++) {
			printf("%u:%s ", i-1, module_type_str[i]);
		}
		printf("\r\n");
		printf("chip - 0~7\r\n");
		printf("vin - 0~7\r\n");
		printf("vcap - 0~7\r\n");
		printf("mux_id - 0~1\r\n");
		printf("lane_num - 1~4 (only for mipi)\r\n");
		printf("det - 0~1\r\n");
		printf("w - 0~%u (if det==1, set 0 to ignore)\r\n", VDO_MEM_SIZE_W);
		printf("h - 0~%u (if det==1, set 0 to ignore)\r\n", VDO_MEM_SIZE_H);
		printf("fps - 0~60 (if det==1, set 0 to ignore)\r\n");
		printf("interlace - 0~1 (if det==1, set 0 to ignore)\r\n");
		return 0;
	}

	i = 0;
	while ((int)i*IN_ARG_NUM < argc) {
		p_stream = &g_stream[i];
		p_stream->module_type 				= atoi(argv[(i*IN_ARG_NUM)+IN_ARG_MODULE_TYPE])+1;
		p_stream->chip_id 					= atoi(argv[(i*IN_ARG_NUM)+IN_ARG_CHIP_ID]);
		p_stream->vin_id 					= atoi(argv[(i*IN_ARG_NUM)+IN_ARG_VIN_ID]);
		p_stream->vcap_id 					= atoi(argv[(i*IN_ARG_NUM)+IN_ARG_VCAP_ID]);
		p_stream->ccir_mux_index 			= atoi(argv[(i*IN_ARG_NUM)+IN_ARG_CCIR_MUX_INDEX]);
		p_stream->mipi_data_lane			= atoi(argv[(i*IN_ARG_NUM)+IN_ARG_MIPI_DATA_LANE]);
		p_stream->det_en 					= atoi(argv[(i*IN_ARG_NUM)+IN_ARG_DET_EN]);
		p_stream->vdo_size_w 				= atoi(argv[(i*IN_ARG_NUM)+IN_ARG_VDO_SIZE_W]);
		p_stream->vdo_size_h 				= atoi(argv[(i*IN_ARG_NUM)+IN_ARG_VDO_SIZE_H]);
		p_stream->vdo_fps 					= atoi(argv[(i*IN_ARG_NUM)+IN_ARG_VDO_FPS]);
		p_stream->vdo_interlace 			= atoi(argv[(i*IN_ARG_NUM)+IN_ARG_VDO_INTERLACE]);

		p_stream->module_info = &g_ad_module_info[p_stream->module_type];
		p_stream->vproc_id = i;
		p_stream->ccir_info.field_sel = p_stream->vdo_interlace ? VENDOR_VIDEOCAP_FIELD_EN_0 : VENDOR_VIDEOCAP_FIELD_DISABLE;;
		p_stream->ccir_info.fmt = g_ad_module_info[p_stream->module_type].ccir_fmt;
		p_stream->ccir_info.interlace = p_stream->vdo_interlace;
		p_stream->ccir_info.mux_data_index 	= p_stream->ccir_mux_index;

		i++;
	}

	stream_num = i;
	for (i = 0; i < stream_num; i++) {
		p_stream = &g_stream[i];

		p_stream->vcap_id_bit = 0;

		if (p_stream->module_info->bus_type != HD_COMMON_VIDEO_IN_MIPI_CSI) {
			continue;
		}

		for (j = 0; j < stream_num; j++) {
			p_stream_tmp = &g_stream[j];

			if (p_stream->module_type == p_stream_tmp->module_type && p_stream->chip_id == p_stream_tmp->chip_id) {

				switch (p_stream_tmp->vcap_id) {
				case 0:
					p_stream->vcap_id_bit |= HD_VIDEOCAP_0;
					break;
				case 1:
					p_stream->vcap_id_bit |= HD_VIDEOCAP_1;
					break;
				case 2:
					p_stream->vcap_id_bit |= HD_VIDEOCAP_2;
					break;
				case 3:
					p_stream->vcap_id_bit |= HD_VIDEOCAP_3;
					break;
				case 4:
					p_stream->vcap_id_bit |= HD_VIDEOCAP_4;
					break;
				case 5:
					p_stream->vcap_id_bit |= HD_VIDEOCAP_5;
					break;
				case 6:
					p_stream->vcap_id_bit |= HD_VIDEOCAP_6;
					break;
				case 7:
					p_stream->vcap_id_bit |= HD_VIDEOCAP_7;
					break;
				default:
					printf("Stream %u wrong vcap id %u\r\n", j, p_stream_tmp->vcap_id);
					p_stream->vcap_id_bit |= HD_VIDEOCAP_0;
				}
			}
		}
	}

	for (i = 0; i < stream_num; i++) {
		p_stream = &g_stream[i];
		printf("Stream%u, module_type=%s chip_id=%u vin_id=%u vcap_id=%u ccir_mux_index=%u "
			"data_lane=%u det_en=%u vdo_size_w=%u vdo_size_h=%u vdo_fps=%u vdo_interlace=%u "
			"ccir_fmt=%s vproc_id=%u vcap_id_bit=0x%x field_sel=%u\r\n",
			i, module_type_str[p_stream->module_type], p_stream->chip_id, p_stream->vin_id, p_stream->vcap_id, p_stream->ccir_mux_index,
			p_stream->mipi_data_lane, p_stream->det_en, p_stream->vdo_size_w, p_stream->vdo_size_h, p_stream->vdo_fps, p_stream->vdo_interlace,
			ccir_fmt_str[p_stream->module_info->ccir_fmt], p_stream->vproc_id, (unsigned int)p_stream->vcap_id_bit, p_stream->ccir_info.field_sel);
	}

	return stream_num;
}

MAIN(argc, argv)
{
	HD_RESULT ret;
	INT key;
	VIDEO_LIVEVIEW *p_stream;
	UINT32 mem_reg = 0;
	INT32 narg[10] = {0};
	CHAR str_buf [32] = {0};
	UINT32 stream_num, i_stream;

	printf("\r\n\r\nAP version " AP_VERSION "\r\n\r\n");

	g_sen_det_plug_exit = 0;

	stream_num = get_input(argc-1, &argv[1]);
	if (stream_num == 0) {
		printf("get_input fail\n");
		goto exit;
	}

	// init hdal
	ret = hd_common_init(0);
	if (ret != HD_OK) {
		printf("common fail=%d\n", ret);
		goto exit;
	}

	// init memory
	ret = mem_init(stream_num);
	if (ret != HD_OK) {
		printf("mem fail=%d\n", ret);
		goto exit;
	}

	// init all modules
	ret = init_module();
	if (ret != HD_OK) {
		printf("init fail=%d\n", ret);
		goto exit;
	}

	for (i_stream = 0; i_stream < STREAM_MAX; i_stream++) {

		p_stream = &g_stream[i_stream];

		if (p_stream->module_type == AD_MODULE_TYPE_UNKNOWN) {
			continue;
		}

		printf("================== Stream %u ==================\n", i_stream);

		if (p_stream->module_info == NULL) {
			printf("NULL module_info\n");
			goto exit;
		}
	
		ret = stream_start(i_stream);
		if (ret < 0) {
			printf("start fail\n");
			goto exit;
		}
	}
	
	printf("start fail\n");
	ret = pthread_create(&g_sen_det_plug_hdl, NULL, sen_det_plug_tsk, NULL);
	if (ret < 0) {
		printf("create thread fail\n");
		goto exit;
	}
	
	// query user key
	printf("Enter q to exit\n");
	while (1) {
		key = GETCHAR();
		if (key == 'q' || key == 0x3) {
			// quit program
			g_sen_det_plug_exit = 1;
			break;
		}

		#if (DEBUG_MENU == 1)
		if (key == 'd') {
			// enter debug menu
			hd_debug_run_menu();
			printf("\r\nEnter q to exit, Enter d to debug\r\n");
		}
		#endif
		if (key == '0') {
			get_cap_sysinfo(0);
		}

		if (key == 's') {

			while (1) {
				key = GETCHAR();

				if (key == 'q' || key == 0x3) {
					break;
				} else {
					narg[0] = key - '0';

					if (narg[0] >= 0 && narg[0] < STREAM_MAX) {
						g_vout_id = narg[0];
					} else {
						printf("Enter which stream id (0~%u) you to show. Enter q to exit\n", STREAM_MAX-1);
					}
				}

				usleep(1000*1000);
			}
		}

		if (key == 'c') {
			printf("Enter csi0 virtual channel id0 (sie1)\n");
			while (1) {
				key = GETCHAR();

				if (key == 'q' || key == 0x3) {
					break;
				} else {
					narg[0] = key - '0';

					if (narg[0] >= 0 && narg[0] < 4) {
						mem_reg = 0x6A020103 | (narg[0] << 12);
						sprintf(str_buf, "mem w f0280004 %x", (unsigned int)mem_reg);
						printf("Set virtual channel %s\n", str_buf);
#if defined(__LINUX)
						system(str_buf);
#endif
					}
				}

				usleep(1000*1000);
			}
		}

		usleep(1000*1000);
	}

	pthread_join(g_sen_det_plug_hdl, NULL);


exit:
	for (i_stream = 0; i_stream < STREAM_MAX; i_stream++) {

		p_stream = &g_stream[i_stream];

		if (p_stream->module_type == AD_MODULE_TYPE_UNKNOWN) {
			continue;
		}

		ret = stream_pause(i_stream);
		if (ret < 0) {
			printf("pause fail\n");
		}

		ret = stream_stop(i_stream);
		if (ret < 0) {
			printf("stop fail\n");
		}
	}

	// uninit all modules
	ret = exit_module();
	if (ret != HD_OK) {
		printf("exit fail=%d\n", ret);
	}

	// uninit memory
	ret = mem_exit();
	if (ret != HD_OK) {
		printf("mem fail=%d\n", ret);
	}

	// uninit hdal
	ret = hd_common_uninit();
	if (ret != HD_OK) {
		printf("common fail=%d\n", ret);
	}

	return 0;
}
