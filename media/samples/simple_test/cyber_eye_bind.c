#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

#include <assert.h>
#include <errno.h>
#include <error.h>
#include <fcntl.h>
#include <getopt.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/poll.h>
#include <time.h>
#include <unistd.h>

#include <rk_aiq_user_api2_camgroup.h>
#include <rk_aiq_user_api2_imgproc.h>
#include <rk_aiq_user_api2_sysctl.h>

#include "rk_debug.h"
#include "rk_defines.h"
#include "rk_mpi_adec.h"
#include "rk_mpi_aenc.h"
#include "rk_mpi_ai.h"
#include "rk_mpi_ao.h"
#include "rk_mpi_avs.h"
#include "rk_mpi_cal.h"
#include "rk_mpi_ivs.h"
#include "rk_mpi_mb.h"
#include "rk_mpi_rgn.h"
#include "rk_mpi_sys.h"
#include "rk_mpi_tde.h"
#include "rk_mpi_vdec.h"
#include "rk_mpi_venc.h"
#include "rk_mpi_vi.h"
#include "rk_mpi_vo.h"
#include "rk_mpi_vpss.h"

/* ------------------------------------------------------------------------ */
/* AIQ context management                                                    */
/* ------------------------------------------------------------------------ */

#define MAX_AIQ_CTX 8
static rk_aiq_sys_ctx_t *g_aiq_ctx[MAX_AIQ_CTX];
rk_aiq_working_mode_t g_WDRMode[MAX_AIQ_CTX];

static atomic_int g_sof_cnt = 0;
static atomic_bool g_should_quit_aiq = false;

/* ------------------------------------------------------------------------ */
/* Globals                                                                    */
/* ------------------------------------------------------------------------ */

#define BUFFER_SIZE 255

static RK_U64 g_s32FrameCnt = (RK_U64)-1; /* -1 == unlimited */
static volatile bool g_quit = false;
static FILE *g_venc0_file = NULL;

/* ------------------------------------------------------------------------ */
/* Signal handling                                                            */
/* ------------------------------------------------------------------------ */

static void sigterm_handler(int sig) {
    RK_LOGE("signal %d", sig);
	g_quit = true;
}

/* ------------------------------------------------------------------------ */
/* Helpers                                                                    */
/* ------------------------------------------------------------------------ */

RK_U64 TEST_COMM_GetNowUs() {
	struct timespec time = {0, 0};
	clock_gettime(CLOCK_MONOTONIC, &time);
	return (RK_U64)time.tv_sec * 1000000 + (RK_U64)time.tv_nsec / 1000; /* microseconds */
}

/* ------------------------------------------------------------------------ */
/* AIQ / ISP init / run / stop (adapted from multi-channel demo)             */
/* ------------------------------------------------------------------------ */

static XCamReturn SIMPLE_COMM_ISP_SofCb(rk_aiq_metas_t *meta) {
	g_sof_cnt++;
	if (g_sof_cnt <= 2)
		RK_LOGI("=== %u ===", meta->frame_id);
	return XCAM_RETURN_NO_ERROR;
}

static XCamReturn SIMPLE_COMM_ISP_ErrCb(rk_aiq_err_msg_t *msg) {
	if (msg->err_code == XCAM_RETURN_BYPASS)
		g_should_quit_aiq = true;
	return XCAM_RETURN_NO_ERROR;
}

RK_S32 SIMPLE_COMM_ISP_Init(RK_S32 CamId, rk_aiq_working_mode_t WDRMode, RK_BOOL MultiCam,
                            const char *iq_file_dir) {
	if (CamId >= MAX_AIQ_CTX) {
		RK_PRINT("%s : CamId is over %d\n", __FUNCTION__, MAX_AIQ_CTX);
		return -1;
	}

	setlinebuf(stdout);
	if (iq_file_dir == NULL) {
		RK_PRINT("SIMPLE_COMM_ISP_Init : not start.\n");
		g_aiq_ctx[CamId] = NULL;
		return 0;
	}

	/* must set HDR_MODE before init */
	g_WDRMode[CamId] = WDRMode;
	char hdr_str[16];
    snprintf(hdr_str, sizeof(hdr_str), "%d", (int)WDRMode);
	setenv("HDR_MODE", hdr_str, 1);

	rk_aiq_sys_ctx_t *aiq_ctx;
	rk_aiq_static_info_t aiq_static_info;

	rk_aiq_uapi2_sysctl_enumStaticMetas(CamId, &aiq_static_info);

	RK_PRINT("ID: %d, sensor_name is %s, iqfiles is %s\n", CamId,
	       aiq_static_info.sensor_info.sensor_name, iq_file_dir);

	aiq_ctx =
	    rk_aiq_uapi2_sysctl_init(aiq_static_info.sensor_info.sensor_name, iq_file_dir,
	                             SIMPLE_COMM_ISP_ErrCb, SIMPLE_COMM_ISP_SofCb);

	if (MultiCam)
		rk_aiq_uapi2_sysctl_setMulCamConc(aiq_ctx, true);

//     rk_aiq_rotation_t rot = RK_AIQ_ROTATION_90;
//     rk_aiq_uapi2_sysctl_setSharpFbcRotation(aiq_ctx, rot);

//     rk_aiq_mems_sensor_intf_t intf = {0};
//     const char* main_scene = "good";
//     const char* sub_scene = "bad";
//     rk_aiq_uapi2_sysctl_setMulCamConc(aiq_ctx, true);
//     rk_aiq_uapi2_sysctl_regMemsSensorIntf(aiq_ctx, &intf);
//     rk_aiq_uapi2_sysctl_switch_scene(aiq_ctx, main_scene, sub_scene);


	g_aiq_ctx[CamId] = aiq_ctx;
	return 0;
}

RK_S32 SIMPLE_COMM_ISP_Run(RK_S32 CamId) {
	if (CamId >= MAX_AIQ_CTX || !g_aiq_ctx[CamId]) {
		RK_PRINT("%s : CamId is over %d or not init\n", __FUNCTION__, MAX_AIQ_CTX);
		return -1;
	}

	if (rk_aiq_uapi2_sysctl_prepare(g_aiq_ctx[CamId], 0, 0, g_WDRMode[CamId])) {
		RK_PRINT("rkaiq engine prepare failed !\n");
		g_aiq_ctx[CamId] = NULL;
		return -1;
	}
	RK_PRINT("rk_aiq_uapi2_sysctl_init/prepare succeed\n");
	if (rk_aiq_uapi2_sysctl_start(g_aiq_ctx[CamId])) {
		RK_PRINT("rk_aiq_uapi2_sysctl_start  failed\n");
		return -1;
	}
	RK_PRINT("rk_aiq_uapi2_sysctl_start succeed\n");

	return 0;
}

RK_S32 SIMPLE_COMM_ISP_Stop(RK_S32 CamId) {
	if (CamId >= MAX_AIQ_CTX || !g_aiq_ctx[CamId]) {
		RK_PRINT("%s : CamId is over %d or not init g_aiq_ctx[%d] = %p\n", __FUNCTION__,
		       MAX_AIQ_CTX, CamId, g_aiq_ctx[CamId]);
		return -1;
	}
	RK_PRINT("rk_aiq_uapi2_sysctl_stop enter\n");
	rk_aiq_uapi2_sysctl_stop(g_aiq_ctx[CamId], false);
	RK_PRINT("rk_aiq_uapi2_sysctl_deinit enter\n");
	rk_aiq_uapi2_sysctl_deinit(g_aiq_ctx[CamId]);
	RK_PRINT("rk_aiq_uapi2_sysctl_deinit exit\n");

	g_aiq_ctx[CamId] = NULL;
	return 0;
}

/* ------------------------------------------------------------------------ */
/* VI init (single dev/pipe, channel attr)                                   */
/* ------------------------------------------------------------------------ */

static int vi_dev_init(void) {
	RK_LOGI("%s", __func__);
	int ret = 0;
	int devId = 0;
	int pipeId = devId;

	VI_DEV_ATTR_S stDevAttr;
	VI_DEV_BIND_PIPE_S stBindPipe;
	memset(&stDevAttr, 0, sizeof(stDevAttr));
	memset(&stBindPipe, 0, sizeof(stBindPipe));

	/* 0. get dev config status */
	ret = RK_MPI_VI_GetDevAttr(devId, &stDevAttr);
	if (ret == RK_ERR_VI_NOT_CONFIG) {
		/* 0-1. config dev */
		ret = RK_MPI_VI_SetDevAttr(devId, &stDevAttr);
		if (ret != RK_SUCCESS) {
			RK_LOGI("RK_MPI_VI_SetDevAttr %x", ret);
			return -1;
		}
	} else {
		RK_LOGI("RK_MPI_VI_SetDevAttr already");
	}

	/* 1. get dev enable status */
	ret = RK_MPI_VI_GetDevIsEnable(devId);
	if (ret != RK_SUCCESS) {
		/* 1-2. enable dev */
		ret = RK_MPI_VI_EnableDev(devId);
		if (ret != RK_SUCCESS) {
			RK_LOGI("RK_MPI_VI_EnableDev %x", ret);
			return -1;
		}
		/* 1-3. bind dev/pipe */
		stBindPipe.u32Num = 1;
		stBindPipe.PipeId[0] = pipeId;
		ret = RK_MPI_VI_SetDevBindPipe(devId, &stBindPipe);
		if (ret != RK_SUCCESS) {
			RK_LOGI("RK_MPI_VI_SetDevBindPipe %x", ret);
			return -1;
		}
	} else {
		RK_LOGI("RK_MPI_VI_EnableDev already");
	}

	return 0;
}

static int vi_chn_init(int channelId, int width, int height) {
	int ret;
	int buf_cnt = 2;

	VI_CHN_ATTR_S vi_chn_attr;
	memset(&vi_chn_attr, 0, sizeof(vi_chn_attr));
	vi_chn_attr.stIspOpt.u32BufCount = buf_cnt;
	vi_chn_attr.stIspOpt.enMemoryType = VI_V4L2_MEMORY_TYPE_DMABUF;
	vi_chn_attr.stSize.u32Width = width;
	vi_chn_attr.stSize.u32Height = height;
    // 	Sadly SC3336 does not support higher frame rates
	vi_chn_attr.stFrameRate.s32SrcFrameRate = 30;
	vi_chn_attr.stFrameRate.s32DstFrameRate = 30;
	vi_chn_attr.bMirror = RK_FALSE;
	vi_chn_attr.bFlip = RK_FALSE;
	vi_chn_attr.enPixelFormat = RK_FMT_YUV420SP;
	vi_chn_attr.enCompressMode = COMPRESS_MODE_NONE;
	/* 0: get fail, 1..u32BufCount: ok; if bound to another device, must be < u32BufCount */
	vi_chn_attr.u32Depth = 0;
	ret = RK_MPI_VI_SetChnAttr(0, channelId, &vi_chn_attr);
	ret |= RK_MPI_VI_EnableChn(0, channelId);
	if (ret) {
		RK_LOGI("ERROR: create VI error! ret=%d", ret);
		return ret;
	}

	return ret;
}

/* ------------------------------------------------------------------------ */
/* VENC init (H264)                                                         */
/* ------------------------------------------------------------------------ */

static RK_S32 venc_init(int chnId, int width, int height) {
	RK_LOGI("========%s========\n", __func__);
	VENC_RECV_PIC_PARAM_S stRecvParam;
	VENC_CHN_ATTR_S stAttr;
	memset(&stAttr, 0, sizeof(VENC_CHN_ATTR_S));

	stAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
    stAttr.stRcAttr.stH264Cbr.u32BitRate = 10 * 1024;
    stAttr.stRcAttr.stH264Cbr.u32Gop = 30;

	stAttr.stVencAttr.enType = RK_VIDEO_ID_AVC;
	stAttr.stVencAttr.enPixelFormat = RK_FMT_YUV420SP;
	stAttr.stVencAttr.u32Profile = H264E_PROFILE_HIGH;
	stAttr.stVencAttr.u32PicWidth = width;
	stAttr.stVencAttr.u32PicHeight = height;
	stAttr.stVencAttr.u32VirWidth = width;
	stAttr.stVencAttr.u32VirHeight = height;
	stAttr.stVencAttr.u32StreamBufCnt = 1;
	stAttr.stVencAttr.u32BufSize = width * height * 3 / 2;
	stAttr.stVencAttr.enMirror = MIRROR_NONE;
// 	stAttr.stVencAttr.enMirror = MIRROR_VERTICAL;
// 	stAttr.stVencAttr.enMirror = MIRROR_HORIZONTAL;

	RK_MPI_VENC_CreateChn(chnId, &stAttr);

    RK_S32 s32Ret = RK_FAILURE;
	s32Ret = RK_MPI_VENC_SetChnRotation(chnId, ROTATION_180);
	if (s32Ret != RK_SUCCESS) {
		RK_LOGE("RK_MPI_VENC_SetChnRotation failure:%X", s32Ret);
	}

	memset(&stRecvParam, 0, sizeof(VENC_RECV_PIC_PARAM_S));
	stRecvParam.s32RecvPicNum = -1;
	RK_MPI_VENC_StartRecvFrame(chnId, &stRecvParam);

	return 0;
}

/* ------------------------------------------------------------------------ */
/* VENC stream-getting thread                                                 */
/* ------------------------------------------------------------------------ */

static void *GetMediaBuffer0(void *arg) {
	(void)arg;
	RK_LOGI("========%s========", __func__);
	void *pData = RK_NULL;
	RK_U64 loopCount = 0;
	int s32Ret;

	VENC_STREAM_S stFrame;
	stFrame.pstPack = malloc(sizeof(VENC_PACK_S));
	if (!stFrame.pstPack) {
		RK_LOGE("malloc for stFrame.pstPack failure");
		return NULL;
	}

	while (!g_quit) {
		s32Ret = RK_MPI_VENC_GetStream(0, &stFrame, -1);
		if (s32Ret == RK_SUCCESS) {
			if (g_venc0_file) {
				pData = RK_MPI_MB_Handle2VirAddr(stFrame.pstPack->pMbBlk);
				fwrite(pData, 1, stFrame.pstPack->u32Len, g_venc0_file);
				fflush(g_venc0_file);
			}
			RK_U64 nowUs = TEST_COMM_GetNowUs();

			RK_LOGD("chn:0, loopCount:%llu enc->seq:%d wd:%d pts=%lld delay=%lldus",
			        (unsigned long long)loopCount, stFrame.u32Seq, stFrame.pstPack->u32Len,
			        stFrame.pstPack->u64PTS, nowUs - stFrame.pstPack->u64PTS);

			s32Ret = RK_MPI_VENC_ReleaseStream(0, &stFrame);
			if (s32Ret != RK_SUCCESS) {
				RK_LOGE("RK_MPI_VENC_ReleaseStream fail %x", s32Ret);
			}
			loopCount++;
		} else {
			RK_LOGE("RK_MPI_VENC_GetStream fail %x", s32Ret);
		}

		if ((g_s32FrameCnt != (RK_U64)-1) && (loopCount > g_s32FrameCnt)) {
			g_quit = true;
			break;
		}

		usleep(10 * 1000);
	}

	if (g_venc0_file) {
		fclose(g_venc0_file);
		g_venc0_file = NULL;
	}

	free(stFrame.pstPack);
	return NULL;
}

/* ------------------------------------------------------------------------ */
/* Usage                                                                      */
/* ------------------------------------------------------------------------ */

static RK_CHAR optstr[] = "?::w:h:c:I:e:o:a::";

static const struct option long_options[] = {
    {"width", required_argument, NULL, 'w'},
    {"height", required_argument, NULL, 'h'},
    {"frame_cnt", required_argument, NULL, 'c'},
    {"camid", required_argument, NULL, 'I'},
    {"encode", required_argument, NULL, 'e'},
    {"output", required_argument, NULL, 'o'},
    {"aiq", optional_argument, NULL, 'a'},
    {"help", optional_argument, NULL, '?'},
    {NULL, 0, NULL, 0},
};

static void print_usage(const RK_CHAR *name) {
	RK_PRINT("usage example:\n");
	RK_PRINT("\t%s -I 0 -w 1280 -h 720 -e h264 -o /tmp/venc.h264 -a /etc/iqfiles/\n", name);
	RK_PRINT("\t-w | --width:     VI width, Default: 1280\n");
	RK_PRINT("\t-h | --height:    VI height, Default: 720\n");
	RK_PRINT("\t-c | --frame_cnt: frame count of output, Default: -1 (unlimited)\n");
	RK_PRINT("\t-I | --camid:     VI channel id, Default: 0. "
	        "0:rkisp_mainpath,1:rkisp_selfpath,2:rkisp_bypasspath\n");
	RK_PRINT("\t-o | --output:    output file path, Default: NULL\n");
	RK_PRINT("\t-a | --aiq:       enable AIQ/ISP with iqfiles dir, e.g. -a /etc/iqfiles/, \n"
	        "empty path uses the default location.\n"
	        "\t                  Without this option AIQ must be started by another \n"
	        "application.\n");
}

/* ------------------------------------------------------------------------ */
/* main                                                                       */
/* ------------------------------------------------------------------------ */

int main(int argc, char *argv[]) {
	RK_S32 s32Ret = RK_FAILURE;
	RK_U32 u32Width = 1280;
	RK_U32 u32Height = 720;
	RK_CHAR *pOutPath = NULL;
	RK_CHAR *pCodecName = "H264";
	RK_S32 s32chnlId = 0;
	int c;
	int ret = -1;
	char *iq_file_dir = NULL;
	RK_S32 s32CamId = 0;
	rk_aiq_working_mode_t hdr_mode = RK_AIQ_WORKING_MODE_NORMAL;
	RK_BOOL bMultictx = RK_FALSE;
	bool aiq_started = false;

	while ((c = getopt_long(argc, argv, optstr, long_options, NULL)) != -1) {
		const char *tmp_optarg = optarg;
		switch (c) {
		case 'w':
			u32Width = atoi(optarg);
			break;
		case 'h':
			u32Height = atoi(optarg);
			break;
		case 'I':
			s32chnlId = atoi(optarg);
			break;
		case 'c':
			g_s32FrameCnt = (RK_U64)atoi(optarg);
			break;
		case 'o':
			pOutPath = optarg;
			break;
		case 'a':
			if (!optarg && NULL != argv[optind] && '-' != argv[optind][0]) {
				tmp_optarg = argv[optind++];
			}
			iq_file_dir = tmp_optarg ? (char *)tmp_optarg : NULL;
			break;
		case '?':
		default:
			print_usage(argv[0]);
			return -1;
		}
	}

	RK_PRINT("# VERSION: Cyber-eye-1.0.1\n");
	RK_PRINT("CodecName: %s\n", pCodecName);
	RK_PRINT("Resolution: %dx%d\n", u32Width, u32Height);
	RK_PRINT("Output Path: %s\n", pOutPath);
	RK_PRINT("CameraIdx: %d\n", s32chnlId);
	RK_PRINT("Frame Count to save: %lld\n", (long long)g_s32FrameCnt);
	RK_PRINT("IQ Path: %s\n\n", iq_file_dir);

	signal(SIGINT, sigterm_handler);
	signal(SIGTERM, sigterm_handler);

	/* -------------------------------------------------------------- */
	/* Start AIQ/ISP first (it must run before MPI/VI starts streaming) */
	/* -------------------------------------------------------------- */
	if (iq_file_dir) {
		RK_PRINT("Rkaiq XML DirPath: %s\n", iq_file_dir);
		RK_PRINT("bMultictx: %d\n\n", bMultictx);

		s32Ret = SIMPLE_COMM_ISP_Init(s32CamId, hdr_mode, bMultictx, iq_file_dir);
		s32Ret |= SIMPLE_COMM_ISP_Run(s32CamId);
		if (s32Ret != RK_SUCCESS) {
			RK_PRINT("ISP init failure:%X\n", s32Ret);
			return -1;
		}
		aiq_started = true;
	}

	if (pOutPath) {
		g_venc0_file = fopen(pOutPath, "w");
		if (!g_venc0_file) {
			RK_PRINT("ERROR: open file: %s fail, exit\n", pOutPath);
			if (aiq_started)
				SIMPLE_COMM_ISP_Stop(s32CamId);
			return 0;
		}
	}

	if (RK_MPI_SYS_Init() != RK_SUCCESS) {
		RK_PRINT("rk mpi sys init fail!\n");
		goto __FAILED;
	}

	vi_dev_init();
	vi_chn_init(s32chnlId, u32Width, u32Height);

	/* venc init */
	venc_init(0, u32Width, u32Height);

	MPP_CHN_S stSrcChn, stDestChn;
	/* bind vi to venc */
	stSrcChn.enModId = RK_ID_VI;
	stSrcChn.s32DevId = 0;
	stSrcChn.s32ChnId = s32chnlId;

	stDestChn.enModId = RK_ID_VENC;
	stDestChn.s32DevId = 0;
	stDestChn.s32ChnId = 0;
	RK_LOGI("====RK_MPI_SYS_Bind vi%d to venc0====", s32chnlId);
	s32Ret = RK_MPI_SYS_Bind(&stSrcChn, &stDestChn);
	if (s32Ret != RK_SUCCESS) {
		RK_LOGE("bind vi%d to venc0 failed:%x", s32chnlId, s32Ret);
		goto __FAILED;
	}

	pthread_t main_thread;
	pthread_create(&main_thread, NULL, GetMediaBuffer0, NULL);

	RK_LOGI("%s initial finish", __func__);

	while (!g_quit) {
		if (g_should_quit_aiq)
			g_quit = true;
		usleep(50000);
	}
	pthread_join(main_thread, NULL);

	s32Ret = RK_MPI_SYS_UnBind(&stSrcChn, &stDestChn);
	if (s32Ret != RK_SUCCESS) {
		RK_LOGE("RK_MPI_SYS_UnBind fail %x", s32Ret);
	}

	s32Ret = RK_MPI_VI_DisableChn(0, s32chnlId);
	if (s32Ret != RK_SUCCESS)
		RK_LOGE("RK_MPI_VI_DisableChn %x", s32Ret);

	s32Ret = RK_MPI_VENC_StopRecvFrame(0);
	if (s32Ret != RK_SUCCESS) {
		RK_LOGE("RK_MPI_VENC_StopRecvFrame fail %x", s32Ret);
	}

	s32Ret = RK_MPI_VENC_DestroyChn(0);
	if (s32Ret != RK_SUCCESS) {
		RK_LOGE("RK_MPI_VENC_DestroyChn fail %x", s32Ret);
	}

	s32Ret = RK_MPI_VI_DisableDev(0);
	if (s32Ret != RK_SUCCESS)
		RK_LOGE("RK_MPI_VI_DisableDev %x", s32Ret);

	ret = 0;

__FAILED:
	RK_LOGE("test running exit:%d", s32Ret);
	RK_MPI_SYS_Exit();

	if (aiq_started)
		SIMPLE_COMM_ISP_Stop(s32CamId);

	return ret;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */