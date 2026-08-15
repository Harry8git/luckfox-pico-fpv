#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/poll.h>
#include <time.h>
#include <unistd.h>

#include "rtsp_demo.h"
#ifdef RV1126_RV1109
#include <rk_aiq_user_api_camgroup.h>
#include <rk_aiq_user_api_imgproc.h>
#include <rk_aiq_user_api_sysctl.h>
#else
#include <rk_aiq_user_api2_camgroup.h>
#include <rk_aiq_user_api2_imgproc.h>
#include <rk_aiq_user_api2_sysctl.h>
#endif

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

#define MAX_AIQ_CTX 8
static rk_aiq_sys_ctx_t *g_aiq_ctx[MAX_AIQ_CTX];
rk_aiq_working_mode_t g_WDRMode[MAX_AIQ_CTX];
#include <stdatomic.h>
static atomic_int g_sof_cnt = 0;
static atomic_bool g_should_quit = false;

static RK_S32 g_s32FrameCnt = -1;
static RK_U32 g_u32Bitrate = 2400; /* 2.4 Mbps CBR */
static bool quit = false;

rtsp_demo_handle g_rtsplive = NULL;
static rtsp_session_handle g_rtsp_session;

static void sigterm_handler(int sig) {
	fprintf(stderr, "signal %d, exiting...\n", sig);
	quit = true;
}

RK_U64 TEST_COMM_GetNowUs() {
	struct timespec time = {0, 0};
	clock_gettime(CLOCK_MONOTONIC, &time);
	return (RK_U64)time.tv_sec * 1000000 + (RK_U64)time.tv_nsec / 1000;
}

static void *GetMediaBuffer0(void *arg) {
	(void)arg;
	printf("========%s Started (Low Latency Mode)========\n", __func__);
	void *pData = RK_NULL;
	int loopCount = 0;
	int s32Ret;

	VENC_STREAM_S stFrame;
	stFrame.pstPack = malloc(sizeof(VENC_PACK_S));
	if (!stFrame.pstPack) {
		RK_LOGE("Failed to allocate VENC_PACK_S memory");
		return NULL;
	}

	while (!quit) {
		/* Blocking wait with 200ms timeout for direct stream ingest */
		s32Ret = RK_MPI_VENC_GetStream(0, &stFrame, 200);
		if (s32Ret == RK_SUCCESS) {
			if (g_rtsplive && g_rtsp_session) {
				pData = RK_MPI_MB_Handle2VirAddr(stFrame.pstPack->pMbBlk);
				rtsp_tx_video(g_rtsp_session, pData, stFrame.pstPack->u32Len,
				              stFrame.pstPack->u64PTS);
				rtsp_do_event(g_rtsplive);
			}

			s32Ret = RK_MPI_VENC_ReleaseStream(0, &stFrame);
			if (s32Ret != RK_SUCCESS) {
				RK_LOGE("RK_MPI_VENC_ReleaseStream fail %x", s32Ret);
			}
			loopCount++;
		}

		if ((g_s32FrameCnt >= 0) && (loopCount >= g_s32FrameCnt)) {
			quit = true;
			break;
		}
		/* Removed artificial usleep(10*1000) to ensure zero-latency 60fps delivery */
	}

	printf("======exit %s=======\n", __func__);
	free(stFrame.pstPack);
	return NULL;
}

static RK_S32 test_venc_init(int chnId, int width, int height, RK_CODEC_ID_E enType) {
	printf("========%s: Initializing %s @ %dx%d (%u Kbps, 60fps)========\n",
	       __func__, (enType == RK_VIDEO_ID_HEVC ? "H.265" : "H.264"),
	       width, height, g_u32Bitrate);

	VENC_RECV_PIC_PARAM_S stRecvParam;
	VENC_CHN_ATTR_S stAttr;
	memset(&stAttr, 0, sizeof(VENC_CHN_ATTR_S));

	/* 60 FPS Rate Control & Bitrate */
	if (enType == RK_VIDEO_ID_HEVC) {
		stAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
		stAttr.stRcAttr.stH265Cbr.u32BitRate = g_u32Bitrate;
		stAttr.stRcAttr.stH265Cbr.u32Gop = 60;
		stAttr.stRcAttr.stH265Cbr.u32SrcFrameRateNum = 60;
		stAttr.stRcAttr.stH265Cbr.u32SrcFrameRateDen = 1;
		stAttr.stRcAttr.stH265Cbr.fr32DstFrameRateNum = 60;
		stAttr.stRcAttr.stH265Cbr.fr32DstFrameRateDen = 1;
	} else if (enType == RK_VIDEO_ID_AVC) {
		stAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
		stAttr.stRcAttr.stH264Cbr.u32BitRate = g_u32Bitrate;
		stAttr.stRcAttr.stH264Cbr.u32Gop = 60;
		stAttr.stRcAttr.stH264Cbr.u32SrcFrameRateNum = 60;
		stAttr.stRcAttr.stH264Cbr.u32SrcFrameRateDen = 1;
		stAttr.stRcAttr.stH264Cbr.fr32DstFrameRateNum = 60;
		stAttr.stRcAttr.stH264Cbr.fr32DstFrameRateDen = 1;
	}

	stAttr.stVencAttr.enType = enType;
	stAttr.stVencAttr.enPixelFormat = RK_FMT_YUV420SP;
	if (enType == RK_VIDEO_ID_AVC)
		stAttr.stVencAttr.u32Profile = H264E_PROFILE_HIGH;
	else if (enType == RK_VIDEO_ID_HEVC)
		stAttr.stVencAttr.u32Profile = H265E_PROFILE_MAIN;

	stAttr.stVencAttr.u32PicWidth = width;
	stAttr.stVencAttr.u32PicHeight = height;
	stAttr.stVencAttr.u32VirWidth = width;
	stAttr.stVencAttr.u32VirHeight = height;
	stAttr.stVencAttr.u32StreamBufCnt = 2; /* Ultra-low latency ring buffer */
	stAttr.stVencAttr.u32BufSize = width * height * 3 / 2;
	stAttr.stVencAttr.enMirror = MIRROR_NONE;

	RK_MPI_VENC_CreateChn(chnId, &stAttr);

	/* Gradual Intra Refresh (GIR / Row-by-Row Intra Refresh) */
	VENC_INTRA_REFRESH_S stIntraRefresh;
	memset(&stIntraRefresh, 0, sizeof(VENC_INTRA_REFRESH_S));
	stIntraRefresh.bRefreshEnable = RK_TRUE;
	stIntraRefresh.enIntraRefreshMode = INTRA_REFRESH_ROW;
	stIntraRefresh.u32RefreshNum = 5; /* Refreshes 5 macroblock rows per frame */
	stIntraRefresh.u32ReqIQp = 25;
	RK_MPI_VENC_SetIntraRefresh(chnId, &stIntraRefresh);

	memset(&stRecvParam, 0, sizeof(VENC_RECV_PIC_PARAM_S));
	stRecvParam.s32RecvPicNum = -1;
	RK_MPI_VENC_StartRecvFrame(chnId, &stRecvParam);

	return 0;
}

int vi_dev_init() {
	int ret = 0;
	int devId = 0;
	int pipeId = devId;

	VI_DEV_ATTR_S stDevAttr;
	VI_DEV_BIND_PIPE_S stBindPipe;
	memset(&stDevAttr, 0, sizeof(stDevAttr));
	memset(&stBindPipe, 0, sizeof(stBindPipe));

	ret = RK_MPI_VI_GetDevAttr(devId, &stDevAttr);
	if (ret == RK_ERR_VI_NOT_CONFIG) {
		ret = RK_MPI_VI_SetDevAttr(devId, &stDevAttr);
		if (ret != RK_SUCCESS) {
			printf("RK_MPI_VI_SetDevAttr %x\n", ret);
			return -1;
		}
	}

	ret = RK_MPI_VI_GetDevIsEnable(devId);
	if (ret != RK_SUCCESS) {
		ret = RK_MPI_VI_EnableDev(devId);
		if (ret != RK_SUCCESS) {
			printf("RK_MPI_VI_EnableDev %x\n", ret);
			return -1;
		}
		stBindPipe.u32Num = 1;
		stBindPipe.PipeId[0] = pipeId;
		ret = RK_MPI_VI_SetDevBindPipe(devId, &stBindPipe);
		if (ret != RK_SUCCESS) {
			printf("RK_MPI_VI_SetDevBindPipe %x\n", ret);
			return -1;
		}
	}

	return 0;
}

int vi_chn_init(int channelId, int width, int height) {
	int ret;
	int buf_cnt = 2;
	VI_CHN_ATTR_S vi_chn_attr;
	memset(&vi_chn_attr, 0, sizeof(vi_chn_attr));
	vi_chn_attr.stIspOpt.u32BufCount = buf_cnt;
	vi_chn_attr.stIspOpt.enMemoryType = VI_V4L2_MEMORY_TYPE_DMABUF;
	vi_chn_attr.stSize.u32Width = width;
	vi_chn_attr.stSize.u32Height = height;
	vi_chn_attr.enPixelFormat = RK_FMT_YUV420SP;
	vi_chn_attr.enCompressMode = COMPRESS_MODE_NONE;
	vi_chn_attr.u32Depth = 0;

	/* Set 60 FPS on VI Ingest */
	vi_chn_attr.stFrameRate.s32SrcFrameRate = 60;
	vi_chn_attr.stFrameRate.s32DstFrameRate = 60;

	ret = RK_MPI_VI_SetChnAttr(0, channelId, &vi_chn_attr);
	ret |= RK_MPI_VI_EnableChn(0, channelId);
	if (ret) {
		printf("ERROR: create VI error! ret=%d\n", ret);
		return ret;
	}

	return ret;
}

static XCamReturn SIMPLE_COMM_ISP_SofCb(rk_aiq_metas_t *meta) {
	g_sof_cnt++;
	return XCAM_RETURN_NO_ERROR;
}

static XCamReturn SIMPLE_COMM_ISP_ErrCb(rk_aiq_err_msg_t *msg) {
	if (msg->err_code == XCAM_RETURN_BYPASS)
		g_should_quit = true;
	return XCAM_RETURN_NO_ERROR;
}

RK_S32 SIMPLE_COMM_ISP_Init(RK_S32 CamId, rk_aiq_working_mode_t WDRMode, RK_BOOL MultiCam,
                            const char *iq_file_dir) {
	if (CamId >= MAX_AIQ_CTX) return -1;
	setlinebuf(stdout);
	if (iq_file_dir == NULL) {
		g_aiq_ctx[CamId] = NULL;
		return 0;
	}

	g_WDRMode[CamId] = WDRMode;
	char hdr_str[16];
	snprintf(hdr_str, sizeof(hdr_str), "%d", (int)WDRMode);
	setenv("HDR_MODE", hdr_str, 1);

	rk_aiq_sys_ctx_t *aiq_ctx;
	rk_aiq_static_info_t aiq_static_info;

#ifdef RV1126_RV1109
	rk_aiq_uapi_sysctl_enumStaticMetas(CamId, &aiq_static_info);
	aiq_ctx = rk_aiq_uapi_sysctl_init(aiq_static_info.sensor_info.sensor_name, iq_file_dir,
	                                 SIMPLE_COMM_ISP_ErrCb, SIMPLE_COMM_ISP_SofCb);
	if (MultiCam) rk_aiq_uapi_sysctl_setMulCamConc(aiq_ctx, true);
#else
	rk_aiq_uapi2_sysctl_enumStaticMetas(CamId, &aiq_static_info);
	aiq_ctx = rk_aiq_uapi2_sysctl_init(aiq_static_info.sensor_info.sensor_name, iq_file_dir,
	                                  SIMPLE_COMM_ISP_ErrCb, SIMPLE_COMM_ISP_SofCb);
	if (MultiCam) rk_aiq_uapi2_sysctl_setMulCamConc(aiq_ctx, true);
#endif

	g_aiq_ctx[CamId] = aiq_ctx;
	return 0;
}

RK_S32 SIMPLE_COMM_ISP_Run(RK_S32 CamId) {
	if (CamId >= MAX_AIQ_CTX || !g_aiq_ctx[CamId]) return -1;

#ifdef RV1126_RV1109
	if (rk_aiq_uapi_sysctl_prepare(g_aiq_ctx[CamId], 0, 0, g_WDRMode[CamId])) {
		g_aiq_ctx[CamId] = NULL;
		return -1;
	}
	if (rk_aiq_uapi_sysctl_start(g_aiq_ctx[CamId])) return -1;
#else
	if (rk_aiq_uapi2_sysctl_prepare(g_aiq_ctx[CamId], 0, 0, g_WDRMode[CamId])) {
		g_aiq_ctx[CamId] = NULL;
		return -1;
	}
	if (rk_aiq_uapi2_sysctl_start(g_aiq_ctx[CamId])) return -1;
#endif
	return 0;
}

RK_S32 SIMPLE_COMM_ISP_Stop(RK_S32 CamId) {
	if (CamId >= MAX_AIQ_CTX || !g_aiq_ctx[CamId]) return -1;

#ifdef RV1126_RV1109
	rk_aiq_uapi_sysctl_stop(g_aiq_ctx[CamId], false);
	rk_aiq_uapi_sysctl_deinit(g_aiq_ctx[CamId]);
#else
	rk_aiq_uapi2_sysctl_stop(g_aiq_ctx[CamId], false);
	rk_aiq_uapi2_sysctl_deinit(g_aiq_ctx[CamId]);
#endif

	g_aiq_ctx[CamId] = NULL;
	return 0;
}

static RK_CHAR optstr[] = "?::a::w:h:c:I:e:b:";
static void print_usage(const RK_CHAR *name) {
	printf("Usage:\n");
	printf("\t%s -w 1280 -h 720 -e h265 -b 2400 (Stream at rtsp://<ip>/live/0)\n", name);
	printf("\t-w | --width: Output width (Default: 1280)\n");
	printf("\t-h | --height: Output height (Default: 720)\n");
	printf("\t-a | --aiq: IQ file path (Default: /etc/iqfiles)\n");
	printf("\t-e | --encode: Codec (Default: h265, options: h264, h265)\n");
	printf("\t-b | --bitrate: Bitrate in Kbps (Default: 2400)\n");
}

int main(int argc, char *argv[]) {
	RK_S32 s32Ret = RK_FAILURE;
	/* Defaults: 720p, H.265 (HEVC), 2.4 Mbps, Camera Channel 0 */
	RK_U32 u32Width = 1280;
	RK_U32 u32Height = 720;
	RK_CODEC_ID_E enCodecType = RK_VIDEO_ID_HEVC;
	RK_CHAR *pCodecName = "H265";
	RK_S32 s32chnlId = 0;
	char *iq_dir = "/etc/iqfiles";
	int c;
	int ret = -1;

	while ((c = getopt(argc, argv, optstr)) != -1) {
		switch (c) {
		case 'a':
			if (optarg) iq_dir = optarg;
			break;
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
			g_s32FrameCnt = atoi(optarg);
			break;
		case 'e':
			if (!strcmp(optarg, "h264")) {
				enCodecType = RK_VIDEO_ID_AVC;
				pCodecName = "H264";
			} else if (!strcmp(optarg, "h265")) {
				enCodecType = RK_VIDEO_ID_HEVC;
				pCodecName = "H265";
			}
			break;
		case 'b':
			g_u32Bitrate = atoi(optarg);
			break;
		case '?':
		default:
			print_usage(argv[0]);
			return -1;
		}
	}

	printf("\n========================================\n");
	printf(" Stream: RTSP Server @ rtsp://<board_ip>/live/0\n");
	printf(" Codec: %s\n", pCodecName);
	printf(" Resolution: %dx%d (Scaled from 1080p Full FOV)\n", u32Width, u32Height);
	printf(" Framerate: 60 FPS\n");
	printf(" Bitrate: %u Kbps CBR (Gradual Intra Refresh Enabled)\n", g_u32Bitrate);
	printf("========================================\n\n");

	signal(SIGINT, sigterm_handler);

	if (iq_dir) {
#ifdef RKAIQ
		printf("Loading ISP IQ files from %s\n", iq_dir);
		SIMPLE_COMM_ISP_Init(0, RK_AIQ_WORKING_MODE_NORMAL, 0, iq_dir);
		SIMPLE_COMM_ISP_Run(0);
#endif
	}

	/* Initialize RTSP Session */
	g_rtsplive = create_rtsp_demo(554);
	g_rtsp_session = rtsp_new_session(g_rtsplive, "/live/0");
	if (enCodecType == RK_VIDEO_ID_AVC) {
		rtsp_set_video(g_rtsp_session, RTSP_CODEC_ID_VIDEO_H264, NULL, 0);
	} else if (enCodecType == RK_VIDEO_ID_HEVC) {
		rtsp_set_video(g_rtsp_session, RTSP_CODEC_ID_VIDEO_H265, NULL, 0);
	}
	rtsp_sync_video_ts(g_rtsp_session, rtsp_get_reltime(), rtsp_get_ntptime());

	if (RK_MPI_SYS_Init() != RK_SUCCESS) {
		RK_LOGE("RK_MPI_SYS_Init failed");
		goto __FAILED;
	}

	vi_dev_init();
	vi_chn_init(s32chnlId, u32Width, u32Height);

	test_venc_init(0, u32Width, u32Height, enCodecType);

	MPP_CHN_S stSrcChn, stDestChn;
	stSrcChn.enModId = RK_ID_VI;
	stSrcChn.s32DevId = 0;
	stSrcChn.s32ChnId = s32chnlId;

	stDestChn.enModId = RK_ID_VENC;
	stDestChn.s32DevId = 0;
	stDestChn.s32ChnId = 0;

	s32Ret = RK_MPI_SYS_Bind(&stSrcChn, &stDestChn);
	if (s32Ret != RK_SUCCESS) {
		RK_LOGE("RK_MPI_SYS_Bind failed: %x", s32Ret);
		goto __FAILED;
	}

	pthread_t stream_thread;
	pthread_create(&stream_thread, NULL, GetMediaBuffer0, NULL);

	while (!quit) {
		usleep(200000);
	}

	pthread_join(stream_thread, NULL);

	if (g_rtsplive)
		rtsp_del_demo(g_rtsplive);

	RK_MPI_SYS_UnBind(&stSrcChn, &stDestChn);
	RK_MPI_VI_DisableChn(0, s32chnlId);
	RK_MPI_VENC_StopRecvFrame(0);
	RK_MPI_VENC_DestroyChn(0);
	RK_MPI_VI_DisableDev(0);
	ret = 0;

__FAILED:
	RK_MPI_SYS_Exit();
#ifdef RKAIQ
	SIMPLE_COMM_ISP_Stop(0);
#endif
	return ret;
}