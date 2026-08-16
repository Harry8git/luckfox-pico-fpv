/*
 * Dedicated Low-Latency H.265 720p60 USB CDC Streamer (VTX)
 * Hardware Pipeline: Sony IMX462 (1080p60) -> VI (720p Scaler) -> VENC (H.265 CBR+GIR) -> USB CDC (/dev/ttyGS0)
 */

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

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
#include "rk_mpi_mb.h"
#include "rk_mpi_sys.h"
#include "rk_mpi_venc.h"
#include "rk_mpi_vi.h"

#define VTX_WIDTH          1280
#define VTX_HEIGHT         720
#define VTX_FPS            60
#define VTX_BITRATE_KBPS   2400        /* 2.4 Mbps CBR */
#define VTX_CDC_DEV        "/dev/ttyGS0"
#define VTX_IQ_DIR         "/etc/iqfiles"

static rk_aiq_sys_ctx_t *g_aiq_ctx = NULL;
static bool quit = false;

static void sigterm_handler(int sig) {
	fprintf(stderr, "\nCaught signal %d, stopping VTX streamer...\n", sig);
	quit = true;
}

/* ----------------------------------------------------------------------------
 * USB CDC Video Streaming Thread
 * ---------------------------------------------------------------------------- */
static void *venc_cdc_stream_thread(void *arg) {
	(void)arg;
	VENC_STREAM_S stFrame;
	stFrame.pstPack = malloc(sizeof(VENC_PACK_S));
	if (!stFrame.pstPack) {
		RK_LOGE("Failed to allocate VENC_PACK_S memory");
		quit = true;
		return NULL;
	}

	int cdc_fd = open(VTX_CDC_DEV, O_WRONLY | O_NONBLOCK | O_NOCTTY);
	if (cdc_fd < 0) {
		fprintf(stderr, "ERROR: Could not open %s (%s). Check USB connection.\n",
		        VTX_CDC_DEV, strerror(errno));
		quit = true;
		free(stFrame.pstPack);
		return NULL;
	}

	printf("\n>>> VTX STREAMING ACTIVE: Outputting H.265 to %s <<<\n\n", VTX_CDC_DEV);

	while (!quit) {
		/* Direct blocking wait (200ms timeout) */
		RK_S32 s32Ret = RK_MPI_VENC_GetStream(0, &stFrame, 200);
		if (s32Ret == RK_SUCCESS) {
			void *pData = RK_MPI_MB_Handle2VirAddr(stFrame.pstPack->pMbBlk);
			uint8_t *packet_buf = (uint8_t *)pData + stFrame.pstPack->u32Offset;
			uint32_t packet_len = stFrame.pstPack->u32Len - stFrame.pstPack->u32Offset;

			if (packet_buf && packet_len > 0) {
				ssize_t written = write(cdc_fd, packet_buf, packet_len);
				if (written < 0 && (errno != EAGAIN && errno != EWOULDBLOCK)) {
					RK_LOGD("USB CDC write issue: %s", strerror(errno));
				}
			}
			RK_MPI_VENC_ReleaseStream(0, &stFrame);
		}
	}

	if (cdc_fd >= 0) {
		close(cdc_fd);
		printf("Closed USB CDC device node\n");
	}

	free(stFrame.pstPack);
	return NULL;
}

/* ----------------------------------------------------------------------------
 * VI / VENC / ISP Hardware Configuration
 * ---------------------------------------------------------------------------- */
static int vi_init(int channelId, int width, int height) {
	VI_DEV_ATTR_S stDevAttr;
	VI_DEV_BIND_PIPE_S stBindPipe;
	memset(&stDevAttr, 0, sizeof(stDevAttr));
	memset(&stBindPipe, 0, sizeof(stBindPipe));

	RK_MPI_VI_GetDevAttr(0, &stDevAttr);
	RK_MPI_VI_SetDevAttr(0, &stDevAttr);
	RK_MPI_VI_EnableDev(0);

	stBindPipe.u32Num = 1;
	stBindPipe.PipeId[0] = 0;
	RK_MPI_VI_SetDevBindPipe(0, &stBindPipe);

	VI_CHN_ATTR_S vi_chn_attr;
	memset(&vi_chn_attr, 0, sizeof(vi_chn_attr));
	vi_chn_attr.stIspOpt.u32BufCount = 2;
	vi_chn_attr.stIspOpt.enMemoryType = VI_V4L2_MEMORY_TYPE_DMABUF;
	vi_chn_attr.stSize.u32Width = width;   /* 1280 (Downscaled by ISP) */
	vi_chn_attr.stSize.u32Height = height; /* 720 */
	vi_chn_attr.enPixelFormat = RK_FMT_YUV420SP;
	vi_chn_attr.enCompressMode = COMPRESS_MODE_NONE;
	vi_chn_attr.u32Depth = 0;

	/* Fixed 60 FPS */
	vi_chn_attr.stFrameRate.s32SrcFrameRate = VTX_FPS;
	vi_chn_attr.stFrameRate.s32DstFrameRate = VTX_FPS;

	RK_S32 ret = RK_MPI_VI_SetChnAttr(0, channelId, &vi_chn_attr);
	ret |= RK_MPI_VI_EnableChn(0, channelId);
	return ret;
}

static int venc_init(int chnId, int width, int height) {
	VENC_RECV_PIC_PARAM_S stRecvParam;
	VENC_CHN_ATTR_S stAttr;
	memset(&stAttr, 0, sizeof(VENC_CHN_ATTR_S));

	/* H.265 CBR Rate Control */
	stAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
	stAttr.stRcAttr.stH265Cbr.u32BitRate = VTX_BITRATE_KBPS;
	stAttr.stRcAttr.stH265Cbr.u32Gop = VTX_FPS;
	stAttr.stRcAttr.stH265Cbr.u32SrcFrameRateNum = VTX_FPS;
	stAttr.stRcAttr.stH265Cbr.u32SrcFrameRateDen = 1;
	stAttr.stRcAttr.stH265Cbr.fr32DstFrameRateNum = VTX_FPS;
	stAttr.stRcAttr.stH265Cbr.fr32DstFrameRateDen = 1;

	stAttr.stVencAttr.enType = RK_VIDEO_ID_HEVC;
	stAttr.stVencAttr.enPixelFormat = RK_FMT_YUV420SP;
	stAttr.stVencAttr.u32Profile = H265E_PROFILE_MAIN;
	stAttr.stVencAttr.u32PicWidth = width;
	stAttr.stVencAttr.u32PicHeight = height;
	stAttr.stVencAttr.u32VirWidth = width;
	stAttr.stVencAttr.u32VirHeight = height;
	stAttr.stVencAttr.u32StreamBufCnt = 2; /* Low latency buffer pool */
	stAttr.stVencAttr.u32BufSize = width * height * 3 / 2;
	stAttr.stVencAttr.enMirror = MIRROR_NONE;

	RK_MPI_VENC_CreateChn(chnId, &stAttr);

	/* 1. Motion Deblur: Keeps edges sharp during high-speed drone turns */
	RK_MPI_VENC_EnableMotionDeblur(chnId, RK_TRUE);

	/* 2. Flat Scaling List: Reduces DCT latency & decoding complexity */
	VENC_H265_TRANS_S pstH265Trans;
	RK_MPI_VENC_GetH265Trans(chnId, &pstH265Trans);
	pstH265Trans.bScalingListEnabled = 0;
	RK_MPI_VENC_SetH265Trans(chnId, &pstH265Trans);

	/* 3. Gradual Intra Refresh (GIR): Flat bit distribution across frames */
	VENC_INTRA_REFRESH_S stIntraRefresh;
	memset(&stIntraRefresh, 0, sizeof(VENC_INTRA_REFRESH_S));
	stIntraRefresh.bRefreshEnable = RK_TRUE;
	stIntraRefresh.enIntraRefreshMode = INTRA_REFRESH_ROW;
	stIntraRefresh.u32RefreshNum = height / VTX_FPS; /* 12 rows per frame */
	stIntraRefresh.u32ReqIQp = 25;
	RK_MPI_VENC_SetIntraRefresh(chnId, &stIntraRefresh);

	/* 4. Strict QP Smoothing: Prevents sudden pixelation when panning */
	VENC_RC_PARAM_S stRcParam;
	memset(&stRcParam, 0, sizeof(VENC_RC_PARAM_S));
	stRcParam.s32FirstFrameStartQp = 26;
	stRcParam.stParamH265.u32StepQp = 3;
	stRcParam.stParamH265.u32MinQp = 18;
	stRcParam.stParamH265.u32MaxQp = 40;
	stRcParam.stParamH265.u32MinIQp = 18;
	stRcParam.stParamH265.u32MaxIQp = 36;
	RK_MPI_VENC_SetRcParam(chnId, &stRcParam);

	memset(&stRecvParam, 0, sizeof(VENC_RECV_PIC_PARAM_S));
	stRecvParam.s32RecvPicNum = -1;
	RK_MPI_VENC_StartRecvFrame(chnId, &stRecvParam);

	return 0;
}

/* ----------------------------------------------------------------------------
 * Main Entry Point
 * ---------------------------------------------------------------------------- */
int main(int argc, char *argv[]) {
	(void)argc;
	(void)argv;

	signal(SIGINT, sigterm_handler);
	signal(SIGTERM, sigterm_handler);

	printf("\n=======================================================\n");
	printf(" Luckfox Pico Zero - Dedicated H.265 Low-Latency VTX\n");
	printf(" Ingest       : Sony IMX462 1080p60 (Full FOV)\n");
	printf(" Scaler/Output: 1280x720 @ 60 FPS (Hardware ISP)\n");
	printf(" Video Codec  : H.265 Main Profile\n");
	printf(" Bitrate      : %u Kbps CBR (Gradual Refresh Active)\n", VTX_BITRATE_KBPS);
	printf(" Destination  : %s\n", VTX_CDC_DEV);
	printf("=======================================================\n\n");

	/* Initialize RKAIQ ISP unconditionally from /etc/iqfiles */
	rk_aiq_static_info_t aiq_static_info;
	rk_aiq_uapi2_sysctl_enumStaticMetas(0, &aiq_static_info);
	g_aiq_ctx = rk_aiq_uapi2_sysctl_init(aiq_static_info.sensor_info.sensor_name,
	                                    VTX_IQ_DIR, NULL, NULL);
	if (g_aiq_ctx) {
		rk_aiq_uapi2_sysctl_prepare(g_aiq_ctx, 0, 0, RK_AIQ_WORKING_MODE_NORMAL);
		rk_aiq_uapi2_sysctl_start(g_aiq_ctx);
	}

	if (RK_MPI_SYS_Init() != RK_SUCCESS) {
		RK_LOGE("RK_MPI_SYS_Init failed");
		return -1;
	}

	/* Initialize Hardware VI (ISP downscales 1080p -> 720p) & VENC (H.265) */
	vi_init(0, VTX_WIDTH, VTX_HEIGHT);
	venc_init(0, VTX_WIDTH, VTX_HEIGHT);

	/* Direct VI[0] -> VENC[0] Binding */
	MPP_CHN_S stSrcChn = { .enModId = RK_ID_VI,   .s32DevId = 0, .s32ChnId = 0 };
	MPP_CHN_S stDestChn = { .enModId = RK_ID_VENC, .s32DevId = 0, .s32ChnId = 0 };
	RK_MPI_SYS_Bind(&stSrcChn, &stDestChn);

	/* Launch USB CDC Transmission Thread */
	pthread_t cdc_thread;
	pthread_create(&cdc_thread, NULL, venc_cdc_stream_thread, NULL);

	while (!quit) {
		sleep(1);
	}

	pthread_join(cdc_thread, NULL);

	/* Clean Shutdown */
	RK_MPI_SYS_UnBind(&stSrcChn, &stDestChn);
	RK_MPI_VI_DisableChn(0, 0);
	RK_MPI_VENC_StopRecvFrame(0);
	RK_MPI_VENC_DestroyChn(0);
	RK_MPI_VI_DisableDev(0);
	RK_MPI_SYS_Exit();

	if (g_aiq_ctx) {
		rk_aiq_uapi2_sysctl_stop(g_aiq_ctx, false);
		rk_aiq_uapi2_sysctl_deinit(g_aiq_ctx);
	}

	printf("VTX Pipeline cleanly terminated.\n");
	return 0;
}