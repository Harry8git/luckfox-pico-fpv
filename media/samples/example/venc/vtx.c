/*
 * Dedicated Low-Latency H.265 720p60 USB CDC Streamer (VTX)
 * Pipeline: Sony IMX462 (1080p60) -> Direct VI (720p Hardware Scaler) -> VENC (H.265 CBR+GIR) -> USB CDC (/dev/ttyGS0)
 */

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/prctl.h>
#include <time.h>
#include <unistd.h>

#include "rk_debug.h"
#include "rk_defines.h"
#include "rk_mpi_mb.h"
#include "rk_mpi_sys.h"
#include "rk_mpi_venc.h"
#include "rk_mpi_vi.h"
#include "sample_comm.h"

#define VTX_WIDTH          1280
#define VTX_HEIGHT         720
#define VTX_FPS            60
#define VTX_BITRATE_KBPS   2400        /* 2.4 Mbps CBR */
#define VTX_CDC_DEV        "/dev/ttyGS0"
#define VTX_IQ_DIR         "/etc/iqfiles"
#define MAX_PACK_COUNT     8

static volatile bool quit = false;

static void sigterm_handler(int sig) {
	fprintf(stderr, "\nSignal %d received, stopping VTX...\n", sig);
	quit = true;
}

/* Fast, non-blocking-polling write to USB CDC */
static inline int cdc_write_all(int fd, const uint8_t *buf, size_t len) {
	size_t total_written = 0;
	struct pollfd pfd = { .fd = fd, .events = POLLOUT };

	while (total_written < len && !quit) {
		int ret = poll(&pfd, 1, 20); /* 20ms poll timeout */
		if (ret < 0) {
			if (errno == EINTR) continue;
			return -1;
		}
		if (ret == 0) continue;

		ssize_t n = write(fd, buf + total_written, len - total_written);
		if (n < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
			return -1;
		}
		total_written += (size_t)n;
	}
	return (int)total_written;
}

/* ----------------------------------------------------------------------------
 * Real-Time Video Streaming Thread
 * ---------------------------------------------------------------------------- */
static void *venc_cdc_stream_thread(void *arg) {
	(void)arg;
	prctl(PR_SET_NAME, "vtx_cdc_tx", 0, 0, 0);

	/* Elevate thread to real-time priority (SCHED_FIFO) */
	struct sched_param param;
	param.sched_priority = 80;
	pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

	VENC_STREAM_S stFrame;
	memset(&stFrame, 0, sizeof(VENC_STREAM_S));
	stFrame.pstPack = malloc(sizeof(VENC_PACK_S) * MAX_PACK_COUNT);
	if (!stFrame.pstPack) {
		RK_LOGE("Failed to allocate VENC_PACK_S");
		quit = true;
		return NULL;
	}

	int cdc_fd = open(VTX_CDC_DEV, O_WRONLY | O_NONBLOCK | O_NOCTTY);
	if (cdc_fd < 0) {
		fprintf(stderr, "ERROR: Could not open %s (%s)\n", VTX_CDC_DEV, strerror(errno));
		quit = true;
		free(stFrame.pstPack);
		return NULL;
	}

	printf(">>> VTX ACTIVE: Streaming 720p60 H.265 to %s <<<\n", VTX_CDC_DEV);

	while (!quit) {
		stFrame.u32PackCount = MAX_PACK_COUNT;

		/* Hardware interrupt wait: lowest possible latency without CPU busy-wait */
		RK_S32 s32Ret = RK_MPI_VENC_GetStream(0, &stFrame, 100);
		if (s32Ret == RK_SUCCESS) {
			for (RK_U32 i = 0; i < stFrame.u32PackCount; i++) {
				void *pData = RK_MPI_MB_Handle2VirAddr(stFrame.pstPack[i].pMbBlk);
				uint8_t *buf = (uint8_t *)pData + stFrame.pstPack[i].u32Offset;
				uint32_t len = stFrame.pstPack[i].u32Len - stFrame.pstPack[i].u32Offset;

				if (buf && len > 0) {
					cdc_write_all(cdc_fd, buf, len);
				}
			}
			RK_MPI_VENC_ReleaseStream(0, &stFrame);
		}
	}

	close(cdc_fd);
	free(stFrame.pstPack);
	return NULL;
}

/* ----------------------------------------------------------------------------
 * VI / VENC Hardware Setup
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
	vi_chn_attr.stIspOpt.u32BufCount = 2; /* 2-buffer pool for minimum latency */
	vi_chn_attr.stIspOpt.enMemoryType = VI_V4L2_MEMORY_TYPE_DMABUF;
	vi_chn_attr.stSize.u32Width = width;   /* 1280x720 scaled directly in ISP */
	vi_chn_attr.stSize.u32Height = height;
	vi_chn_attr.enPixelFormat = RK_FMT_YUV420SP;
	vi_chn_attr.enCompressMode = COMPRESS_MODE_NONE;
	vi_chn_attr.u32Depth = 0;

	/* -1 disables software rate throttling, locking to native sensor clock */
	vi_chn_attr.stFrameRate.s32SrcFrameRate = -1;
	vi_chn_attr.stFrameRate.s32DstFrameRate = -1;

	RK_S32 ret = RK_MPI_VI_SetChnAttr(0, channelId, &vi_chn_attr);
	ret |= RK_MPI_VI_EnableChn(0, channelId);
	return ret;
}

static int venc_init(int chnId, int width, int height) {
	VENC_CHN_ATTR_S stAttr;
	memset(&stAttr, 0, sizeof(VENC_CHN_ATTR_S));

	/* H.265 CBR Rate Control */
	stAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
	stAttr.stRcAttr.stH265Cbr.u32BitRate = VTX_BITRATE_KBPS;
	stAttr.stRcAttr.stH265Cbr.u32Gop = VTX_FPS * 2;
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
	stAttr.stVencAttr.u32StreamBufCnt = 2;
	stAttr.stVencAttr.u32BufSize = width * height * 3 / 2;
	stAttr.stVencAttr.enMirror = MIRROR_NONE;

	/* GOP Normal-P Mode */
	stAttr.stGopAttr.enGopMode = VENC_GOPMODE_NORMALP;

	RK_MPI_VENC_CreateChn(chnId, &stAttr);

	/* 1. Motion Deblur */
	RK_MPI_VENC_EnableMotionDeblur(chnId, RK_TRUE);

	/* 2. Flat Scaling List (reduces transform latency) */
	VENC_H265_TRANS_S pstH265Trans;
	RK_MPI_VENC_GetH265Trans(chnId, &pstH265Trans);
	pstH265Trans.bScalingListEnabled = 0;
	RK_MPI_VENC_SetH265Trans(chnId, &pstH265Trans);

	/* 3. Gradual Intra Refresh: 1 CTU row/frame for flat bit distribution */
	VENC_INTRA_REFRESH_S stIntraRefresh;
	memset(&stIntraRefresh, 0, sizeof(VENC_INTRA_REFRESH_S));
	stIntraRefresh.bRefreshEnable = RK_TRUE;
	stIntraRefresh.enIntraRefreshMode = INTRA_REFRESH_ROW;
	stIntraRefresh.u32RefreshNum = 1;
	stIntraRefresh.u32ReqIQp = 25;
	RK_MPI_VENC_SetIntraRefresh(chnId, &stIntraRefresh);

	/* 4. Strict QP Range */
	VENC_RC_PARAM_S stRcParam;
	memset(&stRcParam, 0, sizeof(VENC_RC_PARAM_S));
	stRcParam.s32FirstFrameStartQp = 26;
	stRcParam.stParamH265.u32StepQp = 3;
	stRcParam.stParamH265.u32MinQp = 18;
	stRcParam.stParamH265.u32MaxQp = 40;
	stRcParam.stParamH265.u32MinIQp = 18;
	stRcParam.stParamH265.u32MaxIQp = 36;
	RK_MPI_VENC_SetRcParam(chnId, &stRcParam);

	VENC_RECV_PIC_PARAM_S stRecvParam;
	memset(&stRecvParam, 0, sizeof(VENC_RECV_PIC_PARAM_S));
	stRecvParam.s32RecvPicNum = -1;
	RK_MPI_VENC_StartRecvFrame(chnId, &stRecvParam);

	return 0;
}

/* ----------------------------------------------------------------------------
 * Main Function
 * ---------------------------------------------------------------------------- */
int main(int argc, char *argv[]) {
	(void)argc;
	(void)argv;

	signal(SIGINT, sigterm_handler);
	signal(SIGTERM, sigterm_handler);

	printf("\n=======================================================\n");
	printf(" Luckfox Pico - Low-Latency Direct H.265 VTX\n");
	printf(" Ingest       : Sony IMX462 (ISP 720p60 Direct Scaling)\n");
	printf(" Video Codec  : H.265 CBR (%u Kbps) + GIR Active\n", VTX_BITRATE_KBPS);
	printf(" Target Node  : %s\n", VTX_CDC_DEV);
	printf("=======================================================\n\n");

	/* 1. Official ISP initialization using sample_comm */
	SAMPLE_COMM_ISP_Init(0, RK_AIQ_WORKING_MODE_NORMAL, RK_FALSE, VTX_IQ_DIR);
	SAMPLE_COMM_ISP_Run(0);

	/* 2. System Init */
	if (RK_MPI_SYS_Init() != RK_SUCCESS) {
		RK_LOGE("RK_MPI_SYS_Init failed");
		return -1;
	}

	/* 3. Initialize VI and VENC */
	vi_init(0, VTX_WIDTH, VTX_HEIGHT);
	venc_init(0, VTX_WIDTH, VTX_HEIGHT);

	/* 4. Direct VI -> VENC Zero-Copy Binding */
	MPP_CHN_S stSrcChn  = { .enModId = RK_ID_VI,   .s32DevId = 0, .s32ChnId = 0 };
	MPP_CHN_S stDestChn = { .enModId = RK_ID_VENC, .s32DevId = 0, .s32ChnId = 0 };
	RK_MPI_SYS_Bind(&stSrcChn, &stDestChn);

	/* 5. Start Transmission Thread */
	pthread_t cdc_thread;
	pthread_create(&cdc_thread, NULL, venc_cdc_stream_thread, NULL);

	while (!quit) {
		sleep(1);
	}

	pthread_join(cdc_thread, NULL);

	/* Clean Teardown */
	RK_MPI_SYS_UnBind(&stSrcChn, &stDestChn);
	RK_MPI_VI_DisableChn(0, 0);
	RK_MPI_VENC_StopRecvFrame(0);
	RK_MPI_VENC_DestroyChn(0);
	RK_MPI_VI_DisableDev(0);
	RK_MPI_SYS_Exit();

	SAMPLE_COMM_ISP_Stop(0);

	printf("VTX cleanly terminated.\n");
	return 0;
}