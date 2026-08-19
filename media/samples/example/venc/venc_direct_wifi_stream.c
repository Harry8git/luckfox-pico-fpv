/*
 * Dedicated Low-Latency H.265 720p60 WiFi RTSP Streamer with Gradual Intra Refresh (GIR)
 * Architecture: Sony IMX462 (1080p60) -> VI (720p Scaler) -> VENC (H.265 CBR + GIR) -> RTSP (/live/0)
 */

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/prctl.h>
#include <time.h>
#include <unistd.h>

#include "rtsp_demo.h"
#include "sample_comm.h"

#define VTX_WIDTH          1280
#define VTX_HEIGHT         720
#define VTX_FPS            60
#define VTX_BITRATE_KBPS   2500        /* 2.5 Mbps */
#define VTX_RTSP_PORT      554
#define VTX_IQ_DIR         "/etc/iqfiles"

rtsp_demo_handle g_rtsplive = NULL;
static rtsp_session_handle g_rtsp_session = NULL;

typedef struct _rkMpiCtx {
	SAMPLE_VI_CTX_S vi;
	SAMPLE_VENC_CTX_S venc;
} SAMPLE_MPI_CTX_S;

static bool quit = false;
static void sigterm_handler(int sig) {
	fprintf(stderr, "\nSignal %d received, stopping RTSP server...\n", sig);
	quit = true;
}

/******************************************************************************
 * function : VENC Stream Thread Callback
 ******************************************************************************/
static void *venc_get_stream(void *pArgs) {
	SAMPLE_VENC_CTX_S *ctx = (SAMPLE_VENC_CTX_S *)(pArgs);
	RK_S32 s32Ret = RK_FAILURE;
	void *pData = RK_NULL;

	prctl(PR_SET_NAME, "vtx_rtsp_tx", 0, 0, 0);

	printf("\n=======================================================\n");
	printf(" >>> RTSP SERVER ACTIVE: rtsp://<LUCKFOX_IP>:554/live/0 <<<\n");
	printf("=======================================================\n\n");

	while (!quit) {
		s32Ret = SAMPLE_COMM_VENC_GetStream(ctx, &pData);
		if (s32Ret == RK_SUCCESS) {
			if (g_rtsp_session && pData) {
				rtsp_tx_video(g_rtsp_session, pData, ctx->stFrame.pstPack->u32Len,
				              ctx->stFrame.pstPack->u64PTS);
				rtsp_do_event(g_rtsplive);
			}
			SAMPLE_COMM_VENC_ReleaseStream(ctx);
		} else {
			rtsp_do_event(g_rtsplive);
		}
		usleep(1000);
	}

	return RK_NULL;
}

/******************************************************************************
 * function : main()
 ******************************************************************************/
int main(int argc, char *argv[]) {
	(void)argc;
	(void)argv;

	SAMPLE_MPI_CTX_S *ctx;
	MPP_CHN_S stSrcChn, stDestChn;

	signal(SIGINT, sigterm_handler);
	signal(SIGTERM, sigterm_handler);

	ctx = (SAMPLE_MPI_CTX_S *)malloc(sizeof(SAMPLE_MPI_CTX_S));
	if (!ctx) {
		RK_LOGE("Failed to allocate SAMPLE_MPI_CTX_S");
		return -1;
	}
	memset(ctx, 0, sizeof(SAMPLE_MPI_CTX_S));

	printf("\n=======================================================\n");
	printf(" Luckfox Pico - Low-Latency H.265 RTSP VTX (GIR Active)\n");
	printf(" Ingest       : Sony IMX462 (ISP 720p60 Scaling)\n");
	printf(" Video Codec  : H.265 CBR (%u Kbps), 60 FPS\n", VTX_BITRATE_KBPS);
	printf(" Stream URL   : rtsp://<LUCKFOX_IP>:554/live/0\n");
	printf("=======================================================\n\n");

	/* 1. Initialize RTSP Demo Server */
	g_rtsplive = create_rtsp_demo(VTX_RTSP_PORT);
	g_rtsp_session = rtsp_new_session(g_rtsplive, "/live/0");
	rtsp_set_video(g_rtsp_session, RTSP_CODEC_ID_VIDEO_H265, NULL, 0);
	rtsp_sync_video_ts(g_rtsp_session, rtsp_get_reltime(), rtsp_get_ntptime());

	/* 2. Initialize RKAIQ ISP */
	SAMPLE_COMM_ISP_Init(0, RK_AIQ_WORKING_MODE_NORMAL, RK_FALSE, VTX_IQ_DIR);
	SAMPLE_COMM_ISP_Run(0);

	/* 3. Initialize System Subsystem */
	if (RK_MPI_SYS_Init() != RK_SUCCESS) {
		RK_LOGE("RK_MPI_SYS_Init failed");
		goto __FAILED;
	}

	/* 4. Configure VI Channel 0 (Hardware Scaler 1080p -> 720p) */
	ctx->vi.u32Width = VTX_WIDTH;
	ctx->vi.u32Height = VTX_HEIGHT;
	ctx->vi.s32DevId = 0;
	ctx->vi.u32PipeId = 0;
	ctx->vi.s32ChnId = 0;
	ctx->vi.stChnAttr.stIspOpt.u32BufCount = 3;
	ctx->vi.stChnAttr.stIspOpt.enMemoryType = VI_V4L2_MEMORY_TYPE_DMABUF;
	ctx->vi.stChnAttr.u32Depth = 0;
	ctx->vi.stChnAttr.enPixelFormat = RK_FMT_YUV420SP;
	ctx->vi.stChnAttr.enCompressMode = COMPRESS_MODE_NONE;
	ctx->vi.stChnAttr.stFrameRate.s32SrcFrameRate = -1;
	ctx->vi.stChnAttr.stFrameRate.s32DstFrameRate = -1;
	SAMPLE_COMM_VI_CreateChn(&ctx->vi);

	/* 5. Configure VENC Channel 0 (H.265 CBR with Long GOP) */
	ctx->venc.s32ChnId = 0;
	ctx->venc.u32Width = VTX_WIDTH;
	ctx->venc.u32Height = VTX_HEIGHT;
	ctx->venc.u32Fps = VTX_FPS;
	
	/* Long 5-second GOP: Eliminates bandwidth spikes since GIR provides intra-refresh */
	ctx->venc.u32Gop = VTX_FPS * 5; 
	
	ctx->venc.u32BitRate = VTX_BITRATE_KBPS;
	ctx->venc.enCodecType = RK_CODEC_TYPE_H265;
	ctx->venc.enRcMode = VENC_RC_MODE_H265CBR;
	ctx->venc.getStreamCbFunc = venc_get_stream;
	ctx->venc.s32loopCount = -1;
	ctx->venc.dstFilePath = NULL;
	ctx->venc.u32BuffSize = VTX_WIDTH * VTX_HEIGHT * 3 / 2;
	ctx->venc.u32StreamBufCnt = 2;
	ctx->venc.stChnAttr.stVencAttr.u32Profile = 0; /* H.265 Main Profile */
	ctx->venc.stChnAttr.stGopAttr.enGopMode = VENC_GOPMODE_NORMALP;
	SAMPLE_COMM_VENC_CreateChn(&ctx->venc);

	/* -----------------------------------------------------------------
	 * 5b. Gradual Intra Refresh (GIR)
	 * ----------------------------------------------------------------- */
	VENC_INTRA_REFRESH_S stIntraRefresh;
	memset(&stIntraRefresh, 0, sizeof(VENC_INTRA_REFRESH_S));
	stIntraRefresh.bRefreshEnable     = RK_TRUE;
	stIntraRefresh.enIntraRefreshMode = INTRA_REFRESH_ROW;  /* Top-to-bottom refresh */
	stIntraRefresh.u32RefreshNum      = 2;                  /* 2 CTU rows/frame (~375ms full screen refresh) */
	stIntraRefresh.u32ReqIQp          = 25;                 /* Quality QP for intra blocks */
	RK_MPI_VENC_SetIntraRefresh(ctx->venc.s32ChnId, &stIntraRefresh);

	/* =================================================================
	 * [FEATURE: SLICE SPLITTING]
	 * To disable: comment out this block or set bSplitEnable = RK_FALSE
	 * Purpose: Divides large frames into <= 1350-byte slices so 1 packet
	 *          equals 1 independent NAL. Packet loss will only glitch a
	 *          single horizontal slice rather than freeze the entire screen.
	 * ================================================================= */
	VENC_SLICE_SPLIT_S stSliceSplit;
	memset(&stSliceSplit, 0, sizeof(VENC_SLICE_SPLIT_S));
	stSliceSplit.bSplitEnable = RK_TRUE;
	stSliceSplit.u32SplitSize = 1350; /* Keep individual slice <= 1350 bytes */
	RK_MPI_VENC_SetSliceSplit(ctx->venc.s32ChnId, &stSliceSplit);
	/* =================================================================
	 * [END FEATURE: SLICE SPLITTING]
	 * ================================================================= */

	/* =================================================================
	 * [FEATURE: INLINE HEADERS & FLAT TRANSFORM]
	 * To disable: comment out this block
	 * Purpose: Disables scaling list to reduce DCT transform latency and
	 *          ensures decoder parameter sets remain consistent.
	 * ================================================================= */
	VENC_H265_TRANS_S pstH265Trans;
	RK_MPI_VENC_GetH265Trans(ctx->venc.s32ChnId, &pstH265Trans);
	pstH265Trans.bScalingListEnabled = 0; /* Flat scaling list */
	RK_MPI_VENC_SetH265Trans(ctx->venc.s32ChnId, &pstH265Trans);
	/* =================================================================
	 * [END FEATURE: INLINE HEADERS & FLAT TRANSFORM]
	 * ================================================================= */

	/* 6. Direct Bind: VI[0] -> VENC[0] */
	stSrcChn.enModId = RK_ID_VI;
	stSrcChn.s32DevId = ctx->vi.s32DevId;
	stSrcChn.s32ChnId = ctx->vi.s32ChnId;

	stDestChn.enModId = RK_ID_VENC;
	stDestChn.s32DevId = 0;
	stDestChn.s32ChnId = ctx->venc.s32ChnId;

	SAMPLE_COMM_Bind(&stSrcChn, &stDestChn);

	printf("[+] Pipeline Initialized (GIR + Slice Split Active). Running...\n");

	while (!quit) {
		sleep(1);
	}

	printf("\n[+] Exiting and tearing down pipeline...\n");

	/* Clean Teardown */
	if (ctx->venc.getStreamCbFunc) {
		pthread_join(ctx->venc.getStreamThread, NULL);
	}

	SAMPLE_COMM_UnBind(&stSrcChn, &stDestChn);
	SAMPLE_COMM_VENC_DestroyChn(&ctx->venc);
	SAMPLE_COMM_VI_DestroyChn(&ctx->vi);

__FAILED:
	RK_MPI_SYS_Exit();

	if (g_rtsplive) {
		rtsp_del_demo(g_rtsplive);
	}

	SAMPLE_COMM_ISP_Stop(0);

	if (ctx) {
		free(ctx);
		ctx = NULL;
	}

	printf("[+] RTSP Pipeline cleanly terminated.\n");
	return 0;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif