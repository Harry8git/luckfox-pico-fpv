/*
 * Copyright 2021 Rockchip Electronics Co. LTD
 * Custom H.265 Hardware Encoder to USB CDC-ACM Streamer
 *usage: 
./build.sh media


 RkLunch-stop.sh
/userdata/sample_venc_cdc -a /etc/iqfiles/
/userdata/vtx -a /etc/iqfiles/
*/

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/prctl.h>
#include <time.h>
#include <unistd.h>

#include "sample_comm.h"


#include "rk_aiq_user_api2_sysctl.h"


typedef struct _rkMpiCtx {
	SAMPLE_VI_CTX_S vi;
	SAMPLE_VO_CTX_S vo;
	SAMPLE_VPSS_CTX_S vpss;
	SAMPLE_VENC_CTX_S venc;
	SAMPLE_RGN_CTX_S rgn[2];
} SAMPLE_MPI_CTX_S;

static bool quit = false;
static void sigterm_handler(int sig) {
	fprintf(stderr, "\nCaught signal %d, shutting down video stream...\n", sig);
	quit = true;
}

static RK_CHAR optstr[] = "?::a::b:w:h:l:o:e:d:D:I:i:L:M:";
static const struct option long_options[] = {
    {"aiq", optional_argument, NULL, 'a'},
    {"bitrate", required_argument, NULL, 'b'},
    {"device_name", required_argument, NULL, 'd'},
    {"width", required_argument, NULL, 'w'},
    {"height", required_argument, NULL, 'h'},
    {"input_bmp_name", required_argument, NULL, 'i'},
    {"loop_count", required_argument, NULL, 'l'},
    {"output_path", required_argument, NULL, 'o'},
    {"encode", required_argument, NULL, 'e'},
    {"disp_devid", required_argument, NULL, 'D'},
    {"camid", required_argument, NULL, 'I'},
    {"multictx", required_argument, NULL, 'M'},
    {"fps", required_argument, NULL, 'f'},
    {"hdr_mode", required_argument, NULL, 'h' + 'm'},
    {"help", optional_argument, NULL, '?'},
    {NULL, 0, NULL, 0},
};

/******************************************************************************
 * function : show usage
 ******************************************************************************/
static void print_usage(const RK_CHAR *name) {
	printf("Usage example (H.265 to USB CDC):\n");
	printf("\t%s -w 1920 -h 1080 -a /etc/iqfiles/ -I 0 -e h265cbr -b 2048 -d /dev/ttyGS0\n", name);
	printf("\nOptions:\n");
#ifdef RKAIQ
	printf("\t-a | --aiq: path to sensor IQ JSON directory, e.g., -a /etc/iqfiles/\n");
	printf("\t-M | --multictx: switch multictx in ISP, 0=disable, 1=enable (default 0)\n");
#endif
	printf("\t-d | --device_name: USB CDC serial node\n");
	printf("\t-I | --camid: camera index ID, default 0\n");
	printf("\t-w | --width: video width, default 1920\n");
	printf("\t-h | --height: video height, default 1080\n");
	printf("\t-e | --encode: codec type, default: h265cbr (values: h265cbr, h265vbr, h264cbr, h264vbr)\n");
	printf("\t-b | --bitrate: bitrate in kbps, default 2048 (2.0 Mbps)\n");
	printf("\t-l | --loop_count: frame loop count limit (-1 for infinite)\n");
}

/******************************************************************************
 * function : VENC stream retrieval & USB CDC output thread
 ******************************************************************************/
static void *venc_get_stream(void *pArgs) {
	SAMPLE_VENC_CTX_S *ctx = (SAMPLE_VENC_CTX_S *)(pArgs);
	RK_S32 s32Ret = RK_FAILURE;
	void *pData = RK_NULL;
	RK_S32 loopCount = 0;

	// Open USB CDC serial device node in Non-Blocking Mode
	const char *cdc_dev = ctx->dstFilePath ? ctx->dstFilePath : "/dev/ttyGS0";
	int cdc_fd = open(cdc_dev, O_WRONLY | O_NONBLOCK | O_NOCTTY);

	if (cdc_fd < 0) {
		// Fallback check to /dev/ttyGS0 if specified device failed
		cdc_dev = "/dev/ttyGS0";
		cdc_fd = open(cdc_dev, O_WRONLY | O_NONBLOCK | O_NOCTTY);
	}

	if (cdc_fd < 0) {
		printf("ERROR: Failed to open USB CDC device node (/dev/ttyGS0 or /dev/ttyGS0): %s\n", strerror(errno));
		quit = true;
		return RK_NULL;
	}

	printf("\n=======================================================\n");
	printf("SUCCESS: Streaming H.265 to USB CDC port: %s\n", cdc_dev);
	printf("=======================================================\n\n");

	while (!quit) {
		s32Ret = SAMPLE_COMM_VENC_GetStream(ctx, &pData);
		if (s32Ret == RK_SUCCESS) {
			if (ctx->s32loopCount > 0 && loopCount >= ctx->s32loopCount) {
				SAMPLE_COMM_VENC_ReleaseStream(ctx);
				quit = true;
				break;
			}

			// Extract NAL unit pointer and length
			uint8_t *packet_buf = (uint8_t *)(pData) + ctx->stFrame.pstPack->u32Offset;
			uint32_t packet_len = ctx->stFrame.pstPack->u32Len - ctx->stFrame.pstPack->u32Offset;

			if (packet_buf && packet_len > 0) {
				// Stream raw H.265 NAL unit out USB CDC
				ssize_t written = write(cdc_fd, packet_buf, packet_len);

				if (written < 0) {
					if (errno == EAGAIN || errno == EWOULDBLOCK) {
						// USB buffer full: Drop packet to maintain 0 latency
						RK_LOGD("USB CDC buffer full, frame dropped\n");
					} else {
						printf("USB CDC write error: %s\n", strerror(errno));
					}
				}
			}

			SAMPLE_COMM_VENC_ReleaseStream(ctx);
			loopCount++;
		}
		usleep(100); // sleep to yield CPU
	}

	if (cdc_fd >= 0) {
		close(cdc_fd);
		printf("Closed USB CDC device node\n");
	}

	return RK_NULL;
}

/******************************************************************************
 * function : main()
 * Description : Camera VI -> VPSS -> VENC H.265 -> USB CDC
 ******************************************************************************/
int main(int argc, char *argv[]) {
	SAMPLE_MPI_CTX_S *ctx;
	int video_width = 1920;
	int video_height = 1080;
	int venc_width = 1280;
	int venc_height = 720;
	//int disp_width = 1080;
	//int disp_height = 1920;
	//RK_CHAR *pDeviceName = NULL;
	RK_CHAR *pInPathBmp = NULL;
	RK_CHAR *pOutCdcDev = "/dev/ttyGS0";
	
	// Default to H.265 CBR at 2.0 Mbps
	CODEC_TYPE_E enCodecType = RK_CODEC_TYPE_H265;
	VENC_RC_MODE_E enRcMode = VENC_RC_MODE_H265CBR;
    RK_CHAR *pCodecName = "H265CBR";
	
	RK_S32 s32CamId = 0;
	RK_S32 s32DisId = -1;
	//RK_S32 s32DisLayerId = 0;
	RK_S32 s32loopCnt = -1;
	RK_S32 s32BitRate = 2 * 1024; // 2048 kbps = 2.0 Mbps
	MPP_CHN_S stSrcChn, stDestChn;

	// Target FPS
    int target_fps = 60;
	
	ctx = (SAMPLE_MPI_CTX_S *)(malloc(sizeof(SAMPLE_MPI_CTX_S)));
	memset(ctx, 0, sizeof(SAMPLE_MPI_CTX_S));

	// Catch SIGINT and SIGTERM for graceful exit
	signal(SIGINT, sigterm_handler);
	signal(SIGTERM, sigterm_handler);

#ifdef RKAIQ
	RK_BOOL bMultictx = RK_FALSE;
#endif
	int c;
	char *iq_file_dir = NULL;
	while ((c = getopt_long(argc, argv, optstr, long_options, NULL)) != -1) {
		const char *tmp_optarg = optarg;
		switch (c) {
		case 'a':
			if (!optarg && NULL != argv[optind] && '-' != argv[optind][0]) {
				tmp_optarg = argv[optind++];
			}
			if (tmp_optarg) {
				iq_file_dir = (char *)tmp_optarg;
			} else {
				iq_file_dir = NULL;
			}
			break;
		case 'b':
			s32BitRate = atoi(optarg);
			break;
		case 'd':
			pOutCdcDev = optarg;
			break;
		case 'D':
			s32DisId = atoi(optarg);
			break;
		case 'e':
			if (!strcmp(optarg, "h265cbr")) {
				enCodecType = RK_CODEC_TYPE_H265;
				enRcMode = VENC_RC_MODE_H265CBR;
				pCodecName = "H265CBR";
			} else if (!strcmp(optarg, "h265vbr")) {
				enCodecType = RK_CODEC_TYPE_H265;
				enRcMode = VENC_RC_MODE_H265VBR;
				pCodecName = "H265VBR";
			} else if (!strcmp(optarg, "h264cbr")) {
				enCodecType = RK_CODEC_TYPE_H264;
				enRcMode = VENC_RC_MODE_H264CBR;
				pCodecName = "H264CBR";
			} else if (!strcmp(optarg, "h264vbr")) {
				enCodecType = RK_CODEC_TYPE_H264;
				enRcMode = VENC_RC_MODE_H264VBR;
				pCodecName = "H264VBR";
			} else {
				printf("ERROR: Invalid encoder type specified: %s\n", optarg);
				return 0;
			}
			break;
		case 'w':
			video_width = atoi(optarg);
			venc_width = video_width;
			break;
		case 'h':
			video_height = atoi(optarg);
			venc_height = video_height;
			break;
		case 'I':
			s32CamId = atoi(optarg);
			break;
		case 'i':
			pInPathBmp = optarg;
			break;
		case 'l':
			s32loopCnt = atoi(optarg);
			break;
		case 'o':
			pOutCdcDev = optarg;
			break;
#ifdef RKAIQ
		case 'M':
			if (atoi(optarg)) {
				bMultictx = RK_TRUE;
			}
			break;
#endif
		case '?':
		default:
			print_usage(argv[0]);
			return 0;
		}
	}

	printf("\n--- Luckfox Pico H.265 USB CDC Streamer ---\n");
	printf("Camera Index : %d\n", s32CamId);
	printf("Resolution   : %dx%d\n", video_width, video_height);
	printf("Codec Mode   : %s\n", pCodecName);
	printf("Target Bitrate: %d kbps\n", s32BitRate);
	printf("Target USB CDC: %s\n", pOutCdcDev);
	printf("IQ Directory : %s\n\n", iq_file_dir ? iq_file_dir : "Auto");

	// Initialize RKAIQ ISP
	if (iq_file_dir) {
#ifdef RKAIQ
    rk_aiq_working_mode_t hdr_mode = RK_AIQ_WORKING_MODE_NORMAL;
    SAMPLE_COMM_ISP_Init(s32CamId, hdr_mode, bMultictx, iq_file_dir);
    SAMPLE_COMM_ISP_Run(s32CamId);

    //    Force Camera Sensor & ISP Exposure Engine to 60 FPS
    //    rk_aiq_sys_ctx_t *aiq_ctx = rk_aiq_uapi2_sysctl_get_ctx(s32CamId);
    //	  if (aiq_ctx) {
    //    rk_aiq_user_api2_ae_setFixFrameRate(aiq_ctx, 60);
    //    printf("ISP locked to 60 FPS\n");
    //}
#endif
	}

	if (RK_MPI_SYS_Init() != RK_SUCCESS) {
		goto __FAILED;
	}

// Init VI[0]
    ctx->vi.u32Width = video_width;   // 1920
    ctx->vi.u32Height = video_height; // 1080
    ctx->vi.s32DevId = s32CamId;
    ctx->vi.u32PipeId = ctx->vi.s32DevId;
    ctx->vi.s32ChnId = 1;
    ctx->vi.stChnAttr.stIspOpt.u32BufCount = 2;
    ctx->vi.stChnAttr.stIspOpt.enMemoryType = VI_V4L2_MEMORY_TYPE_DMABUF;
    ctx->vi.stChnAttr.u32Depth = 0;
    ctx->vi.stChnAttr.enPixelFormat = RK_FMT_YUV420SP;
    
    // Set VI Framerate to 60 FPS
    ctx->vi.stChnAttr.stFrameRate.s32SrcFrameRate = target_fps;
    ctx->vi.stChnAttr.stFrameRate.s32DstFrameRate = target_fps;
    
    SAMPLE_COMM_VI_CreateChn(&ctx->vi);

// Init VPSS[0] (Hardware RGA Downscaler)
    ctx->vpss.s32GrpId = 0;
    ctx->vpss.s32ChnId = 0;
    ctx->vpss.enVProcDevType = VIDEO_PROC_DEV_RGA; // Hardware RGA
    ctx->vpss.stGrpVpssAttr.enPixelFormat = RK_FMT_YUV420SP;
    ctx->vpss.stGrpVpssAttr.enCompressMode = COMPRESS_MODE_NONE;

    // VPSS Group Input Size (Full 1080p from camera)
    ctx->vpss.stCropInfo.bEnable = RK_FALSE;
    ctx->vpss.stCropInfo.stCropRect.u32Width = video_width;   // 1920
    ctx->vpss.stCropInfo.stCropRect.u32Height = video_height; // 1080

    // VPSS Channel 0 Output Size (Downscaled to 720p)
    ctx->vpss.stVpssChnAttr[0].enChnMode = VPSS_CHN_MODE_USER;
    ctx->vpss.stVpssChnAttr[0].enCompressMode = COMPRESS_MODE_NONE;
    ctx->vpss.stVpssChnAttr[0].enDynamicRange = DYNAMIC_RANGE_SDR8;
    ctx->vpss.stVpssChnAttr[0].enPixelFormat = RK_FMT_YUV420SP;
    ctx->vpss.stVpssChnAttr[0].u32Width = venc_width;   // 1280
    ctx->vpss.stVpssChnAttr[0].u32Height = venc_height; // 720
    
    // Set VPSS Framerate to 60 FPS
    ctx->vpss.stVpssChnAttr[0].stFrameRate.s32SrcFrameRate = target_fps;
    ctx->vpss.stVpssChnAttr[0].stFrameRate.s32DstFrameRate = target_fps;
    
    SAMPLE_COMM_VPSS_CreateChn(&ctx->vpss);

// Init VENC[0] (H.265 Hardware Encoder)
    ctx->venc.s32ChnId = 0;
    ctx->venc.u32Width = venc_width;   // 1280
    ctx->venc.u32Height = venc_height; // 720
    ctx->venc.u32Fps = target_fps;      // Target FPS
    ctx->venc.u32Gop = 2 * target_fps;      // (complete) I-Frame distance
    ctx->venc.u32BitRate = s32BitRate;  // 2000 kbps (2.0 Mbps CBR)
    ctx->venc.enCodecType = enCodecType;
    ctx->venc.enRcMode = enRcMode;
    ctx->venc.getStreamCbFunc = venc_get_stream;
    ctx->venc.dstFilePath = pOutCdcDev;
    ctx->venc.stChnAttr.stVencAttr.u32Profile = 0; // H.265 Main Profile
    ctx->venc.stChnAttr.stGopAttr.enGopMode = VENC_GOPMODE_NORMALP;
    
/*
	ctx->venc.stChnAttr.stRcAttr.stH265Cbr.u32MinQp = 28;
	ctx->venc.stChnAttr.stRcAttr.stH265Cbr.u32MaxQp = 45;
	ctx->venc.stChnAttr.stRcAttr.stH265Cbr.u32MinIQp = 28;
	ctx->venc.stChnAttr.stRcAttr.stH265Cbr.u32MaxIQp = 45;
*/


    SAMPLE_COMM_VENC_CreateChn(&ctx->venc);
	// Enable Gradual / Periodic Intra Refresh (PIR) for Flat Radio Bitrate
	VENC_INTRA_REFRESH_S stIntraRefresh;
	memset(&stIntraRefresh, 0, sizeof(VENC_INTRA_REFRESH_S));

	stIntraRefresh.bRefreshEnable = RK_TRUE;                  // Enable PIR
	stIntraRefresh.enIntraRefreshMode = INTRA_REFRESH_ROW;    // Refresh row-by-row (or INTRA_REFRESH_COLUMN)
	stIntraRefresh.u32RefreshNum = venc_height / 60;          // Number of rows refreshed per frame (e.g. 720 / 60 = 12 rows)
	stIntraRefresh.u32ReqIQp = 28;                            // Intra refresh QP quality 0 to 51 low is higher bitrate

	// Apply to VENC Channel 0
	RK_S32 s32Ret = RK_MPI_VENC_SetIntraRefresh(ctx->venc.s32ChnId, &stIntraRefresh);
	if (s32Ret == RK_SUCCESS) {
		printf("Gradual Intra Refresh (PIR) ENABLED: Packet sizes are now 100%% flat!\n");
	} else {
		printf("Warning: Failed to set Intra Refresh (Error: %d)\n", s32Ret);
	}

	// Bind VI[0] to VPSS[0]
	stSrcChn.enModId = RK_ID_VI;
	stSrcChn.s32DevId = ctx->vi.s32DevId;
	stSrcChn.s32ChnId = ctx->vi.s32ChnId;
	stDestChn.enModId = RK_ID_VPSS;
	stDestChn.s32DevId = ctx->vpss.s32GrpId;
	stDestChn.s32ChnId = ctx->vpss.s32ChnId;
	SAMPLE_COMM_Bind(&stSrcChn, &stDestChn);

	// Bind VPSS[0] to VENC[0]
	stSrcChn.enModId = RK_ID_VPSS;
	stSrcChn.s32DevId = ctx->vpss.s32GrpId;
	stSrcChn.s32ChnId = ctx->vpss.s32ChnId;
	stDestChn.enModId = RK_ID_VENC;
	stDestChn.s32DevId = 0;
	stDestChn.s32ChnId = ctx->venc.s32ChnId;
	SAMPLE_COMM_Bind(&stSrcChn, &stDestChn);

	printf("Camera & Hardware Encoder Initialization Finished. Streaming...\n");

	while (!quit) {
		sleep(1);
	}

	printf("\nShutting down pipeline...\n");

	if (ctx->venc.getStreamCbFunc) {
		pthread_join(ctx->venc.getStreamThread, NULL);
	}

	// UnBind VPSS[0] and VENC[0]
	stSrcChn.enModId = RK_ID_VPSS;
	stSrcChn.s32DevId = ctx->vpss.s32GrpId;
	stSrcChn.s32ChnId = ctx->vpss.s32ChnId;
	stDestChn.enModId = RK_ID_VENC;
	stDestChn.s32DevId = 0;
	stDestChn.s32ChnId = ctx->venc.s32ChnId;
	SAMPLE_COMM_UnBind(&stSrcChn, &stDestChn);

	// UnBind VI[0] and VPSS[0]
	stSrcChn.enModId = RK_ID_VI;
	stSrcChn.s32DevId = ctx->vi.s32DevId;
	stSrcChn.s32ChnId = ctx->vi.s32ChnId;
	stDestChn.enModId = RK_ID_VPSS;
	stDestChn.s32DevId = ctx->vpss.s32GrpId;
	stDestChn.s32ChnId = ctx->vpss.s32ChnId;
	SAMPLE_COMM_UnBind(&stSrcChn, &stDestChn);

	// Destroy VENC, VPSS, and VI channels
	SAMPLE_COMM_VENC_DestroyChn(&ctx->venc);
	SAMPLE_COMM_VPSS_DestroyChn(&ctx->vpss);
	SAMPLE_COMM_VI_DestroyChn(&ctx->vi);

__FAILED:
	RK_MPI_SYS_Exit();
	if (iq_file_dir) {
#ifdef RKAIQ
		SAMPLE_COMM_ISP_Stop(s32CamId);
#endif
	}
	if (ctx) {
		free(ctx);
		ctx = RK_NULL;
	}

	return 0;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */