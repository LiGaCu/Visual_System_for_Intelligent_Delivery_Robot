#ifndef VIDEOENCODER_H
#define VIDEOENCODER_H

#include <stdio.h>

extern "C"
{
	#include <libavcodec/avcodec.h>
	#include <libavutil/opt.h>
	#include <libavutil/imgutils.h>
	#include <libswscale/swscale.h>
}
#include "opencv2/opencv.hpp"

using namespace::std;
using namespace::cv;

class myselfencoder
{
public:
	uchar I_P_type=(uchar)'P';
	myselfencoder(int w, int h);
	~myselfencoder();
	int encoderinit();
	int bgr2yuv(Mat& src);
	int beginencode(vector<uchar> *buff264);
private:
	int width, height;
	int src_width, src_height;
	int ret;
	AVCodec *pCodec;
	AVCodecContext *pCodecCtx = NULL;
	AVFrame *pFrame;
	AVPacket *pkt;
	AVCodecID codec_id = AV_CODEC_ID_H264;

	uint8_t *src_data[4], *dst_data[4];
	int src_linesize[4], dst_linesize[4];
	SwsContext *sws_ctx;
	enum AVPixelFormat src_pix_fmt = AV_PIX_FMT_BGR24, dst_pix_fmt = AV_PIX_FMT_YUV420P;
	int notinitall = 1;
};

#endif