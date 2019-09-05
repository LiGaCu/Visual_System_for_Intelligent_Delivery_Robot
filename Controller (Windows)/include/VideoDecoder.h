#ifndef VIDEODECODER_H
#define VIDEODECODER_H

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

class myselfdecoder
{
public:
	myselfdecoder();
	~myselfdecoder();
	int decoderinit();
	int begindecode(vector<uchar> *buff264, Mat& src);
private:
	int width, height;
	int ret;
	AVCodec *pCodec;
	AVCodecContext *pCodecCtx = NULL;
	AVFrame *pFrame;
	AVPacket *pkt;
	AVCodecID codec_id = AV_CODEC_ID_H264;
	AVCodecParserContext *parser;

	uint8_t *src_data[4], *dst_data[4];
	int src_linesize[4], dst_linesize[4];
	SwsContext *sws_ctx;
	enum AVPixelFormat src_pix_fmt = AV_PIX_FMT_YUV420P, dst_pix_fmt = AV_PIX_FMT_BGR24;
	int notinitall=1;
};

#endif