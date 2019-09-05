#include "../include/VideoEncoder.h"
#include <iostream>

using namespace::std;

/*w,h为设定编码完成后的长和宽*/
myselfencoder::myselfencoder(int w,int h)
{
	width = w;
	height = h;
}

myselfencoder::~myselfencoder()
{
	avcodec_close(pCodecCtx);
	av_free(pCodecCtx);
	av_frame_free(&pFrame);
	av_packet_free(&pkt);
	av_freep(&pFrame->data[0]);

	av_freep(&src_data[0]);
	av_freep(&dst_data[0]);
	sws_freeContext(sws_ctx);
}

int myselfencoder::encoderinit()
{
	pCodec = avcodec_find_encoder(codec_id);
	//pCodec = avcodec_find_encoder_by_name("libx264rgb");
	if (!pCodec) 
	{
		cout<<"Codec not found\n";
		return -1;
	}
	pCodecCtx = avcodec_alloc_context3(pCodec);
	if (!pCodecCtx) {
		cout<<"Could not allocate video codec context\n";
		return -1;
	}
	pkt = av_packet_alloc();
	if (!pkt)
		return -1;

//	pCodecCtx->bit_rate = 400000;
	pCodecCtx->width = width;
	pCodecCtx->height = height;
	pCodecCtx->time_base.num = 1;
	pCodecCtx->time_base.den = 30;
	pCodecCtx->gop_size = 15;
	pCodecCtx->max_b_frames = 0;
	pCodecCtx->pix_fmt = AV_PIX_FMT_YUV420P;
	pCodecCtx->thread_count = 1;
	av_opt_set(pCodecCtx->priv_data, "tune", "zerolatency", 0);
	av_opt_set(pCodecCtx->priv_data, "preset", "slow", 0);

	/* open it */
	ret = avcodec_open2(pCodecCtx, pCodec, NULL);
	if (ret< 0)
	{
		cout<<"Could not open codec\n";
		return -1;
	}
	pFrame = av_frame_alloc();
	if (!pFrame)
	{
		cout<<"Could not allocate video frame\n";
		return -1;
	}
	pFrame->format = pCodecCtx->pix_fmt;
	pFrame->width = pCodecCtx->width;
	pFrame->height = pCodecCtx->height;
	pFrame->pts = 0;
	ret = av_frame_get_buffer(pFrame, 32);
	if (ret < 0) 
	{
		cout<< "Could not allocate the video frame data\n";
		return -1;
	}
	return 0;
}

int myselfencoder::beginencode(vector<uchar> *buff264)
{
	ret = av_frame_make_writable(pFrame);
	if (ret < 0)
	{
		return -1;
	}
	pFrame->data[0] = dst_data[0];
	pFrame->data[1] = dst_data[1];
	pFrame->data[2] = dst_data[2];
	pFrame->pts++;

	/* send the frame to the encoder */
	//if (pFrame)
	//	cout<<"Send frame:"<< pFrame->pts << " 	" << ret <<"\n";

	ret = avcodec_send_frame(pCodecCtx, pFrame);
	if (ret < 0) 
	{
		cout<< "Error sending a frame for encoding\n";
		return -1;
	}
	ret = avcodec_receive_packet(pCodecCtx, pkt);
	if (ret < 0)
	{
		cout << "Error recieving a frame for encoding\n";
		return -1;
	}
	//cout << "Receive frame:" << pFrame->pts <<" 	"<<ret<<"\n";
	uchar *ppktdata= pkt->data;
	buff264->clear();
	for (auto i = 0; i < pkt->size; i++)
	{
		buff264->push_back(*ppktdata);
		ppktdata++;
	}
	if (pkt->pts % 15 == 1)
		I_P_type =(uchar) 'I';
	else
		I_P_type =(uchar) 'P';
	av_packet_unref(pkt);
	return 0;
}

/*自动将Mat大小转化为设定待发送大小*/
int myselfencoder::bgr2yuv(Mat& src)
{
	if (notinitall)
	{
		src_width = src.cols;
		src_height = src.rows;
		/* create scaling context */
		sws_ctx = sws_getContext(src_width, src_height, src_pix_fmt,
			width, height, dst_pix_fmt,
			SWS_BILINEAR, NULL, NULL, NULL);
		if (!sws_ctx)
		{
			cout << "Impossible to create scale context for the conversion ";
			return -1;
		}

		/* allocate source and destination image buffers */
		if ((ret = av_image_alloc(src_data, src_linesize,
			src_width, src_height, src_pix_fmt, 16)) < 0) {
			cout << "Could not allocate source image\n";
			return -1;
		}

		/* buffer is going to be written to rawvideo file, no alignment */
		if ((ret = av_image_alloc(dst_data, dst_linesize,
			width, height, dst_pix_fmt, 1)) < 0) {
			cout << "Could not allocate destination image\n";
			return -1;
		}
		notinitall = 0;
	}
	src_data[0] = src.data;
	/* convert to destination format */
	sws_scale(sws_ctx, (const uint8_t * const*)src_data,
		src_linesize, 0, src_height, dst_data, dst_linesize);
	return 0;
}