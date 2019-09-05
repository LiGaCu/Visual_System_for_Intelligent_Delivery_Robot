#include "VideoDecoder.h"
#include <iostream>

using namespace::std;

myselfdecoder::myselfdecoder()
{
}

myselfdecoder::~myselfdecoder()
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

int myselfdecoder::decoderinit()
{
	pkt = av_packet_alloc();
	if (!pkt)
		return -1;

	pCodec = avcodec_find_decoder(codec_id);
	//pCodec = avcodec_find_encoder_by_name("libx264rgb");
	if (!pCodec) 
	{
		cout<<"Codec not found\n";
		return -1;
	}
	parser = av_parser_init(pCodec->id);
	if (!parser) {
		fprintf(stderr, "parser not found\n");
		exit(1);
	}
	pCodecCtx = avcodec_alloc_context3(pCodec);
	if (!pCodecCtx) {
		cout<<"Could not allocate video codec context\n";
		return -1;
	}

	/* For some codecs, such as msmpeg4 and mpeg4, width and height
	MUST be initialized there because this information is not
	available in the bitstream. */
	/* open it */
	ret = avcodec_open2(pCodecCtx, pCodec, NULL);
	if (ret< 0)
	{
		cout << "Could not open codec\n";
		return -1;
	}
	pFrame = av_frame_alloc();
	if (!pFrame)
	{
		cout << "Could not allocate video frame\n";
		return -1;
	}
	return 0;
}

int myselfdecoder::begindecode(vector<uchar> *buff264, Mat& src)
{
	/*ret = av_parser_parse2(parser, pCodecCtx, &pkt->data, &pkt->size,
		buff264->data(), buff264->size(), AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
	if (ret < 0)
	{
		cout<< "Error while parsing\n";
		return -1;
	}*/

	ret = av_packet_from_data(pkt, buff264->data(), buff264->size());
	if (ret < 0)
	{
		cout << "Error while transing data to packet\n";
		return -1;
	}

	ret = avcodec_send_packet(pCodecCtx, pkt);

	if (ret < 0) 
	{
		cout<<"Error sending a packet for decoding\n";
		return -1;
	}

	ret = avcodec_receive_frame(pCodecCtx, pFrame);
	if (ret < 0) 
	{
		cout<<"Error during decoding\n";
		return -1;
	}

	if (notinitall)
	{
		width = pFrame->width;
		height = pFrame->height;
		src_linesize[0] = pFrame->linesize[0];
		src_linesize[1] = pFrame->linesize[1];
		src_linesize[2] = pFrame->linesize[2];

		/* create scaling context */
		sws_ctx = sws_getContext(width, height, src_pix_fmt,
			width, height, dst_pix_fmt,
			SWS_BILINEAR, NULL, NULL, NULL);
		if (!sws_ctx)
		{
			cout << "Impossible to create scale context for the conversion ";
			return -1;
		}
		/* allocate source and destination image buffers */
		if ((ret = av_image_alloc(src_data, src_linesize,
			width, height, src_pix_fmt, 16)) < 0) {
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
	src_data[0] = pFrame->data[0];
	src_data[1] = pFrame->data[1];
	src_data[2] = pFrame->data[2];

	/* convert to destination format */
	sws_scale(sws_ctx, (const uint8_t * const*)src_data,
		src_linesize, 0, height, dst_data, dst_linesize);
	Mat temp(height, width, CV_8UC3);
	temp.data = dst_data[0];
	temp.copyTo(src);
	return 0;
}
