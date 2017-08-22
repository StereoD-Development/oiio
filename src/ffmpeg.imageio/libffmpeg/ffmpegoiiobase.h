/*
  Copyright 2014 Larry Gritz and the other authors and contributors.
  All Rights Reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of the software's owners nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  (This is the Modified BSD License)
*/

#ifndef FFMPEGOIIOBASE_H
#define FFMPEGOIIOBASE_H

/// ------------------------------------------------------------------------
/// -- Boiler Plate
/// ------------------------------------------------------------------------

extern "C" { // ffmpeg is a C api
#include <cerrno>
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(57,24,0)
# include <libavutil/imgutils.h>
# include <libavutil/pixfmt.h>
# include <libavutil/opt.h>
#endif
}

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

#define OIIO_FFMPEG_BUF_SIZE 32768

#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55,28,1)
#  define av_frame_alloc  avcodec_alloc_frame
//Ancient versions used av_freep
#  if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(54,59,100)
#    define av_frame_free   av_freep
#  else
#    define av_frame_free   avcodec_free_frame
#  endif
#endif

// PIX_FMT was renamed to AV_PIX_FMT on this version
#if LIBAVUTIL_VERSION_INT < AV_VERSION_INT(51,74,100)
#  define AVPixelFormat PixelFormat
#  define AV_PIX_FMT_RGB24 PIX_FMT_RGB24
#  define AV_PIX_FMT_RGB48 PIX_FMT_RGB48
#  define AV_PIX_FMT_YUVJ420P PIX_FMT_YUVJ420P
#  define AV_PIX_FMT_YUVJ422P PIX_FMT_YUVJ422P
#  define AV_PIX_FMT_YUVJ440P PIX_FMT_YUVJ440P
#  define AV_PIX_FMT_YUVJ444P PIX_FMT_YUVJ444P
#  define AV_PIX_FMT_YUV420P  PIX_FMT_YUV420P
#  define AV_PIX_FMT_YUV422P  PIX_FMT_YUV422P
#  define AV_PIX_FMT_YUV440P  PIX_FMT_YUV440P
#  define AV_PIX_FMT_YUV444P  PIX_FMT_YUV444P
#endif

// r_frame_rate deprecated in ffmpeg
// see ffmpeg commit #aba232c for details
#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(52,42,0)
#  define r_frame_rate avg_frame_rate
#endif

// Changes for ffmpeg 3.0
#define USE_FFMPEG_3_0 (LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(57,24,0))

#if USE_FFMPEG_3_0
#  define av_free_packet av_packet_unref
#  define avpicture_get_size(fmt,w,h) av_image_get_buffer_size(fmt,w,h,1)

inline int avpicture_fill(AVPicture *picture, uint8_t *ptr,
                          enum AVPixelFormat pix_fmt, int width, int height)
{
    AVFrame *frame = reinterpret_cast<AVFrame *>(picture);
    return av_image_fill_arrays(frame->data, frame->linesize,
                                ptr, pix_fmt, width, height, 1);
}
#endif

// Changes for ffmpeg 3.1
#define USE_FFMPEG_3_1 (LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(57, 48, 101))

#if USE_FFMPEG_3_1
// AVStream::codec was changed to AVStream::codecpar
#  define stream_codec(ix) m_format_context->streams[(ix)]->codecpar
// avcodec_decode_video2 was deprecated.
// This now works by sending `avpkt` to the decoder, which buffers the
// decoded image in `avctx`. Then `avcodec_receive_frame` will copy the
// frame to `picture`.
inline int receive_frame(AVCodecContext *avctx, AVFrame *picture,
                         AVPacket *avpkt, int *got_frame)
{
    int ret;
    // Currently, as of Aug 2017, the send/receive offers poor
    // performence when we are looking for realtime playback of
    // imagery. For now, we'll stick with the outmoded, but effective
    // avcodec_decode_video2
    avcodec_decode_video2(avctx, picture, &ret, avpkt);

    // -- The code for send/receive synchronously
    // *got_frame = 0;
    // if (avpkt) {
    //     ret = avcodec_send_packet(avctx, avpkt);
    //     if (ret < 0)
    //         return (ret == AVERROR_EOF) ? 0 : ret;
    // }

    // ret = avcodec_receive_frame(avctx, picture);
    // if (ret < 0 && ret != AVERROR(EAGAIN) && ret != AVERROR_EOF)
    //     return ret;
    // if (ret >= 0)
    //     *got_frame = 1;

    return ret;
}
#else
#  define stream_codec(ix) m_format_context->streams[(ix)]->codec
inline int receive_frame(AVCodecContext *avctx, AVFrame *picture,
                         AVPacket *avpkt, int *got_frame)
{
    int ret;
    *got_frame = (ret >= 0) ? 1 : 0;
    avcodec_decode_video2(avctx, picture, &ret, avpkt);
    return ret;
}
#endif


/// ------------------------------------------------------------------------
/// -- Includes, Statics, And Structures 
/// ------------------------------------------------------------------------

#include <boost/thread/once.hpp>

#include <OpenImageIO/imageio.h>
#include <OpenImageIO/stream.h>
#include <iostream>


#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#endif // FFMPEGOIIOBASE_H
