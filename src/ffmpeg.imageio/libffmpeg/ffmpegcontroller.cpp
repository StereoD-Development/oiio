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

#include "ffmpegcontroller.h"

OIIO_NAMESPACE_BEGIN


namespace ffmpeg_sio // Stream I/O
{

    // FFmpeg stream reading requires this for buffer logic. It's integral
    // when dealing with the in-memory buffer objects that select OIIO plugins
    // support. When we are given a movie file to stream/seek, we pass these
    // functions into av_seek_frame().

    static int read_packet (void *opaque, uint8_t *buf, int buf_size)
    {
        // Read function that handles our membuf transparently.

        auto stream = ((OIIO::no_copy_membuf*)opaque);

        std::ptrdiff_t s = stream->size();
        if (s > (std::numeric_limits<int>::max()))
        {
            // If we're too big, use the buf_size
            s = buf_size;
        }

        buf_size = std::min(buf_size, (int)s); // If that's all they supply then so be it.

        if (stream->end_of_buffer())
            return AVERROR_EOF;

        stream->read((char *)buf, buf_size);
        return buf_size;
    }

    static int64_t seek (void *opaque, int64_t pos, int whence)
    {
        // Seek function for determining position as well as moving
        // our internal buffer read pointer.

        auto stream = ((OIIO::no_copy_membuf*)opaque);

        if (whence == AVSEEK_SIZE)
        {
            return stream->size ();
        }

        return stream->seekg (pos).tellg ();
    }

};


FFmpegController::FFmpegController ()
    : m_index(-1)
    , m_avstream(NULL)
    , m_codec_context(NULL)
    , m_codec(NULL)
    , m_avframe(NULL)
    , m_rgb_frame(nullptr)
    , m_convert_context(NULL)
    , m_format_context(NULL)
    , m_fps_num(1)
    , m_fps_den(1)
    , m_start_pts(0)
    , m_frames(-1)
    , m_pts_seen(false)
    , m_timestamp_field(&AVPacket::pts)
    , m_width(0)
    , m_height(0)
    , m_aspect(1.0)
    , m_decode_next_frame_in(-1)
    , m_decode_next_frame_out(-1)
    , m_decode_latency(0)
    , m_frame_calculated(-1)
    , m_stride(0)
    , m_error_string(std::string())
    , m_error_code(0)
{}



FFmpegController::~FFmpegController ()
{
    /// Run cleanup on all FFmpeg items we've generated
    if (m_avframe)
        av_frame_free (&m_avframe);
  
    if (m_codec_context)
        avcodec_close (m_codec_context);
  
    if (m_convert_context)
        sws_freeContext (m_convert_context);
  
    if (m_format_context)
    {
        if (m_format_context->pb)
        {
            av_freep (&m_format_context->pb->buffer);
            av_freep (&m_format_context->pb);
        }
        avformat_close_input (&m_format_context);    
    }

    av_frame_free (&m_rgb_frame);
}



int64_t
FFmpegController::open (const char *file_name, ImageSpec &spec, ImageInput *inp)
{
    if (avformat_find_stream_info (m_format_context, NULL) < 0)
    {
        inp->error ("\"%s\" could not find stream info", file_name);
        return -1;
    }
    m_index = -1;
    for (unsigned int i = 0; i < m_format_context->nb_streams; i++) {
        if (stream_codec(i)->codec_type == AVMEDIA_TYPE_VIDEO) {
            if (m_index < 0) {
                m_index = i;
            }
            // m_video_indexes.push_back (i); // needed for later use
            break;
        }
    }
    if (m_index == -1) {
        inp->error ("\"%s\" could not find a valid videostream", file_name);
        return -1;
    }

    // codec context for videostream
#if USE_FFMPEG_3_1
    AVCodecParameters *par = stream_codec(m_index);

    m_codec = avcodec_find_decoder(par->codec_id);
    if (!m_codec) {
        inp->error ("\"%s\" can't find decoder", file_name);
        return -1;
    }
    m_codec_context = avcodec_alloc_context3(m_codec);
    if (!m_codec_context) {
        inp->error ("\"%s\" can't allocate decoder context", file_name);
        return -1;
    }

    int ret;

    ret = avcodec_parameters_to_context(m_codec_context, par);
    if (ret < 0) {
        inp->error ("\"%s\" unsupported codec", file_name);
        return -1;
    }
#else
    m_codec_context = stream_codec(m_index);

    m_codec = avcodec_find_decoder (m_codec_context->codec_id);
    if (!m_codec) {
        inp->error ("\"%s\" unsupported codec", file_name);
        return -1;
    }
#endif

    av_opt_set(m_codec_context->priv_data, "tune", "zerolatency", 0);
    m_codec_context->flags2 |= AV_CODEC_FLAG2_FAST;
    m_codec_context->flags2 |= AV_CODEC_FLAG_LOW_DELAY;

    if (avcodec_open2 (m_codec_context, m_codec, NULL) < 0) {
        inp->error ("\"%s\" could not open codec", file_name);
        return -1;
    }

    m_avstream = m_format_context->streams[m_index];
    if (m_avstream->r_frame_rate.num != 0 && m_avstream->r_frame_rate.den != 0) {
        m_frame_rate = m_avstream->r_frame_rate;
    }

    m_frames = m_avstream->nb_frames;
    m_start_pts = m_avstream->start_time;
    if (!m_frames)
    {
        seek (0);
        av_init_packet (&m_avPacket);
        av_read_frame (m_format_context, &m_avPacket);

        int64_t first_pts = m_avPacket.pts;
        int64_t max_pts = 0 ;
        av_free_packet (&m_avPacket);
        seek (1 << 29);
        av_init_packet (&m_avPacket);

        while (m_avstream && av_read_frame (m_format_context, &m_avPacket) >= 0) {
            int64_t current_pts = static_cast<int64_t> (
                av_q2d(m_avstream->time_base) * (m_avPacket.pts - first_pts) * fps()
            );
            if (current_pts > max_pts) {
                max_pts = current_pts + 1;
            }
            av_free_packet (&m_avPacket); //Always free before format_context usage
        }
        m_frames = max_pts;
    }
    m_avframe = av_frame_alloc();
    m_rgb_frame = av_frame_alloc();

    switch (m_codec_context->pix_fmt) { // deprecation warning for YUV formats
        case AV_PIX_FMT_YUVJ420P:
            m_src_pix_format = AV_PIX_FMT_YUV420P;
            break;
        case AV_PIX_FMT_YUVJ422P:
            m_src_pix_format = AV_PIX_FMT_YUV422P;
            break;
        case AV_PIX_FMT_YUVJ444P:
            m_src_pix_format = AV_PIX_FMT_YUV444P;
            break;
        case AV_PIX_FMT_YUVJ440P:
            m_src_pix_format = AV_PIX_FMT_YUV440P;
            break;
        default:
            m_src_pix_format = m_codec_context->pix_fmt;
            break;
    }

    spec = ImageSpec (m_codec_context->width, m_codec_context->height, 3);

    switch (m_src_pix_format) {
        // support for 10-bit and 12-bit pix_fmts
        case AV_PIX_FMT_YUV420P10BE:
        case AV_PIX_FMT_YUV420P10LE:
        case AV_PIX_FMT_YUV422P10BE:
        case AV_PIX_FMT_YUV422P10LE:
        case AV_PIX_FMT_YUV444P10BE:
        case AV_PIX_FMT_YUV444P10LE:
        case AV_PIX_FMT_YUV420P12BE:
        case AV_PIX_FMT_YUV420P12LE:
        case AV_PIX_FMT_YUV422P12BE:
        case AV_PIX_FMT_YUV422P12LE:
        case AV_PIX_FMT_YUV444P12BE:
        case AV_PIX_FMT_YUV444P12LE:
            spec.set_format (TypeDesc::UINT16);
            m_dst_pix_format = AV_PIX_FMT_RGB48;
            m_stride = (size_t) (spec.width * 3 * 2);
            break;
        default:
            spec.set_format (TypeDesc::UINT8);
            m_dst_pix_format = AV_PIX_FMT_RGB24;
            m_stride = (size_t) (spec.width * 3);
            break;
    }

    m_rgb_buffer.resize(
        avpicture_get_size (m_dst_pix_format,
        m_codec_context->width,
        m_codec_context->height),
        0
    );

    AVDictionaryEntry *tag = NULL;
    while ((tag = av_dict_get (m_format_context->metadata, "", tag, AV_DICT_IGNORE_SUFFIX))) {
        spec.attribute (tag->key, tag->value);
    }
    int rat[2] = { m_frame_rate.num, m_frame_rate.den };

    m_fps_num = m_frame_rate.num;
    m_fps_den = m_frame_rate.den;

    spec.attribute ("FramesPerSecond", TypeDesc::TypeRational, &rat);
    spec.attribute ("oiio:Movie", true);
    spec.attribute ("oiio:BitsPerSample", m_codec_context->bits_per_raw_sample);
    spec.attribute ("oiio:NumberOfFrames", (int)m_frames);

    m_width = stream_codec(m_index)->width;
    m_height = stream_codec(m_index)->height;
    return m_frames;
}



int64_t
FFmpegController::open (char *buffer, size_t size, ImageSpec &spec, ImageInput *inp)
{

    m_stream = OIIO::no_copy_membuf (buffer, size);
    // Give the avio_alloc_context buffer some space
    m_context_buffer = (uint8_t*)av_malloc (OIIO_FFMPEG_BUF_SIZE);
    if (!m_context_buffer)
    {
        inp->error ("FFmpegInput::open(*buffer...): couldn't allocate context buffer.");
        return -1;
    }

    // In order to use a buffer, ffpmeg must aquire the format and
    // other information about the item passed in.
    m_format_context = avformat_alloc_context ();
    m_format_context->pb = avio_alloc_context (
        (unsigned char*)m_context_buffer, OIIO_FFMPEG_BUF_SIZE,
        0, &m_stream, ffmpeg_sio::read_packet, NULL, ffmpeg_sio::seek
    );
    if (!m_format_context->pb)
    {
        inp->error ("FFmpegInput::open(*buffer...): couldn't create avio context.");
        return -1;
    }

    // Probe for the input format.
    AVProbeData probe_data;
    probe_data.buf_size = (size < OIIO_FFMPEG_BUF_SIZE) ? size : OIIO_FFMPEG_BUF_SIZE;
    probe_data.filename = "stream";
    probe_data.buf = (unsigned char *)malloc (probe_data.buf_size);
    memcpy(probe_data.buf, buffer, OIIO_FFMPEG_BUF_SIZE);

    AVInputFormat *input_format = av_probe_input_format(&probe_data, 1);
    if (!input_format)
        input_format = av_probe_input_format(&probe_data, 0);

    free(probe_data.buf);
    probe_data.buf = nullptr;

    input_format->flags |= AVFMT_NOFILE;
    int err = avformat_open_input(&m_format_context,
                                  "stream", input_format, NULL);
    if (err != 0)
    {
        inp->error ("FFmpegInput::open(*buffer...): Error code: %s", err);
        return -1;
    }

    return open ("stream", spec, inp);

}



int64_t
FFmpegController::frame_to_pts (int frame) const
{
    return m_start_pts + (int64_t(frame) * m_fps_den * m_avstream->time_base.den) /
                         (int64_t(m_fps_num) * m_avstream->time_base.num);
}



int
FFmpegController::pts_to_frame (int64_t pts) const
{
    return (int64_t(pts - m_start_pts) * m_avstream->time_base.num *  m_fps_num) / 
           (int64_t(m_avstream->time_base.den) * m_fps_den);
}



SwsContext*
FFmpegController::convert_context ()
{
    if (!m_convert_context)
        m_convert_context = sws_getContext(m_width, m_height, m_src_pix_format,
                                           m_width, m_height, m_dst_pix_format,
                                           SWS_BICUBIC, NULL, NULL, NULL);
    return m_convert_context;
}



int
FFmpegController::codec_delay () const
{
    if (m_codec->capabilities & CODEC_CAP_DELAY)
        return m_codec_context->delay + m_codec_context->has_b_frames;
    return m_codec_context->has_b_frames;
}



bool
FFmpegController::seek (int frame)
{
    int64_t offset = frame_to_pts (frame);
    int flags = AVSEEK_FLAG_BACKWARD;
    avcodec_flush_buffers (m_codec_context);
    m_error_code = av_seek_frame (m_format_context, m_index, offset, flags );
    if (m_error_code < 0)
    {
        m_error_string = "FFmpegInput: Failed to seek for given frame.";
        return false;
    }
    m_frame_calculated = -1;
    return true;
}



double
FFmpegController::fps () const
{
    if (m_frame_rate.den)
        return av_q2d(m_frame_rate);
    return 1.0f;
}



bool
FFmpegController::read_to (int frame, int y, void *data)
{
    if (m_frame_calculated != frame)
    {
        if (! read_frame(frame))
            return false;
    }
    memcpy (data, m_rgb_frame->data[0] + y * m_rgb_frame->linesize[0], m_stride);
    return true;
}


bool
FFmpegController::read_to (int frame, int ybegin, int yend,
                           int chbegin, int chend, void *data)
{
    if (! read_frame (frame))
        return false;

    memcpy(data, m_rgb_frame->data[0] + (ybegin * m_rgb_frame->linesize[0]),
           m_stride * (yend - ybegin));
    return true;
}



int64_t
FFmpegController::time_stamp(int frame) const
{
    int64_t timestamp = static_cast<int64_t>((static_cast<double> (frame) / (fps() * av_q2d(
            m_format_context->streams[m_index]->time_base))
        )
    );

    if (static_cast<int64_t>(m_format_context->start_time) != int64_t(AV_NOPTS_VALUE)) {
        timestamp += static_cast<int64_t>(
            static_cast<double> (
                m_format_context->start_time
            ) * AV_TIME_BASE / av_q2d(m_format_context->streams[m_index]->time_base));
    }
    return timestamp;
}



bool
FFmpegController::read_frame (int frame)
{

    // We may retry reading data because video tends to be unstable
    // when attempting to retrieve pixels. It's an odd problem but a known
    // one to the video world
    int retry = 100000;
    bool await = false;
    int last_seeked = -1;

    if ( frame != m_decode_next_frame_out )
    {

        last_seeked = frame;
        m_decode_next_frame_in = -1;
        m_decode_next_frame_out = -1;
        m_decode_latency = 0;
        await = true;

        avcodec_flush_buffers (m_codec_context);
        if (! seek (frame))
            return false;
    }

    av_init_packet(&m_avPacket); // Initialize our packet. This guy will do a lot for us.

    bool has_picture = false;
    while (! has_picture)
    {

        bool decode_attempted = false;
        int frame_decoded = 0;

        if (m_decode_next_frame_in < m_frames)
        {

            m_error_code = av_read_frame (m_format_context, &m_avPacket);
            if (m_error_code < 0)
            {
                m_error_string = "FFmpegInput: Couldn't read frame. (has_picture)";
                break;
            }

            // Make sure that we belong to the proper stream before going any
            // further. If not, keep looping.
            if (m_avPacket.stream_index == m_index)
            {
                if (m_avPacket.pts != (int64_t)AV_NOPTS_VALUE)
                    m_pts_seen = true;

                if (last_seeked >= 0)
                {
                    int seeked_frame = -1;
                    if (m_avPacket.*m_timestamp_field == (int64_t)AV_NOPTS_VALUE ||
                        (seeked_frame = pts_to_frame (m_avPacket.*m_timestamp_field)) > last_seeked)
                    {

                        if (--last_seeked < 0)
                        {
                            // We have landed at a position we can't seek back from.
                            if (m_timestamp_field == &AVPacket::pts && m_pts_seen)
                            {
                                m_timestamp_field = &AVPacket::dts;
                                last_seeked = frame;
                            }
                            else
                            {
                                m_error_code = 1;
                                m_error_string = "FFmpegInput: Corrupted File";
                                break;
                            }
                        }

                        avcodec_flush_buffers (m_codec_context);
                        if (! seek (last_seeked))
                            break;
                    }
                    else
                    {
                        m_decode_next_frame_out = m_decode_next_frame_in = seeked_frame;
                        last_seeked = -1;
                    }
                }
                // Once we've seeked, go into the decoding process
                if (last_seeked < 0)
                {
                    ++m_decode_next_frame_in;
                    decode_attempted = true;
                    m_error_code = receive_frame (m_codec_context, m_avframe, &m_avPacket, &frame_decoded);
                    if (m_error_code < 0)
                    {
                        m_error_string = "FFmpegInput: Couldn't receive frame";
                        break;
                    }
                }
            } // The stream found wasn't this controllers stream.
        }
        else // The decode next wasn't in the frames found via our video.
        {
            decode_attempted = true;
            m_error_code = receive_frame (m_codec_context, m_avframe, &m_avPacket, &frame_decoded);
            if (m_error_code < 0)
            {
                m_error_string = "FFmpegInput: Couldn't receive frame (l).";
                break;
            }
        }

        if (frame_decoded >= 0)
        {
            // In the event we have our frame decoded, we should be able to just
            // press on.
            await = false;
            if (m_decode_next_frame_out == frame)
            {
                avpicture_fill (
                    reinterpret_cast<AVPicture*>(m_rgb_frame),
                    &m_rgb_buffer[0],
                    m_dst_pix_format,
                    m_width,
                    m_height
                );

                sws_scale
                (
                    convert_context(),
                    static_cast<uint8_t const * const *> (m_avframe->data),
                    m_avframe->linesize,
                    0,
                    m_codec_context->height,
                    m_rgb_frame->data,
                    m_rgb_frame->linesize
                );

                has_picture = true;
            }

            ++m_decode_next_frame_out;
        }
        else if (decode_attempted)
        {
            // We've tried decoding but it was unable to produce a resulting frame
            // of data. In order to account for _those_ files that may let this happen
            // we try again, otherwise chalk it up to a retry (If we have any left)
            ++m_decode_latency;
            if (m_decode_latency > codec_delay())
            {
                int seek_target = -1;
                if (await)
                {
                    if (m_decode_next_frame_out > 0)
                        seek_target = m_decode_next_frame_out - 1; // We can step back one still.
                }
                else if (retry > 0)
                {
                    // We'll give it another go.
                    retry--;
                    seek_target = frame;
                }
                else
                {
                    m_error_string = "FFmpegInput: Error decoding and no longer able to process.";
                    break;
                }

                // Try again, if we have a retry left, otherwise, time to kick the bucket.
                last_seeked = seek_target;
                m_decode_next_frame_in = -1;
                m_decode_next_frame_out = -1;
                m_decode_latency = 0;
                await = true;

                avcodec_flush_buffers (m_codec_context);
                if (! seek (seek_target))
                    break;
            }
        }

        av_free_packet(&m_avPacket);

    } // while (! has_picture) loop

    if (! has_picture)
    {
        if (m_avPacket.size > 0)
            av_free_packet(&m_avPacket);
        m_decode_next_frame_out = -1;
    }
    else
    {
        m_frame_calculated = frame;
    }

    return has_picture;
}

OIIO_NAMESPACE_END
