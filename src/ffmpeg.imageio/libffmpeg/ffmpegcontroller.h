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

#ifndef FFMPEGCONTROLLER_H
#define FFMPEGCONTROLLER_H

#include "ffmpegoiiobase.h"

#include <OpenImageIO/imageio.h>
#include <OpenImageIO/stream.h>
#include <iostream>
#include <memory>

OIIO_NAMESPACE_BEGIN

/// The Controller that houses various FFmpeg I/O helper functions that
/// our image plugins take advantage of
class FFmpegController
{

public:

    /// Clean Slate Constructor/Destructor
    FFmpegController();
    virtual ~FFmpegController();

    /// -----------------------------------------------------------------
    /// -- Functions \/
    /// -----------------------------------------------------------------

    /// Cracking open the file/memory and looking into it's specs
    int64_t open (const char *file_name, ImageSpec &spec, ImageInput *inp);
    int64_t open (char *buffer, size_t size, ImageSpec &spec, ImageInput *inp);

    /// Our PTS generated from our frame is the result of multiplying our
    /// time_base.den by the fount fps den and this specfic frame. Divided
    /// out our .num (den/num). AV formats love to get messy with their
    /// timing structures.
    int64_t frame_to_pts (int frame) const;
    int pts_to_frame (int64_t pts) const;

    /// Make sure we've build the SwsContext and return that.
    SwsContext* convert_context(); // May not need

    bool seek (int frame);
    double fps () const;
    int64_t time_stamp (int frame) const;

    /// Select formats require additional frames before creating a
    /// complete output.
    int codec_delay () const;

    const char *error_string () const { return m_error_string.c_str(); }

    /// -----------------------------------------------------------------
    /// -- Reading \/
    /// -----------------------------------------------------------------

    /// Move data from calclated deltas into a container.
    bool read_to (int frame, int y, void *data);
    bool read_to (int frame, int ybegin, int yend,
                  int chbegin, int chend, void *data);

    /// Read the given frame into the local buffer for data extraction
    bool read_frame (int frame);

    /// -----------------------------------------------------------------
    /// -- Members \/
    /// -----------------------------------------------------------------

    /// Stream Data Elements. Housing for most data and what our routines
    /// rely on for formating and allocation.
    int             m_index;         ///< The stream index
    AVStream*       m_avstream;      ///< The video stream itself
    AVCodecContext* m_codec_context; ///< The m_avstream codec context
    AVCodec*        m_codec;         ///< ?
    AVFrame*        m_avframe;       ///< The decoding frame
    AVFrame*        m_rgb_frame;     ///< Housing for decoded pixels (?)
    SwsContext*     m_convert_context; ///< The conversion context

    AVPacket m_avPacket; ///< Packet for sending/retrieving data from ffmpeg

    AVFormatContext* m_format_context; ///< The video context
    AVPixelFormat    m_dst_pix_format; ///< The resulting format for our pixels
    AVPixelFormat    m_src_pix_format;

    /// -----------------------------------------------------------------
    /// Time/Frame, and Size management. This is a complex system of units and
    /// types so it's best to try and manage them all in one location.
    int m_fps_num;   ///< The m_avframe.time_base.num (I think)
    int m_fps_den;   ///< The m_avframe.time_base.den (I think) // REVIST

    AVRational m_frame_rate; ///< The stored frame rate of our stream

    int64_t m_start_pts; ///< The starting PTS (Positional Time Stamp) of this stream
    int64_t m_frames;    ///< The duration of our stream in frames

    bool m_pts_seen; ///< In the event our AVPacket has contained pts over it's life,
                     ///  we have this marked so we know our stream contains PTS's
    int64_t AVPacket::* m_timestamp_field; ///< Pointer to AVPacket member (PTS/DTS)

    int m_width;     ///< Width of our stream
    int m_height;    ///< Height of our stream
    double m_aspect; ///< The aspect ratio of our stream (eughh)

    int m_decode_next_frame_in;  ///< The index of the next frame to decode.
    int m_decode_next_frame_out; ///< The index of the next frame out of decode.
    int m_decode_latency;        ///< Anti-loop detector.

    int64_t m_frame_calculated;        ///< The active frame calculated (if any)
    std::vector<uint8_t> m_rgb_buffer; ///< Housing for a rendered frame (?)
    size_t m_stride;                   ///< How far, in bytes, is a single line

    std::string m_filename;        ///< For when using a file directly
    std::unique_ptr<OIIO::no_copy_membuf> m_stream; ///< For in-memory files.
    uint8_t *m_context_buffer;     ///< The buffer for housing operations.

    std::string m_error_string; ///< The error string (if any)
    int m_error_code;           ///< The ffpmeg error number

};

OIIO_NAMESPACE_END

#endif // FFMPEGCONTROLLER_H
