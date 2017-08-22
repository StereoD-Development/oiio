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

#include "libffmpeg/ffmpegoiiobase.h"
#include "libffmpeg/ffmpegcontroller.h"

OIIO_PLUGIN_NAMESPACE_BEGIN

class FFmpegInput final : public ImageInput {

public:

    FFmpegInput ();
    virtual ~FFmpegInput ();
    virtual const char *format_name (void) const { return "ffmpeg"; }
    virtual bool open (const std::string &name, ImageSpec &spec);
    virtual bool open (char *buffer, size_t size, ImageSpec &newspec);
    virtual bool close (void);
    virtual int current_subimage (void) const { return m_subimage; }
    virtual bool seek_subimage (int subimage, int miplevel, ImageSpec &newspec);
    virtual bool read_native_scanline (int y, int z, void *data);
    virtual bool read_native_scanlines (int ybegin, int yend, int z,
                                        int chbegin, int chend, void *data);

private:

    /// We house most of the nitty-gritty functionality
    /// within this to keep the plugin itself more lightweight.
    FFmpegController* m_controller;

    int64_t m_nsubimages;   ///< For video files, we use this to denote frame count
    int m_subimage;

    void init (void) {
        m_controller = 0;
    }

};



// Obligatory material to make this a recognizeable imageio plugin
OIIO_PLUGIN_EXPORTS_BEGIN

    OIIO_EXPORT int ffmpeg_imageio_version = OIIO_PLUGIN_VERSION;
    OIIO_EXPORT const char* ffmpeg_imageio_library_version () {
        return "FFMpeg " LIBAVFORMAT_IDENT;
    }
    OIIO_EXPORT ImageInput *ffmpeg_input_imageio_create () {
        return new FFmpegInput;
    }
    // FFmpeg hints:
    // AVI (Audio Video Interleaved)
    // QuickTime / MOV
    // raw MPEG-4 video
    // MPEG-1 Systems / MPEG program stream
    OIIO_EXPORT const char *ffmpeg_input_extensions[] = {
        "avi", "mov", "qt", "mp4", "m4a", "3gp", "3g2", "mj2", "m4v", "mpg", NULL
    };


OIIO_PLUGIN_EXPORTS_END



FFmpegInput::FFmpegInput ()
{
    init();
}



FFmpegInput::~FFmpegInput()
{
    close();
}



bool
FFmpegInput::open (const std::string &name, ImageSpec &spec)
{
    m_controller = new FFmpegController();

    // Temporary workaround: refuse to open a file whose name does not
    // indicate that it's a movie file. This avoids the problem that ffmpeg
    // is willing to open tiff and other files better handled by other
    // plugins. The better long-term solution is to replace av_register_all
    // with our own function that registers only the formats that we want
    // this reader to handle. At some point, we will institute that superior
    // approach, but in the mean time, this is a quick solution that 90%
    // does the job.
    bool valid_extension = false;
    for (int i = 0; ffmpeg_input_extensions[i]; ++i)
        if (Strutil::ends_with (name, ffmpeg_input_extensions[i])) {
            valid_extension = true;
            break;
        }
    if (! valid_extension) {
        error ("\"%s\" could not open input", name);
        return false;
    }

    static boost::once_flag init_flag = BOOST_ONCE_INIT;
    boost::call_once (&av_register_all, init_flag);
    const char *file_name = name.c_str();
    av_log_set_level (AV_LOG_FATAL);
    // avformat_open_input() allocs format_context
    if (avformat_open_input (&(m_controller->m_format_context), file_name, NULL, NULL) != 0)
    {
        error ("\"%s\" could not open input", file_name);
        return false;
    }

    m_nsubimages = m_controller->open (file_name, spec, this);
    if (m_nsubimages < 0)
    {
        close ();
        return false;
    }
    m_spec = spec;
    return true;
}



bool
FFmpegInput::open (char *buffer, size_t size, ImageSpec &newspec)
{
    m_controller = new FFmpegController();

    // For the time being, we'll just assume this buffer should be
    // open with FFMpeg. Applications should use the format_name
    // in order to avoid any confusion in plugin useage.

    m_nsubimages = m_controller->open (buffer, size, newspec, this);
    if (m_nsubimages < 0)
    {
        close ();
        return false;
    }
    m_spec = newspec;
    return true;
}



bool
FFmpegInput::seek_subimage (int subimage, int miplevel, ImageSpec &newspec)
{
    if (subimage < 0 || subimage >= m_nsubimages || miplevel > 0) {
        return false;
    }
    if (subimage == m_subimage) {
        newspec = m_spec;
        return true;
    }
    newspec = m_spec;
    m_subimage = subimage;
    return true;
}



bool
FFmpegInput::read_native_scanline (int y, int z, void *data)
{
    return m_controller->read_to (m_subimage, y, data);
}



bool
FFmpegInput::read_native_scanlines (int ybegin, int yend, int z,
                                    int chbegin, int chend, void *data)
{
    return m_controller->read_to (m_subimage, ybegin, yend, chbegin, chend, data);
}



bool
FFmpegInput::close (void)
{
    if (m_controller)
        delete (m_controller);
    init ();
    return true;
}

OIIO_PLUGIN_NAMESPACE_END
