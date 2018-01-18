/// \author Tim Quelch

#include <cstdio>
#include <sstream>
#include <stdexcept>

#include "maze.h"
#include "video-writer.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

namespace mazes {
    namespace detail {

        /// Holds the RGB values for a pixel
        struct Pixel {
            const unsigned char r; ///< The red value of the pixel
            const unsigned char g; ///< The green value of the pixel
            const unsigned char b; ///< The blue value of the pixel

            /// Construct the pixel with specified values
            /// \param r The red value
            /// \param g The green value
            /// \param b The blue value
            Pixel(unsigned char r, unsigned char g, unsigned char b)
                : r{r}
                , g{g}
                , b{b} {}
        };

        Pixel tileToPixel(Tile tile) {
            switch (tile) {
            case Tile::wall:
                return {0, 0, 0};
                break;
            case Tile::passage:
                return {255, 255, 255};
                break;
            case Tile::visited:
                return {255, 0, 255};
                break;
            case Tile::discovered:
                return {255, 255, 0};
                break;
            }
            return {0, 0, 0};
        }

        void checkReturn(std::string const& message, int returnVal) {
            if (returnVal < 0) {
                char err[AV_ERROR_MAX_STRING_SIZE];
                av_strerror(returnVal, err, sizeof(err));
                throw std::runtime_error{message + std::string{err}};
            }
        }

        void checkAlloc(std::string_view message, void* pointer) {
            if (!pointer) {
                throw std::runtime_error{message.data()};
            }
        }

        AVFrame* allocFrame(int width, int height, AVPixelFormat format) {
            AVFrame* frame = av_frame_alloc();
            checkAlloc("Failed to allocate frame", frame);

            frame->format = format;
            frame->width = width;
            frame->height = height;

            int ret = av_frame_get_buffer(frame, 0);
            checkReturn("Could not allocate frame buffer", ret);

            return frame;
        }

        AVFormatContext* allocFormatContext(std::string_view filename) {
            AVFormatContext* format;
            avformat_alloc_output_context2(&format, NULL, NULL, filename.data());
            checkAlloc("Could not allocate output format. Possibly due to failure to deduce output "
                       "format form file name.",
                       format);
            return format;
        }

        AVStream* allocStream(AVFormatContext* outContext, int frameRate) {
            AVStream* stream = avformat_new_stream(outContext, NULL);
            checkAlloc("Could not allocate output stream", stream);
            stream->id = outContext->nb_streams - 1;
            stream->time_base = AVRational{1, frameRate};
            return stream;
        }

        AVCodecContext* allocCodecContext(AVCodec* codec) {
            AVCodecContext* context = avcodec_alloc_context3(codec);
            checkAlloc("Could not allocate codec context", context);
            return context;
        }

        AVPacket* allocPacket() {
            AVPacket* packet = av_packet_alloc();
            checkAlloc("Could not allocate packet", packet);
            return packet;
        }

        struct SwsContext* allocSwsContext(int inWidth,
                                           int inHeight,
                                           int outWidth,
                                           int outHeight,
                                           AVPixelFormat inPix,
                                           AVPixelFormat outPix) {
            struct SwsContext* swsContext =
                sws_getContext(inWidth, inHeight, inPix, outWidth, outHeight, outPix, 0, 0, 0, 0);
            checkAlloc("Could not get conversion context", swsContext);
            return swsContext;
        }

        AVCodec* getCodec(enum AVCodecID codecId) {
            std::string name{avcodec_get_name(codecId)};
            AVCodec* codec = avcodec_find_encoder(codecId);
            checkAlloc(std::string{"Could not find encoder for "} + name, codec);
            return codec;
        }

        void configureCodecContext(AVCodecContext* context,
                                   enum AVCodecID codecId,
                                   AVOutputFormat* format,
                                   int width,
                                   int height,
                                   int frameRate) {
            context->codec_id = codecId;
            context->bit_rate = 400000;
            context->width = width;
            context->height = height;
            context->time_base = AVRational{1, frameRate};
            context->gop_size = 12;
            context->pix_fmt = AV_PIX_FMT_YUV420P;
            if (context->codec_id == AV_CODEC_ID_MPEG2VIDEO) {
                context->max_b_frames = 2;
            }
            if (context->codec_id == AV_CODEC_ID_MPEG1VIDEO) {
                context->mb_decision = 2;
            }
            if (format->flags & AVFMT_GLOBALHEADER) {
                context->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
            }
        }

        void openVideo(AVCodecContext* codecContext, AVStream* stream) {
            int ret = avcodec_open2(codecContext, codecContext->codec, NULL);
            checkReturn("Could not open codec: ", ret);

            ret = avcodec_parameters_from_context(stream->codecpar, codecContext);
            checkReturn("Could not copy stream parameters: ", ret);
        }

        void writeVideoFrame(AVFormatContext* outContext,
                             AVCodecContext* codecContext,
                             AVStream* stream,
                             AVPacket* packet,
                             struct SwsContext* swsContext,
                             AVFrame* rgbFrame,
                             AVFrame* frame) {

            sws_scale(swsContext,
                      rgbFrame->data,
                      rgbFrame->linesize,
                      0,
                      rgbFrame->height,
                      frame->data,
                      frame->linesize);
            frame->pts++;
            int ret = avcodec_send_frame(codecContext, frame);
            checkReturn("Error sending frame for encoding: ", ret);

            while (ret >= 0) {
                ret = avcodec_receive_packet(codecContext, packet);
                if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                    return;
                } else if (ret < 0) {
                    checkReturn("Error encoding frame: ", ret);
                }

                av_packet_rescale_ts(packet, codecContext->time_base, stream->time_base);
                packet->stream_index = stream->index;
                int writeRet = av_interleaved_write_frame(outContext, packet);
                checkReturn("Error while writing frame: ", writeRet);
            }
        }
    } // namespace detail

    VideoWriter::VideoWriter(Maze const& maze,
                             std::string_view filename,
                             unsigned frameRate,
                             unsigned pixelsPerTile) {
        av_log_set_level(AV_LOG_QUIET);
        av_register_all();

        const int size = maze.size();
        const int vidSize = size * pixelsPerTile;
        outContext_ = detail::allocFormatContext(filename);
        enum AVCodecID codecId = outContext_->oformat->video_codec;
        AVCodec* codec = detail::getCodec(codecId);
        codecContext_ = detail::allocCodecContext(codec);
        stream_ = detail::allocStream(outContext_, frameRate);
        frame_ = detail::allocFrame(vidSize, vidSize, AV_PIX_FMT_YUV420P);
        rgbFrame_ = detail::allocFrame(size, size, AV_PIX_FMT_RGB24);
        packet_ = detail::allocPacket();
        swsContext_ = detail::allocSwsContext(
            size, size, vidSize, vidSize, AV_PIX_FMT_RGB24, AV_PIX_FMT_YUV420P);
        detail::configureCodecContext(
            codecContext_, codecId, outContext_->oformat, vidSize, vidSize, frameRate);
        detail::openVideo(codecContext_, stream_);

        av_dump_format(outContext_, 0, filename.data(), 1);

        if (!(outContext_->oformat->flags & AVFMT_NOFILE)) {
            int ret = avio_open(&outContext_->pb, filename.data(), AVIO_FLAG_WRITE);
            detail::checkReturn("Could not open file: ", ret);
        }

        int ret = avformat_write_header(outContext_, NULL);
        detail::checkReturn("Error while opening output file: ", ret);

        auto const& grid{maze.getGrid()};
        for (unsigned j = 0; j < maze.size(); j++) {
            for (unsigned i = 0; i < maze.size(); i++) {
                Tile tile;
                if (!grid[i][j]) {
                    tile = Tile::wall;
                } else {
                    tile = Tile::passage;
                }
                updateTile(i, j, tile);
            }
        }
    }

    VideoWriter::~VideoWriter() {
        av_write_trailer(outContext_);

        if (!(outContext_->oformat->flags & AVFMT_NOFILE)) {
            avio_closep(&outContext_->pb);
        }

        avformat_free_context(outContext_);
        avcodec_free_context(&codecContext_);
        av_frame_free(&frame_);
        av_frame_free(&rgbFrame_);
        av_packet_free(&packet_);
    }

    void VideoWriter::updateTile(int x, int y, Tile tile) {
        int ret = av_frame_make_writable(rgbFrame_);
        detail::checkReturn("Could not make frame writable: ", ret);

        const detail::Pixel pixel{detail::tileToPixel(tile)};
        rgbFrame_->data[0][y * rgbFrame_->linesize[0] + 3 * x + 0] = pixel.r;
        rgbFrame_->data[0][y * rgbFrame_->linesize[0] + 3 * x + 1] = pixel.g;
        rgbFrame_->data[0][y * rgbFrame_->linesize[0] + 3 * x + 2] = pixel.b;
    }

    void VideoWriter::updateLine(int x1, int y1, int x2, int y2, Tile tile) {
        if (x1 != x2 && y1 != y2) {
            throw std::invalid_argument{"Either x coordinates or y coordinates must be equal"};
        }
        int startx = std::min(x1, x2);
        int starty = std::min(y1, y2);
        int endx = std::max(x1 + 1, x2 + 1);
        int endy = std::max(y1 + 1, y2 + 1);
        for (int i = startx; i != endx; i++) {
            for (int j = starty; j != endy; j++) {
                if (!(i == x1 && j == y1)) {
                    updateTile(i, j, tile);
                }
            }
        }
    }

    void VideoWriter::writeFrame() {
        if (frameCounter_++ % nUpdatesPerFrame_ == 0) {
            detail::writeVideoFrame(
                outContext_, codecContext_, stream_, packet_, swsContext_, rgbFrame_, frame_);
        }
    }
} // namespace mazes
