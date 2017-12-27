/// \author Tim Quelch

#ifndef MAZES_VIDEO_WRITER_H
#define MAZES_VIDEO_WRITER_H

#include <string_view>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#include "maze.h"

namespace mazes {
    /// The default frame rate of produced videos.
    constexpr int FRAME_RATE = 100;

    /// Pixels per tile
    constexpr int PIXELS_PER_TILE = 4;

    /// Describes what type of maze tile.
    enum class Tile { wall, passage, visited, discovered };

    /// Writes stages of the solution of the maze to a video file.
    class VideoWriter {
    public:
        /// Construct with specified values.
        /// \param maze The maze to write to the video file
        /// \param filename The file to write the video to
        VideoWriter(Maze const& maze, std::string_view filename);
        ~VideoWriter();

        /// Update a tile in the frame.
        /// \param x coordinate of tile to update
        /// \param y coordinate of tile to update
        /// \param tile the type of tile to update to
        void updateTile(int x, int y, Tile tile);

        /// Write the current frame to the file
        void writeFrame();

    private:
        AVFormatContext* outContext_;   ///< The format context
        AVCodecContext* codecContext_;  ///< The encoding context
        AVStream* stream_;              ///< The file stream
        AVFrame* frame_;                ///< The picture frame
        AVFrame* rgbFrame_;             ///< The RGB picture frame
        AVPacket* packet_;              ///< The stream packet
        struct SwsContext* swsContext_; ///< Conversion context
    };
} // namespace mazes

#endif
