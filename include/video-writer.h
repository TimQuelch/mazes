/// \author Tim Quelch

#ifndef MAZES_VIDEO_WRITER_H
#define MAZES_VIDEO_WRITER_H

#include <string_view>

namespace boost::program_options {
    class options_description;
    class variables_map;
} // namespace boost::program_options
namespace po = boost::program_options;

// Forward declarations for AV lib types
struct AVFormatContext;
struct AVCodecContext;
struct AVStream;
struct AVFrame;
struct AVPacket;
struct SwsContext;

namespace mazes {
    // Forward declaration for Maze
    class Maze;

    /// Writes stages of the solution of the maze to a video file.
    class VideoWriter {
    public:
        /// Describes what type of maze tile.
        enum class Tile { wall, passage, visited, discovered };

        /// Construct with specified values.
        /// \param maze The maze to write to the video file
        /// \param filename The file to write the video to
        /// \param frameRate The framerate of the produced video
        /// \param pixelsPerTile The number of pixels per maze tile
        VideoWriter(Maze const& maze,
                    std::string_view filename,
                    unsigned frameRate,
                    unsigned pixelsPerTile);

        VideoWriter() = default;

        /// Move constructor
        /// \param other another VideoWriter
        VideoWriter(VideoWriter&& other);
        /// Move assignment operator
        /// \param other another VideoWriter
        /// \returns *this
        VideoWriter& operator=(VideoWriter&& other);
        ~VideoWriter();

        /// Update a tile in the frame.
        /// \param x coordinate of tile to update
        /// \param y coordinate of tile to update
        /// \param tile the type of tile to update to
        void updateTile(int x, int y, Tile tile);

        /// Update all tiles in a line. First coordinate (x1, y1) is not updated
        /// \param x1 First x coordinate
        /// \param y1 First y coordinate
        /// \param x2 Second x coordinate
        /// \param y2 Second y coordinate
        /// \param tile The type of tile to update to
        void updateLine(int x1, int y1, int x2, int y2, Tile tile);

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

        unsigned frameCounter_{0};     ///< The current frame
        unsigned nUpdatesPerFrame_{2}; ///< The number of updates per frame
    };
} // namespace mazes

#endif
