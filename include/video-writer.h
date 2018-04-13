/// \author Tim Quelch

#ifndef MAZES_VIDEO_WRITER_H
#define MAZES_VIDEO_WRITER_H

#include <chrono>
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

    /// Describes what type of maze tile.
    enum class Tile { wall, passage, visited, discovered, path };

    /// Simple struct to store a colour
    struct Colour {
        unsigned char r; ///< Red value
        unsigned char g; ///< Green value
        unsigned char b; ///< Blue value
    };

    /// Colour of the walls of the maze
    auto constexpr wallColour = Colour{0, 0, 0};
    /// Colour of the passages of the maze
    auto constexpr passageColour = Colour{255, 255, 255};
    /// Colour of the visited tiles
    auto constexpr visitedColour = Colour{255, 0, 0};
    /// Final colour of the gradient of the visible tiles
    auto constexpr visitedFinalColour = Colour{127, 255, 0};
    /// Colour of discovered nodes
    auto constexpr discoveredColour = Colour{0, 255, 0};
    /// Colour of the final path
    auto constexpr pathColour = Colour{0, 0, 255};

    /// Get the colour for a tile
    /// \param tile The tile to get the corresponding colour for
    /// \returns The colour of the tile
    constexpr Colour tileToColour(Tile tile);
    /// Get the colour for a tile
    /// \param tile The tile to get the corresponding colour for
    /// \param gradient The current gradient. 0 < gradient < 1
    /// \returns The colour of the tile
    constexpr Colour tileToColour(Tile tile, double gradient);

    /// Writes stages of the solution of the maze to a video file.
    class VideoWriter {
    public:
        /// Construct with specified values.
        /// \param maze The maze to write to the video file
        /// \param filename The file to write the video to
        /// \param frameRate The framerate of the produced video
        /// \param pixelsPerTile The number of pixels per maze tile
        /// \param nUpdatesPerFrame The number of updates per video frame
        /// \param gradientRate The rate to change the gradient of the path. Should be <<1
        VideoWriter(Maze const& maze,
                    std::string_view filename,
                    unsigned frameRate,
                    unsigned pixelsPerTile,
                    unsigned nUpdatesPerFrame,
                    double gradientRate);

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

        /// If the specified number of updates have passed, write the frame to file
        void writeUpdate();

        /// Write the current frame to the file
        void writeFrame();

        /// Write the current frame for a specifed timeframe
        /// \tparam Duration a std::chrono duration type
        /// \param duration the duration of time to freeze the frame for
        template <typename Duration>
        void writeFreezeFrame(Duration duration) {
            const auto frames =
                std::chrono::duration_cast<std::chrono::seconds>(duration * frameRate_).count();
            for (auto i = 0; i < frames; i++) {
                writeFrame();
            }
        }

    private:
        AVFormatContext* outContext_;   ///< The format context
        AVCodecContext* codecContext_;  ///< The encoding context
        AVStream* stream_;              ///< The file stream
        AVFrame* frame_;                ///< The picture frame
        AVFrame* rgbFrame_;             ///< The RGB picture frame
        AVPacket* packet_;              ///< The stream packet
        struct SwsContext* swsContext_; ///< Conversion context

        unsigned frameRate_{60};       ///< The frame rate of the video
        unsigned frameCounter_{0};     ///< The current frame
        unsigned nUpdatesPerFrame_{2}; ///< The number of updates per frame

        unsigned gradientCounter_{0}; ///< Counter to determine the current gradient
        double gradientRate_{0.0};    ///< The rate of change of the gradient [0, 1]
    };
} // namespace mazes

#endif
