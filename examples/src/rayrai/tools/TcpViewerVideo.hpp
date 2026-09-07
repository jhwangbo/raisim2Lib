// Copyright (c) 2026 Raion Robotics Inc.
// All rights reserved.

#pragma once

#include <cstddef>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace raisin
{
namespace tcp_viewer
{

/** Encoder settings for a viewer recording. */
struct VideoEncoderSettings {
  int width = 0;
  int height = 0;
  double framesPerSecond = 30.0;
  /** libx264 CRF: 0 is lossless, 23 is the x264 default, 51 is the worst. */
  int quality = 20;
  std::string codec = "libx264";
  std::string pixelFormat = "yuv420p";
};

inline constexpr double kMinVideoFramesPerSecond = 1.0;
inline constexpr double kMaxVideoFramesPerSecond = 240.0;
inline constexpr int kMinVideoQuality = 0;
inline constexpr int kMaxVideoQuality = 51;

/** Number of frames needed to cover elapsed wall time at a constant frame rate.
 * Independent of render frequency and the PNG sequence's capture interval. */
size_t videoFramesDue(double elapsedSeconds, double framesPerSecond, size_t framesWritten);

/** Clamp user-entered settings into ranges ffmpeg accepts. */
VideoEncoderSettings sanitizeVideoEncoderSettings(const VideoEncoderSettings& settings);

/**
 * @brief ffmpeg arguments for encoding raw RGBA frames arriving on stdin.
 *
 * Pure function so the command line is unit-testable without ffmpeg installed.
 */
std::vector<std::string> buildRawVideoEncodeArguments(
  const VideoEncoderSettings& settings, const std::filesystem::path& output);

/**
 * @brief ffmpeg arguments for encoding an existing numbered PNG sequence.
 * @param inputPattern printf-style frame pattern, e.g. `frames/shot_%06d.png`.
 */
std::vector<std::string> buildPngSequenceEncodeArguments(
  const std::filesystem::path& inputPattern, const VideoEncoderSettings& settings,
  const std::filesystem::path& output);

/** Quote @p arguments into a command line that popen()/system() can run. */
std::string buildShellCommand(
  const std::string& executable, const std::vector<std::string>& arguments);

/**
 * @brief Locate an ffmpeg binary.
 *
 * Checks `$RAYRAI_FFMPEG` first so a user can pin a specific build, then the
 * PATH, then the usual package-manager prefixes. Returns an empty string when
 * no executable was found.
 */
std::string findFfmpegExecutable();

/** True when findFfmpegExecutable() resolves, i.e. video recording is possible. */
bool videoEncodingAvailable();

/** Frame filename produced by the viewer's PNG sequence recorder. */
std::string pngSequenceFrameName(const std::string& prefix, int index);
/** printf-style pattern matching pngSequenceFrameName(), for ffmpeg's image2 demuxer. */
std::string pngSequenceFramePattern(const std::string& prefix);

/**
 * @brief Streams raw RGBA frames into ffmpeg over a pipe.
 *
 * One process per recording. Frames must all be the size passed to open(); the
 * encoder rejects mismatched buffers rather than writing torn video.
 */
class VideoEncoder
{
 public:
  using Clock = std::chrono::steady_clock;
  VideoEncoder() = default;
  /** Injectable steady clock for deterministic capture/close regression tests. */
  explicit VideoEncoder(Clock::time_point (*clock)()) : clock_(clock) {}
  VideoEncoder(const VideoEncoder&) = delete;
  VideoEncoder& operator=(const VideoEncoder&) = delete;
  ~VideoEncoder();

  /**
   * @brief Start an encoder writing to @p output.
   * @return False (with @p status set) when ffmpeg is missing, the settings are
   *   unusable, or the pipe could not be opened.
   */
  bool open(const std::filesystem::path& output, const VideoEncoderSettings& settings,
            std::string& status);
  /** Append one top-down RGBA frame. @p byteCount must be width * height * 4. */
  bool writeFrameRgba(const unsigned char* rgba, size_t byteCount, std::string& status);
  /** Frames due on the recording's steady clock; zero means no readback is needed. */
  size_t framesDue() const;
  /** Resample live captures onto the recording clock, repeating the last frame
   * over missed intervals. close() fills the final interval from that frame. */
  bool writeTimedFrameRgba(const unsigned char* rgba, size_t byteCount, std::string& status);
  /** Finish the file. Safe to call when not open. */
  bool close(std::string& status);

  bool isOpen() const { return pipe_ != nullptr; }
  size_t frameCount() const { return frameCount_; }
  const std::filesystem::path& path() const { return path_; }
  const VideoEncoderSettings& settings() const { return settings_; }
  /** Bytes handed to ffmpeg so far; useful for progress display. */
  size_t bytesWritten() const { return bytesWritten_; }

 private:
  std::FILE* pipe_ = nullptr;
  std::filesystem::path path_;
  VideoEncoderSettings settings_;
  size_t frameCount_ = 0;
  size_t bytesWritten_ = 0;
  size_t frameBytes_ = 0;
  Clock::time_point (*clock_)() = &Clock::now;
  std::chrono::steady_clock::time_point started_{};
  std::vector<unsigned char> lastFrame_;
  bool timed_ = false;
};

/**
 * @brief Encode a directory of numbered PNG frames into a video file.
 *
 * Complements the live encoder: an interrupted recording, or one made before
 * ffmpeg was installed, can still be turned into a playable file.
 */
bool encodePngSequenceToVideo(
  const std::filesystem::path& frameDirectory, const std::string& framePrefix,
  const std::filesystem::path& output, const VideoEncoderSettings& settings,
  std::string& status);

} // namespace tcp_viewer
} // namespace raisin
