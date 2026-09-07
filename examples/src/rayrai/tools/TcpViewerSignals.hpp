// Copyright (c) 2026 Raion Robotics Inc.
// All rights reserved.

#pragma once

#include <cstdint>
#include <ctime>
#include <deque>
#include <filesystem>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "rayrai/TcpRemoteSceneState.hpp"

namespace raisin
{
namespace tcp_viewer
{

/**
 * @brief Where a channel's data comes from, which decides when it can record.
 *
 * The protocol streams full pose data for every visual but detailed state only
 * for the one selected object, so a pinned joint channel stops advancing when
 * the selection moves elsewhere. Making that explicit keeps the UI honest
 * instead of showing a silently frozen plot.
 */
enum class SignalChannelScope
{
  /** Derived from the streamed visual pose; recordable for any object. */
  SceneWide,
  /** Needs the selected-object payload; recordable only while selected. */
  SelectionOnly
};

struct SignalChannelDesc {
  /** Stable identifier used by pins, settings, and CSV headers. */
  std::string key;
  /** Display label, including the unit. */
  std::string label;
  SignalChannelScope scope = SignalChannelScope::SceneWide;
  /** printf format for the current-value overlay. */
  std::string valueFormat = "%.3f";
};

/** Channel keys that are always available; also the default plot selection. */
inline constexpr const char* kSignalChannelLinearSpeed = "speed.linear";
inline constexpr const char* kSignalChannelAngularSpeed = "speed.angular";
inline constexpr const char* kSignalChannelGeneralizedSpeed = "speed.generalized";
inline constexpr const char* kSignalChannelContacts = "contacts";

/** Everything needed to evaluate one sample of every channel. */
struct SignalSampleInputs {
  glm::vec3 position{0.0f};
  glm::vec3 linearVelocity{0.0f};
  float angularSpeed = 0.0f;
  float contactCount = 0.0f;
  /** False when no motion estimate exists yet; velocity channels then read 0. */
  bool hasMotionEstimate = false;
  /** Selected-object payload, or nullptr when this object is not the selection. */
  const SelectedObjectInfo* selectedInfo = nullptr;
};

/**
 * @brief Per-joint generalized-velocity offsets implied by @p jointTypes.
 *
 * The protocol sends generalized-coordinate offsets but not velocity offsets, so
 * they are reconstructed from the joint types in order. Returns an empty vector
 * when the reconstruction does not add up to @p dof, so a layout this code does
 * not understand disables the joint-velocity channels instead of plotting
 * misaligned data.
 */
std::vector<int32_t> deriveJointGvOffsets(const std::vector<int32_t>& jointTypes, int32_t dof);

/**
 * @brief Channels available for one object.
 * @param isArticulated Whether the object is an articulated system.
 * @param hasContactTags Whether the server tags contacts with object ids.
 * @param selectedInfo Selected-object payload, or nullptr; supplies joint channels.
 */
std::vector<SignalChannelDesc> buildSignalChannels(
  bool isArticulated, bool hasContactTags, const SelectedObjectInfo* selectedInfo);

/** Channel keys plotted when a user has not chosen any yet. */
std::vector<std::string> defaultSignalChannelKeys(
  const std::vector<SignalChannelDesc>& channels);

/** Evaluate @p channels into one sample, in channel order. */
std::vector<float> sampleSignalChannels(
  const std::vector<SignalChannelDesc>& channels, const SignalSampleInputs& inputs);

/**
 * @brief Fixed-capacity rolling history for one object.
 *
 * Samples are stored as a row per world time over a fixed channel set. Changing
 * the channel set clears the history rather than trying to splice columns, which
 * keeps every stored row the same width and the CSV export rectangular.
 */
class SignalHistory
{
 public:
  /** Sample cap; ~600 rows keeps a long recording bounded (see tcp_checklist.md). */
  static constexpr size_t kMaxSamples = 600;

  /** Replace the channel set. Clears stored samples when the set actually changes. */
  void setChannels(std::vector<SignalChannelDesc> channels);
  const std::vector<SignalChannelDesc>& channels() const { return channels_; }

  /**
   * @brief Append one sample.
   * @return False when @p values does not match the channel count or @p worldTime
   *   is not newer than the last sample (a paused or rewound server resends times).
   */
  bool append(double worldTime, std::vector<float> values, bool selectionAvailable = true);

  void clear();
  size_t size() const { return times_.size(); }
  bool empty() const { return times_.empty(); }
  double lastTime() const;

  const std::deque<double>& times() const { return times_; }
  const std::deque<std::vector<float>>& samples() const { return samples_; }

  /** Index of @p key in the channel set, or -1. */
  int channelIndex(const std::string& key) const;
  /** Whether this row contains a measurement for the channel (not a missing payload). */
  bool sampleAvailable(size_t row, size_t column) const;
  /** Plot samples, holding the last measured value across unavailable rows.
   * Leading unavailable rows are omitted. Returns false for unknown keys. */
  bool series(const std::string& key, std::vector<float>& out) const;

 private:
  std::vector<SignalChannelDesc> channels_;
  std::deque<double> times_;
  std::deque<std::vector<float>> samples_;
  std::deque<bool> selectionAvailable_;
};

/**
 * @brief Write selected channels of @p history to a CSV file.
 *
 * Columns are `time` followed by one column per requested channel key, in the
 * order given. Unknown keys are skipped; unavailable measurements are empty cells.
 */
bool writeSignalCsv(
  const std::filesystem::path& path, const SignalHistory& history,
  const std::vector<std::string>& channelKeys, std::string& status);

/** Timestamped default name for an exported signal CSV. */
std::filesystem::path timestampedSignalCsvPath(
  const std::filesystem::path& directory, const std::string& objectName, std::time_t now);

} // namespace tcp_viewer
} // namespace raisin
