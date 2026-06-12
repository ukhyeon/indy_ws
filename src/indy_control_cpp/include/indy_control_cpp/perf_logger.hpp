#pragma once

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

class PerfCsvLogger
{
public:
  PerfCsvLogger(
    const std::string & path,
    const std::vector<std::string> & header,
    int flush_every = 30)
  : path_(path), header_(header), flush_every_(flush_every)
  {
    std::filesystem::create_directories(std::filesystem::path(path_).parent_path());

    const bool exists = std::filesystem::exists(path_);
    ofs_.open(path_, std::ios::out | std::ios::app);

    if (!exists) {
      writeHeader();
    }
  }

  ~PerfCsvLogger()
  {
    flush();
  }

  static std::chrono::steady_clock::time_point now()
  {
    return std::chrono::steady_clock::now();
  }

  static double msSince(const std::chrono::steady_clock::time_point & t0)
  {
    const auto t1 = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
  }

  static double msBetween(
    const std::chrono::steady_clock::time_point & t0,
    const std::chrono::steady_clock::time_point & t1)
  {
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
  }

  void writeRow(const std::vector<std::string> & values)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    for (size_t i = 0; i < values.size(); ++i) {
      if (i > 0) ofs_ << ",";
      ofs_ << values[i];
    }
    ofs_ << "\n";

    count_++;
    if ((count_ % flush_every_) == 0) {
      ofs_.flush();
    }
  }

  void flush()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (ofs_.is_open()) {
      ofs_.flush();
    }
  }

  template<typename T>
  static std::string toStr(const T & value)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << value;
    return oss.str();
  }

private:
  void writeHeader()
  {
    for (size_t i = 0; i < header_.size(); ++i) {
      if (i > 0) ofs_ << ",";
      ofs_ << header_[i];
    }
    ofs_ << "\n";
    ofs_.flush();
  }

private:
  std::string path_;
  std::vector<std::string> header_;
  int flush_every_{30};
  int count_{0};
  std::ofstream ofs_;
  std::mutex mutex_;
};