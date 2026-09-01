//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "cache_index.h"
#include "livox_lidar_def.h"

namespace livox_ros {

CacheIndex::CacheIndex() {
  std::array<bool, kMaxLidarCount> index_cache = {0};
  index_cache_.swap(index_cache);
}

int8_t CacheIndex::GetFreeIndex(const uint8_t livox_lidar_type, const uint32_t handle, uint8_t& index) {
  std::string key;
  int8_t ret = GenerateIndexKey(livox_lidar_type, handle, key);
  if (ret != 0) {
    return -1;
  }
  std::lock_guard<std::mutex> lock(index_mutex_);
  auto it = map_index_.find(key);
  if (it != map_index_.end()) {
    index = it->second;
    return 0;
  }

  {
    RCLCPP_INFO(DRIVER_LOGGER, "GetFreeIndex key:%s.", key.c_str());
    for (size_t i = 0; i < index_cache_.size(); ++i) {
      if (!index_cache_[i]) {
        index_cache_[i] = 1;
        map_index_[key] = static_cast<uint8_t>(i);
        index = static_cast<uint8_t>(i);
        return 0;
      }
    }
  }
  return -1;
}

int8_t CacheIndex::GenerateIndexKey(const uint8_t livox_lidar_type, const uint32_t handle, std::string& key) {
  if (livox_lidar_type == kLivoxLidarType) {
    key = "livox_lidar_" + std::to_string(handle);
  } else {
    RCLCPP_ERROR(DRIVER_LOGGER, "Can not generate index, the livox lidar type is unknown, the livox lidar type:%u", livox_lidar_type);
    return -1;
  }
  return 0;
}

int8_t CacheIndex::GetIndex(const uint8_t livox_lidar_type, const uint32_t handle, uint8_t& index) {
  std::string key;
  int8_t ret = GenerateIndexKey(livox_lidar_type, handle, key);
  if (ret != 0) {
    return -1;
  }

  std::lock_guard<std::mutex> lock(index_mutex_);
  auto it = map_index_.find(key);
  if (it != map_index_.end()) {
    index = it->second;
    return 0;
  }
  RCLCPP_ERROR(DRIVER_LOGGER, "Can not get index, the livox lidar type:%u, handle:%u", livox_lidar_type, handle);
  return -1;
}

void CacheIndex::ResetIndex(LidarDevice *lidar) {
  std::string key;
  int8_t ret = GenerateIndexKey(lidar->lidar_type, lidar->handle, key);
  if (ret != 0) {
    RCLCPP_ERROR(DRIVER_LOGGER, "Reset index failed, can not generate index key, lidar type:%u, handle:%u.", lidar->lidar_type, lidar->handle);
    return;
  }

  std::lock_guard<std::mutex> lock(index_mutex_);
  auto it = map_index_.find(key);
  if (it != map_index_.end()) {
    index_cache_[it->second] = 0;
    map_index_.erase(it);
  }
}

} // namespace
