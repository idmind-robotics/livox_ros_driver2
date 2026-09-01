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

#include "livox_lidar_callback.h"

#include "livox_lidar_api.h"
#include <string>
#include <thread>
#include <iostream>

namespace livox_ros {

void LivoxLidarCallback::LidarInfoChangeCallback(const uint32_t handle,
                                           const LivoxLidarInfo* info,
                                           void* client_data) {
  if (client_data == nullptr) {
    RCLCPP_ERROR_STREAM(DRIVER_LOGGER, "lidar info change callback failed, client data is nullptr");
    return;
  }
  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);

  LidarDevice* lidar_device = GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    RCLCPP_INFO_STREAM(DRIVER_LOGGER, "found lidar not defined in the user-defined config, ip: " << IpNumToString(handle));
    // add lidar device
    uint8_t index = 0;
    int8_t ret = lds_lidar->cache_index_.GetFreeIndex(kLivoxLidarType, handle, index);
    if (ret != 0) {
      RCLCPP_ERROR_STREAM(DRIVER_LOGGER, "failed to add lidar device, lidar ip: " << IpNumToString(handle));
      return;
    }
    LidarDevice *p_lidar = &(lds_lidar->lidars_[index]);
    p_lidar->lidar_type = kLivoxLidarType;
    p_lidar->handle = handle;
    // Nothing to configure for a lidar we have no config for: -1 means "leave
    // as-is" for every setting, and the extrinsic stays identity. Mark it
    // sampling, otherwise Lddc skips it and the device is discovered but never
    // published.
    p_lidar->livox_config = UserLivoxLidarConfig{};
    p_lidar->livox_config.handle = handle;
    p_lidar->livox_config.pcl_data_type = -1;
    p_lidar->livox_config.pattern_mode = -1;
    p_lidar->livox_config.blind_spot_set = -1;
    p_lidar->livox_config.dual_emit_en = -1;
    p_lidar->connect_state = kConnectStateSampling;
  } else {
    // set the lidar according to the user-defined config
    const UserLivoxLidarConfig& config = lidar_device->livox_config;

    // lock for modify the lidar device set_bits
    {
      std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
      if (config.pcl_data_type != -1 ) {
        lidar_device->livox_config.set_bits |= kConfigDataType;
        SetLivoxLidarPclDataType(handle, static_cast<LivoxLidarPointDataType>(config.pcl_data_type),
                                LivoxLidarCallback::SetDataTypeCallback, lds_lidar);
        RCLCPP_INFO_STREAM(DRIVER_LOGGER, "set pcl data type, handle: " << handle << ", data type: "
                  << static_cast<int32_t>(config.pcl_data_type));
      }
      if (config.pattern_mode != -1) {
        lidar_device->livox_config.set_bits |= kConfigScanPattern;
        SetLivoxLidarScanPattern(handle, static_cast<LivoxLidarScanPattern>(config.pattern_mode),
                              LivoxLidarCallback::SetPatternModeCallback, lds_lidar);
        RCLCPP_INFO_STREAM(DRIVER_LOGGER, "set scan pattern, handle: " << handle << ", scan pattern: "
                  << static_cast<int32_t>(config.pattern_mode));
      }
      if (config.blind_spot_set != -1) {
        lidar_device->livox_config.set_bits |= kConfigBlindSpot;
        SetLivoxLidarBlindSpot(handle, config.blind_spot_set,
                              LivoxLidarCallback::SetBlindSpotCallback, lds_lidar);

        RCLCPP_INFO_STREAM(DRIVER_LOGGER, "set blind spot, handle: " << handle << ", blind spot distance: "
                  << config.blind_spot_set);
      }
      if (config.dual_emit_en != -1) {
        lidar_device->livox_config.set_bits |= kConfigDualEmit;
        SetLivoxLidarDualEmit(handle, (config.dual_emit_en == 0 ? false : true),
                              LivoxLidarCallback::SetDualEmitCallback, lds_lidar);
        RCLCPP_INFO_STREAM(DRIVER_LOGGER, "set dual emit mode, handle: " << handle << ", enable dual emit: "
                  << static_cast<int32_t>(config.dual_emit_en));
      }
      if (!lidar_device->livox_config.set_bits) {
        lidar_device->connect_state = kConnectStateSampling;
      }
    } // free lock for set_bits

    // The driver applies 'extrinsic_parameter' itself, in LidarPubHandler. The
    // install attitude is stored on the device and would otherwise be applied a
    // second time, on top of ours, so zero it explicitly. Zeroing rather than
    // leaving it alone also clears whatever a previous run wrote.
    LivoxLidarInstallAttitude attitude {0.0f, 0.0f, 0.0f, 0, 0, 0};
    SetLivoxLidarInstallAttitude(config.handle, &attitude,
                                 LivoxLidarCallback::SetAttitudeCallback, lds_lidar);
  }

  RCLCPP_INFO_STREAM(DRIVER_LOGGER, "begin to change work mode to 'Normal', handle: " << handle);
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeChangedCallback, nullptr);
  EnableLivoxLidarImuData(handle, LivoxLidarCallback::EnableLivoxLidarImuDataCallback, lds_lidar);
  return;
}

void LivoxLidarCallback::WorkModeChangedCallback(livox_status status,
                                                 uint32_t handle,
                                                 LivoxLidarAsyncControlResponse *response,
                                                 void *client_data) {
  if (status != kLivoxLidarStatusSuccess) {
    RCLCPP_ERROR_STREAM(DRIVER_LOGGER, "failed to change work mode, handle: " << handle << ", try again...");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeChangedCallback, nullptr);
    return;
  }
  RCLCPP_INFO_STREAM(DRIVER_LOGGER, "successfully change work mode, handle: " << handle);
  return;
}

void LivoxLidarCallback::SetDataTypeCallback(livox_status status, uint32_t handle,
                                             LivoxLidarAsyncControlResponse *response,
                                             void *client_data) {
  LidarDevice* lidar_device =  GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    RCLCPP_ERROR_STREAM(DRIVER_LOGGER, "failed to set data type since no lidar device found, handle: "
              << handle);
    return;
  }
  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);

  if (status == kLivoxLidarStatusSuccess) {
    std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
    lidar_device->livox_config.set_bits &= ~((uint32_t)(kConfigDataType));
    if (!lidar_device->livox_config.set_bits) {
      lidar_device->connect_state = kConnectStateSampling;
    }
    RCLCPP_INFO_STREAM(DRIVER_LOGGER, "successfully set data type, handle: " << handle
              << ", set_bit: " << lidar_device->livox_config.set_bits);
  } else if (status == kLivoxLidarStatusTimeout) {
    const UserLivoxLidarConfig& config = lidar_device->livox_config;
    SetLivoxLidarPclDataType(handle, static_cast<LivoxLidarPointDataType>(config.pcl_data_type),
                             LivoxLidarCallback::SetDataTypeCallback, client_data);
    RCLCPP_WARN_STREAM(DRIVER_LOGGER, "set data type timeout, handle: " << handle
              << ", try again...");
  } else {
    RCLCPP_ERROR_STREAM(DRIVER_LOGGER, "failed to set data type, handle: " << handle
              << ", return code: " << response->ret_code
              << ", error key: " << response->error_key);
  }
  return;
}

void LivoxLidarCallback::SetPatternModeCallback(livox_status status, uint32_t handle,
                                                LivoxLidarAsyncControlResponse *response,
                                                void *client_data) {
  LidarDevice* lidar_device =  GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    RCLCPP_ERROR_STREAM(DRIVER_LOGGER, "failed to set pattern mode since no lidar device found, handle: "
              << handle);
    return;
  }
  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);

  if (status == kLivoxLidarStatusSuccess) {
    std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
    lidar_device->livox_config.set_bits &= ~((uint32_t)(kConfigScanPattern));
    if (!lidar_device->livox_config.set_bits) {
      lidar_device->connect_state = kConnectStateSampling;
    }
    RCLCPP_INFO_STREAM(DRIVER_LOGGER, "successfully set pattern mode, handle: " << handle
              << ", set_bit: " << lidar_device->livox_config.set_bits);
  } else if (status == kLivoxLidarStatusTimeout) {
    const UserLivoxLidarConfig& config = lidar_device->livox_config;
    SetLivoxLidarScanPattern(handle, static_cast<LivoxLidarScanPattern>(config.pattern_mode),
                             LivoxLidarCallback::SetPatternModeCallback, client_data);
    RCLCPP_WARN_STREAM(DRIVER_LOGGER, "set pattern mode timeout, handle: " << handle
              << ", try again...");
  } else {
    RCLCPP_ERROR_STREAM(DRIVER_LOGGER, "failed to set pattern mode, handle: " << handle
              << ", return code: " << response->ret_code
              << ", error key: " << response->error_key);
  }
  return;
}

void LivoxLidarCallback::SetBlindSpotCallback(livox_status status, uint32_t handle,
                                              LivoxLidarAsyncControlResponse *response,
                                              void *client_data) {
  LidarDevice* lidar_device =  GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    RCLCPP_ERROR_STREAM(DRIVER_LOGGER, "failed to set blind spot since no lidar device found, handle: "
              << handle);
    return;
  }
  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);

  if (status == kLivoxLidarStatusSuccess) {
    std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
    lidar_device->livox_config.set_bits &= ~((uint32_t)(kConfigBlindSpot));
    if (!lidar_device->livox_config.set_bits) {
      lidar_device->connect_state = kConnectStateSampling;
    }
    RCLCPP_INFO_STREAM(DRIVER_LOGGER, "successfully set blind spot, handle: " << handle
              << ", set_bit: " << lidar_device->livox_config.set_bits);
  } else if (status == kLivoxLidarStatusTimeout) {
    const UserLivoxLidarConfig& config = lidar_device->livox_config;
    SetLivoxLidarBlindSpot(handle, config.blind_spot_set,
                           LivoxLidarCallback::SetBlindSpotCallback, client_data);
    RCLCPP_WARN_STREAM(DRIVER_LOGGER, "set blind spot timeout, handle: " << handle
              << ", try again...");
  } else {
    RCLCPP_ERROR_STREAM(DRIVER_LOGGER, "failed to set blind spot, handle: " << handle
              << ", return code: " << response->ret_code
              << ", error key: " << response->error_key);
  }
  return;
}

void LivoxLidarCallback::SetDualEmitCallback(livox_status status, uint32_t handle,
                                             LivoxLidarAsyncControlResponse *response,
                                             void *client_data) {
  LidarDevice* lidar_device =  GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    RCLCPP_ERROR_STREAM(DRIVER_LOGGER, "failed to set dual emit mode since no lidar device found, handle: "
              << handle);
    return;
  }

  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);
  if (status == kLivoxLidarStatusSuccess) {
    std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
    lidar_device->livox_config.set_bits &= ~((uint32_t)(kConfigDualEmit));
    if (!lidar_device->livox_config.set_bits) {
      lidar_device->connect_state = kConnectStateSampling;
    }
    RCLCPP_INFO_STREAM(DRIVER_LOGGER, "successfully set dual emit mode, handle: " << handle
              << ", set_bit: " << lidar_device->livox_config.set_bits);
  } else if (status == kLivoxLidarStatusTimeout) {
    const UserLivoxLidarConfig& config = lidar_device->livox_config;
    SetLivoxLidarDualEmit(handle, config.dual_emit_en,
                          LivoxLidarCallback::SetDualEmitCallback, client_data);
    RCLCPP_WARN_STREAM(DRIVER_LOGGER, "set dual emit mode timeout, handle: " << handle
              << ", try again...");
  } else {
    RCLCPP_ERROR_STREAM(DRIVER_LOGGER, "failed to set dual emit mode, handle: " << handle
              << ", return code: " << response->ret_code
              << ", error key: " << response->error_key);
  }
  return;
}

void LivoxLidarCallback::SetAttitudeCallback(livox_status status, uint32_t handle,
                                             LivoxLidarAsyncControlResponse *response,
                                             void *client_data) {
  LidarDevice* lidar_device =  GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    RCLCPP_ERROR_STREAM(DRIVER_LOGGER, "failed to set dual emit mode since no lidar device found, handle: "
              << handle);
    return;
  }

  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);
  if (status == kLivoxLidarStatusSuccess) {
    RCLCPP_INFO_STREAM(DRIVER_LOGGER, "successfully set lidar attitude, ip: " << IpNumToString(handle));
  } else if (status == kLivoxLidarStatusTimeout) {
    RCLCPP_WARN_STREAM(DRIVER_LOGGER, "set lidar attitude timeout, ip: " << IpNumToString(handle)
              << ", try again...");
    LivoxLidarInstallAttitude attitude {0.0f, 0.0f, 0.0f, 0, 0, 0};
    SetLivoxLidarInstallAttitude(lidar_device->livox_config.handle, &attitude,
                                 LivoxLidarCallback::SetAttitudeCallback, lds_lidar);
  } else {
    RCLCPP_ERROR_STREAM(DRIVER_LOGGER, "failed to set lidar attitude, ip: " << IpNumToString(handle));
  }
}

void LivoxLidarCallback::EnableLivoxLidarImuDataCallback(livox_status status, uint32_t handle,
                                                         LivoxLidarAsyncControlResponse *response,
                                                         void *client_data) {
  LidarDevice* lidar_device =  GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    RCLCPP_ERROR_STREAM(DRIVER_LOGGER, "failed to set pattern mode since no lidar device found, handle: "
              << handle);
    return;
  }
  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);

  if (response == nullptr) {
    RCLCPP_ERROR_STREAM(DRIVER_LOGGER, "failed to get response since no lidar IMU sensor found, handle: "
              << handle);
    return;
  }

  if (status == kLivoxLidarStatusSuccess) {
    RCLCPP_INFO_STREAM(DRIVER_LOGGER, "successfully enable Livox Lidar imu, ip: " << IpNumToString(handle));
  } else if (status == kLivoxLidarStatusTimeout) {
    RCLCPP_WARN_STREAM(DRIVER_LOGGER, "enable Livox Lidar imu timeout, ip: " << IpNumToString(handle)
              << ", try again...");
    EnableLivoxLidarImuData(handle, LivoxLidarCallback::EnableLivoxLidarImuDataCallback, lds_lidar);
  } else {
    RCLCPP_ERROR_STREAM(DRIVER_LOGGER, "failed to enable Livox Lidar imu, ip: " << IpNumToString(handle));
  }
}

LidarDevice* LivoxLidarCallback::GetLidarDevice(const uint32_t handle, void* client_data) {
  if (client_data == nullptr) {
    RCLCPP_ERROR_STREAM(DRIVER_LOGGER, "failed to get lidar device, client data is nullptr");
    return nullptr;
  }

  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);
  uint8_t index = 0;
  int8_t ret = lds_lidar->cache_index_.GetIndex(kLivoxLidarType, handle, index);
  if (ret != 0) {
    return nullptr;
  }

  return &(lds_lidar->lidars_[index]);
}

} // namespace livox_ros
