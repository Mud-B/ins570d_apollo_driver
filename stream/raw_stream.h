/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include "cyber/cyber.h"

// #include "modules/canbus/proto/chassis.pb.h"
#include "modules/drivers/gnss/proto/config.pb.h"
#include "modules/drivers/gnss/proto/gnss_status.pb.h"

#include "modules/drivers/gnss/parser/data_parser.h"
#include "modules/drivers/gnss/stream/stream.h"

namespace apollo {
namespace drivers {
namespace gnss {

class RawStream {
 public:
  RawStream(const config::Config& config,
            const std::shared_ptr<apollo::cyber::Node>& node);
  ~RawStream();
  bool Init();

  struct Status {
    bool filter[Stream::NUM_STATUS] = {false};
    Stream::Status status;
  };

  void Start();

 private:
  void DataSpin();
  bool Connect();
  bool Disconnect();
  void StreamStatusCheck();
  void GpsbinSpin();
  void GpsbinCallback(const std::shared_ptr<RawData const>& raw_data);


  static constexpr size_t BUFFER_SIZE = 63;
  uint8_t buffer_[BUFFER_SIZE] = {0};
  uint8_t buffer_rtk_[BUFFER_SIZE] = {0};

  std::shared_ptr<Stream> data_stream_;

  std::shared_ptr<Status> data_stream_status_;

  bool rtk_software_solution_ = false;
  bool is_healthy_ = true;
  config::Config config_;

  const std::string raw_data_topic_;

  StreamStatus stream_status_;
  std::unique_ptr<std::thread> data_thread_ptr_;
  std::unique_ptr<DataParser> data_parser_ptr_;
  std::unique_ptr<std::thread> gpsbin_thread_ptr_;
  std::unique_ptr<std::ofstream> gpsbin_stream_ = nullptr;

  std::shared_ptr<apollo::cyber::Node> node_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<StreamStatus>> stream_writer_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<RawData>> raw_writer_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<RawData>> gpsbin_reader_ = nullptr;
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
