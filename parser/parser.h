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

#include <cstdint>
#include <string>

#include "google/protobuf/message.h"

#include "modules/drivers/gnss/proto/config.pb.h"
#include "modules/drivers/gnss/proto/gnss.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/heading.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"

#include "modules/drivers/gnss/util/macros.h"

namespace apollo {
namespace drivers {
namespace gnss {

// Modified by chihow
Gnss gnss_;
GnssBestPose bestpos_;
Imu imu_;
Ins ins_;
InsStat ins_stat_;
GnssEphemeris gnss_ephemeris_;
EpochObservation gnss_observation_;
Heading heading_;

// convert gps time (base on Jan 6 1980) to system time (base on Jan 1 1970)
// notice: Jan 6 1980
//
// linux shell:
// time1 = date +%s -d"Jan 6, 1980 00:00:01"
// time2 = date +%s -d"Jan 1, 1970 00:00:01"
// dif_tick = time1-time2
// 315964800 = 315993601 - 28801

#define EPOCH_AND_SYSTEM_DIFF_SECONDS 315964800

// A helper function that returns a pointer to a protobuf message of type T.
template <class T>
inline T *As(::google::protobuf::Message *message_ptr) {
  return dynamic_cast<T *>(message_ptr);
}

// An abstract class of Parser.
// One should use the create_xxx() functions to create a Parser object.
class Parser {
 public:
  // A general pointer to a protobuf message.
  using MessagePtr = ::google::protobuf::Message *;
  // Return a pointer to a NovAtel parser. The caller should take ownership.
  static Parser *CreateIns570d(const config::Config &config);

  virtual ~Parser() {}

  // Updates the parser with new data. The caller must keep the data valid until
  // GetMessage()
  // returns NONE.
  void Update(const uint8_t *data, size_t length) {
    data_ = data;
    data_end_ = data + length;
  }

  void Update(const std::string &data) {
    Update(reinterpret_cast<const uint8_t *>(data.data()), data.size());
  }

  enum class MessageType {
    NONE,
    GNSS,
    GNSS_RANGE,
    IMU,
    INS,
    INS_STAT,
    WHEEL,
    EPHEMERIDES,
    OBSERVATION,
    BDSEPHEMERIDES,
    RAWIMU,
    GPSEPHEMERIDES,
    GLOEPHEMERIDES,
    BEST_GNSS_POS,
    HEADING,
  };

  // Gets a parsed protobuf message. The caller must consume the message before
  // calling another
  // GetMessage() or Update();
  virtual void GetMessage() = 0;

 protected:
  Parser() {}

  // Point to the beginning and end of data. Do not take ownership.
  const uint8_t *data_ = nullptr;
  const uint8_t *data_end_ = nullptr;

 private:
  DISABLE_COPY_AND_ASSIGN(Parser);
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
