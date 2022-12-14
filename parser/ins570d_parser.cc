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

// An parser for decoding binary messages from a Ins570d receiver. The following
// messages must be
// logged in order for this parser to work properly.
//
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "cyber/cyber.h"

#include "modules/drivers/gnss/parser/ins570d_messages.h"
#include "modules/drivers/gnss/parser/parser.h"
#include "modules/drivers/gnss/util/time_conversion.h"

namespace apollo {
namespace drivers {
namespace gnss {

// Anonymous namespace that contains helper constants and functions.
namespace {

constexpr size_t BUFFER_SIZE = 63;

constexpr int SECONDS_PER_WEEK = 60 * 60 * 24 * 7;

constexpr double DEG_TO_RAD = M_PI / 180.0;

constexpr float FLOAT_NAN = std::numeric_limits<float>::quiet_NaN();

// The Ins570d's orientation covariance matrix is pitch, roll, and yaw. We use
// the index array below
// to convert it to the orientation covariance matrix with order roll, pitch,
// and yaw.
constexpr int INDEX[] = {4, 3, 5, 1, 0, 2, 7, 6, 8};
static_assert(sizeof(INDEX) == 9 * sizeof(int), "Incorrect size of INDEX");

template <typename T>
constexpr bool is_zero(T value) {
  return value == static_cast<T>(0);
}


// Converts Ins570d's azimuth (north = 0, east = 90) to FLU yaw (east = 0, north
// = pi/2).
constexpr double azimuth_deg_to_yaw_rad(double azimuth) {
  return (90.0 - azimuth) * DEG_TO_RAD;
}

// A helper that fills an Point3D object (which uses the FLU frame) using RFU
// measurements.
inline void rfu_to_flu(double r, double f, double u,
                       ::apollo::common::Point3D* flu) {
  flu->set_x(f);
  flu->set_y(r);
  flu->set_z(u);
}

}  // namespace

class Ins570dParser : public Parser {
 public:
  Ins570dParser();
  explicit Ins570dParser(const config::Config& config);

  virtual void GetMessage();

  // 从字节流中截取片段（代表某个物理量的值）
  template<class T>
  T getOneValue(size_t offset_, size_t length_)
  {
      T buf_=T(0);
      for(uint16_t i=offset_,index=0;index<length_;i++,index++){
          buf_ += buffer_.at(i)<<(index*8);
          //printf("%x\n",buffer_.at(i));
      }
      return buf_;
  }

  // 16进制转十进制浮点数(有符号)
  template<class T>
  double hex2Value(T buf,double gain){
      double phyval=0;
      if(typeid(T) == typeid(uint16_t)){
          int16_t buf_i16=0;
          buf_i16 = *((int16_t*)(&buf));
          phyval = (double)buf_i16*gain;
          //printf("TYPE: [uint16_t] [%f] \n",(double)phyval);
      }else if(typeid(T) == typeid(uint32_t)){
          int32_t buf_i32=0;
          buf_i32 = *((int32_t*)(&buf));
          phyval = (double)buf_i32*gain;
      }
      return phyval;
  }


 private:
  bool check_sum();
  // The getxxx functions get ins status from message.
  void getRPY(Ins570d::INS_STATUS& tmp_);
  void getGYRO(Ins570d::INS_STATUS& tmp_);
  void getACC(Ins570d::INS_STATUS& tmp_);
  void getWGS84(Ins570d::INS_STATUS& tmp_);
  void getVEL(Ins570d::INS_STATUS& tmp_);
  void getALISTATUS(Ins570d::INS_STATUS& tmp_);
  void getLOOPDATA(Ins570d::INS_STATUS& tmp_);
  void getDATA123(double data123[]);
  void getPOSI_STD(Ins570d::INS_STATUS& tmp_);
  void getVEL_STD(Ins570d::INS_STATUS& tmp_);
  void getPOSE_STD(Ins570d::INS_STATUS& tmp_);
  void getTEMPERTURE(Ins570d::INS_STATUS& tmp_);
  void getGPSSTATUS(Ins570d::INS_STATUS& tmp_);
  void getWHEELSPEEDSTATUS(Ins570d::INS_STATUS& tmp_);


  void PrepareMessage();

  // The handle_xxx functions return whether a message is ready.
  bool HandleBestPos();

  bool HandleGnssBestpos();

  bool HandleBestVel(const Ins570d::BestVel* vel);

  bool HandleCorrImuData();

  bool HandleInsCov(const Ins570d::InsCov* cov);

  bool HandleInsPva(const Ins570d::InsPva* pva);

  // bool HandleInsPvax(const Ins570d::InsPvaX* pvax);
  bool HandleInsPvax();

  bool HandleRawImuX(const Ins570d::RawImuX* imu);

  bool HandleRawImu();

  bool HandleBdsEph(const Ins570d::BDS_Ephemeris* bds_emph);

  bool HandleGpsEph(const Ins570d::GPS_Ephemeris* gps_emph);

  bool HandleGloEph(const Ins570d::GLO_Ephemeris* glo_emph);

  bool HandleHeading();
  double gyro_scale_ = 0.0;

  double accel_scale_ = 0.0;

  float imu_measurement_span_ = 1.0f / 200.0f;
  float imu_measurement_hz_ = 200.0f;

  int imu_frame_mapping_ = 5;

  double imu_measurement_time_previous_ = -1.0;

  std::vector<uint8_t> buffer_;
  size_t total_length_ = 63;

  std::vector<uint32_t> dummy_array_;
  Ins570d::INS_STATUS ins570d_data_;

  config::ImuType imu_type_ = config::ImuType::ADIS16488;

  // -1 is an unused value.
  Ins570d::SolutionStatus solution_status_ =
      static_cast<Ins570d::SolutionStatus>(Ins570d::SolutionStatus::NONE);
  Ins570d::SolutionType position_type_ =
      static_cast<Ins570d::SolutionType>(Ins570d::SolutionType::NONE);
  Ins570d::SolutionType velocity_type_ =
      static_cast<Ins570d::SolutionType>(Ins570d::SolutionType::NONE);
  Ins570d::InsStatus ins_status_ =
      static_cast<Ins570d::InsStatus>(Ins570d::InsStatus::NONE);

};

double deg2rad(double deg_){
    return deg_*M_PI/180;
}

Parser* Parser::CreateIns570d(const config::Config& config) {
  return new Ins570dParser(config);
}

Ins570dParser::Ins570dParser() {
  buffer_.reserve(BUFFER_SIZE);
  ins_.mutable_position_covariance()->Resize(9, FLOAT_NAN);
  ins_.mutable_euler_angles_covariance()->Resize(9, FLOAT_NAN);
  ins_.mutable_linear_velocity_covariance()->Resize(9, FLOAT_NAN);
}

Ins570dParser::Ins570dParser(const config::Config& config) {
  buffer_.reserve(BUFFER_SIZE);
  ins_.mutable_position_covariance()->Resize(9, FLOAT_NAN);
  ins_.mutable_euler_angles_covariance()->Resize(9, FLOAT_NAN);
  ins_.mutable_linear_velocity_covariance()->Resize(9, FLOAT_NAN);

  if (config.has_imu_type()) {
    imu_type_ = config.imu_type();
  }
}

void Ins570dParser::GetMessage() {
  if (data_ == nullptr) {
    return;
  }

  while (data_ < data_end_) {
    if (buffer_.empty()) {  // Looking for SYNC0
      if (*data_ == Ins570d::SYNC_0)
        buffer_.push_back(*data_);
      ++data_;
    } else if (buffer_.size() == 1) {  // Looking for SYNC1
      if (*data_ == Ins570d::SYNC_1)
        buffer_.push_back(*data_++);
      else
        buffer_.clear();
    } else if (buffer_.size() == 2) {  // Looking for SYNC2
      if (*data_ == Ins570d::SYNC_2)
        buffer_.push_back(*data_++);
      else 
        buffer_.clear();
    } else if (total_length_ > 0) {
      if (buffer_.size() < total_length_) {  // Working on body.
        buffer_.push_back(*data_++);
        continue;
      }
      PrepareMessage();
      buffer_.clear();
    }
  }
  return ;
}

bool Ins570dParser::check_sum() {
  uint8_t checksum_ = 0;
  for(size_t i=0; i < buffer_.size()-1; i++)
      checksum_ ^= buffer_[i];
  return (checksum_ == buffer_.back());
}

void Ins570dParser::PrepareMessage() {
  if (!check_sum()) {
    AERROR << "CRC check failed.";
    return ;
  }
  // split raw data
  dummy_array_.clear();
  std::vector<uint32_t> dummy_array(27);
  for (int i=0;i<27;i++)
    dummy_array[i]=getOneValue<uint32_t>(Ins570d::config_[i][0], Ins570d::config_[i][1]);
  dummy_array_ = dummy_array;
  
  // parse data
  Ins570d::INS_STATUS *tmp_=&(this->ins570d_data_);
  getRPY(*tmp_);
  getGYRO(*tmp_);
  getACC(*tmp_);
  getWGS84(*tmp_);
  getVEL(*tmp_);
  getALISTATUS(*tmp_);
  getLOOPDATA(*tmp_);
  ins570d_data_=*tmp_;
  // debug
  ins570d_data_.debugString();

  HandleGnssBestpos();
  HandleBestPos();
  HandleCorrImuData();
  HandleRawImu();
  HandleInsPvax();
  HandleHeading();
}

void Ins570dParser::getRPY(Ins570d::INS_STATUS& tmp_)
{
    // roll
    tmp_.roll=deg2rad(hex2Value<uint16_t>(dummy_array_.at(3),ANGLE_RESOLUTION));
    tmp_.pitch=deg2rad(hex2Value<uint16_t>(dummy_array_.at(4),ANGLE_RESOLUTION));
    tmp_.yaw=deg2rad(-hex2Value<uint16_t>(dummy_array_.at(5),ANGLE_RESOLUTION));
    
}

void Ins570dParser::getGYRO(Ins570d::INS_STATUS &tmp_)
{
    tmp_.gyro_x=hex2Value<uint16_t>(dummy_array_.at(6),GYRO_RESOLUTION);
    tmp_.gyro_y=hex2Value<uint16_t>(dummy_array_.at(7),GYRO_RESOLUTION);
    tmp_.gyro_z=hex2Value<uint16_t>(dummy_array_.at(8),GYRO_RESOLUTION);
}

void Ins570dParser::getACC(Ins570d::INS_STATUS &tmp_)
{
    tmp_.acc_x=hex2Value<uint16_t>(dummy_array_.at(9),ACC_RESOLUTION);
    tmp_.acc_y=hex2Value<uint16_t>(dummy_array_.at(10),ACC_RESOLUTION);
    tmp_.acc_z=hex2Value<uint16_t>(dummy_array_.at(11),ACC_RESOLUTION);
}

void Ins570dParser::getWGS84(Ins570d::INS_STATUS &tmp_)
{
    tmp_.latitude=hex2Value<uint32_t>(dummy_array_.at(12),WGS_RESOLUTION);
    tmp_.longitude=hex2Value<uint32_t>(dummy_array_.at(13),WGS_RESOLUTION);
    tmp_.altitude=hex2Value<uint32_t>(dummy_array_.at(14),WGS_RESOLUTION);
}

void Ins570dParser::getVEL(Ins570d::INS_STATUS &tmp_)
{
    tmp_.vel_n=hex2Value<uint16_t>(dummy_array_.at(15),VEL_RESOLUTION);
    tmp_.vel_e=hex2Value<uint16_t>(dummy_array_.at(16),VEL_RESOLUTION);
    tmp_.vel_d=hex2Value<uint16_t>(dummy_array_.at(17),VEL_RESOLUTION);
}

// 4个bit都为1时，表示初始化对准完成
void Ins570dParser::getALISTATUS(Ins570d::INS_STATUS &tmp_)
{
    uint8_t status_=(uint8_t)dummy_array_.at(18);
    if(status_==0xFF){
        tmp_.ali=ALIGNED;
    }else{
        tmp_.ali=NOT_ALIGEN;
    }
}

void Ins570dParser::getLOOPDATA(Ins570d::INS_STATUS &tmp_)
{
    uint8_t type_=(uint8_t)dummy_array_.at(23);
    tmp_.data_type_=(data_type)type_;

    switch ((data_type)type_) {
    case POSI_PRECISION:getPOSI_STD(tmp_);  break;
    case VEL_PRECISION:getVEL_STD(tmp_);    break;
    case POSE_PRECISION:getPOSE_STD(tmp_);  break;
    case TEMPERATURE:getTEMPERTURE(tmp_);   break;
    case GPS_STATUS:getGPSSTATUS(tmp_);     break;
    case WHEEL_SPEED_STATUS:getWHEELSPEEDSTATUS(tmp_);break;
    default:{
        //printf("WRONG Data Type!! Please Check!\n");
        break;
    }
    }
}

void Ins570dParser::getDATA123(double data123[])
{
    data123[0]=hex2Value<uint16_t>(dummy_array_.at(19),1);
    data123[1]=hex2Value<uint16_t>(dummy_array_.at(20),1);
    data123[2]=hex2Value<uint16_t>(dummy_array_.at(21),1);
}

void Ins570dParser::getPOSI_STD(Ins570d::INS_STATUS &tmp_)
{
    double data[3];
    getDATA123(data);
    tmp_.lat_std=exp(data[0]/100);
    tmp_.lon_std=exp(data[1]/100);
    tmp_.alti_std=exp(data[2]/100);
}

void Ins570dParser::getVEL_STD(Ins570d::INS_STATUS &tmp_)
{
    double data[3];
    getDATA123(data);
    tmp_.vn_std=exp(data[0]/100);
    tmp_.ve_std=exp(data[1]/100);
    tmp_.vd_std=exp(data[2]/100);
}

void Ins570dParser::getPOSE_STD(Ins570d::INS_STATUS &tmp_)
{
    double data[3];
    getDATA123(data);
    tmp_.roll_std=exp(data[0]/100);
    tmp_.pitch_std=exp(data[1]/100);
    tmp_.yaw_std=exp(data[2]/100);
}

void Ins570dParser::getTEMPERTURE(Ins570d::INS_STATUS &tmp_)
{
    tmp_.dev_temperature=hex2Value<uint16_t>(dummy_array_.at(19),TEMPETURE_RESOLUTION);
}

void Ins570dParser::getGPSSTATUS(Ins570d::INS_STATUS &tmp_)
{
    double data[3];
    getDATA123(data);
    auto d1 = static_cast<apollo::drivers::gnss::Ins570d::SolutionType>(data[0]);
    auto d2 = static_cast<apollo::drivers::gnss::Ins570d::SolutionType>(data[2]);
    if(d1==Ins570d::SolutionType::NARROW_INT &&
       d2==Ins570d::SolutionType::NARROW_INT)
    {
        tmp_.fix_type=Ins570d::SolutionType::NARROW_INT;
    }else if(d1==Ins570d::SolutionType::NARROW_FLOAT && 
             d2==Ins570d::SolutionType::NARROW_FLOAT)
    {
        tmp_.fix_type=Ins570d::SolutionType::NARROW_FLOAT;
    }else
        tmp_.fix_type=Ins570d::SolutionType::NONE;
    tmp_.satellites_num=data[1];
}

void Ins570dParser::getWHEELSPEEDSTATUS(Ins570d::INS_STATUS &tmp_)
{
    tmp_.wheel_speed_status=hex2Value<uint16_t>(dummy_array_.at(20),1);
}

bool Ins570dParser::HandleGnssBestpos() {
  bestpos_.set_sol_status(SolutionStatus::SOL_COMPUTED);
  // bestpos_.set_sol_type(static_cast<apollo::drivers::gnss::SolutionType>
  //                       (ins570d_data_.fix_type));
  bestpos_.set_sol_type(SolutionType::NARROW_INT);
  bestpos_.set_latitude(ins570d_data_.latitude);
  bestpos_.set_longitude(ins570d_data_.longitude);
  bestpos_.set_height_msl(ins570d_data_.altitude);
  // bestpos_.set_undulation(pos->undulation);
  // bestpos_.set_datum_id(
  //     static_cast<apollo::drivers::gnss::DatumId>(pos->datum_id));
  bestpos_.set_latitude_std_dev(ins570d_data_.lat_std);
  bestpos_.set_longitude_std_dev(ins570d_data_.lon_std);
  bestpos_.set_height_std_dev(ins570d_data_.alti_std);
  // bestpos_.set_base_station_id(pos->base_station_id);
  // bestpos_.set_differential_age(pos->differential_age);
  // bestpos_.set_solution_age(pos->solution_age);
  bestpos_.set_num_sats_tracked(ins570d_data_.satellites_num);
  bestpos_.set_num_sats_in_solution(ins570d_data_.satellites_num);
  bestpos_.set_num_sats_l1(ins570d_data_.satellites_num);
  bestpos_.set_num_sats_multi(ins570d_data_.satellites_num);
  // bestpos_.set_extended_solution_status(pos->extended_solution_status);
  // bestpos_.set_galileo_beidou_used_mask(pos->galileo_beidou_used_mask);
  // bestpos_.set_gps_glonass_used_mask(pos->gps_glonass_used_mask);

  // double seconds = gps_week * SECONDS_PER_WEEK + gps_millisecs * 1e-3;
  // bestpos_.set_measurement_time(seconds);
  // AINFO << "Best gnss pose:\r\n" << bestpos_.DebugString();
  return true;
}

bool Ins570dParser::HandleBestPos() {
  gnss_.mutable_position()->set_lon(ins570d_data_.longitude);
  gnss_.mutable_position()->set_lat(ins570d_data_.latitude);
  gnss_.mutable_position()->set_height(ins570d_data_.altitude);
  gnss_.mutable_position_std_dev()->set_x(ins570d_data_.lon_std);
  gnss_.mutable_position_std_dev()->set_y(ins570d_data_.lat_std);
  gnss_.mutable_position_std_dev()->set_z(ins570d_data_.alti_std);
  gnss_.set_num_sats(ins570d_data_.satellites_num);
  solution_status_ = Ins570d::SolutionStatus::SOL_COMPUTED;
  if (position_type_ != ins570d_data_.fix_type) {
    position_type_ = ins570d_data_.fix_type;
    AINFO << "Position type: " << static_cast<int>(position_type_);
  }
  gnss_.set_solution_status(static_cast<uint32_t>(solution_status_));
  if (solution_status_ == Ins570d::SolutionStatus::SOL_COMPUTED) {
    // gnss_.set_position_type(static_cast<uint32_t>(position_type_));
    gnss_.set_position_type(SolutionType::NARROW_INT);
    switch (SolutionType::NARROW_INT) {
      case SolutionType::SINGLE:
      case SolutionType::INS_PSRSP:
        gnss_.set_type(apollo::drivers::gnss::Gnss::SINGLE);
        break;
      case SolutionType::PSRDIFF:
      case SolutionType::SBAS:
      case SolutionType::INS_SBAS:
        gnss_.set_type(apollo::drivers::gnss::Gnss::PSRDIFF);
        break;
      case SolutionType::FLOATCONV:
      case SolutionType::L1_FLOAT:
      case SolutionType::IONOFREE_FLOAT:
      case SolutionType::NARROW_FLOAT:
      case SolutionType::RTK_DIRECT_INS:
      case SolutionType::INS_RTKFLOAT:
        gnss_.set_type(apollo::drivers::gnss::Gnss::RTK_FLOAT);
        break;
      case SolutionType::WIDELANE:
      case SolutionType::NARROWLANE:
      case SolutionType::L1_INT:
      case SolutionType::WIDE_INT:
      case SolutionType::NARROW_INT:
      case SolutionType::INS_RTKFIXED:
        gnss_.set_type(apollo::drivers::gnss::Gnss::RTK_INTEGER);
        break;
      case SolutionType::OMNISTAR:
      case SolutionType::INS_OMNISTAR:
      case SolutionType::INS_OMNISTAR_HP:
      case SolutionType::INS_OMNISTAR_XP:
      case SolutionType::OMNISTAR_HP:
      case SolutionType::OMNISTAR_XP:
      case SolutionType::PPP_CONVERGING:
      case SolutionType::PPP:
      case SolutionType::INS_PPP_CONVERGING:
      case SolutionType::INS_PPP:
        gnss_.set_type(apollo::drivers::gnss::Gnss::PPP);
        break;
      case SolutionType::PROPOGATED:
        gnss_.set_type(apollo::drivers::gnss::Gnss::PROPAGATED);
        break;
      default:
        gnss_.set_type(apollo::drivers::gnss::Gnss::RTK_INTEGER);
    }
  } else {
    gnss_.set_type(apollo::drivers::gnss::Gnss::INVALID);
    gnss_.set_position_type(0);
  }
  // if (pos->datum_id != Ins570d::DatumId::WGS84) {
  //   AERROR_EVERY(5) << "Unexpected Datum Id: "
  //                   << static_cast<int>(pos->datum_id);
  // }

  // double seconds = gps_week * SECONDS_PER_WEEK + gps_millisecs * 1e-3;
  // if (gnss_.measurement_time() != seconds) {
  //   gnss_.set_measurement_time(seconds);
  //   return false;
  // }
  return true;
}

bool Ins570dParser::HandleBestVel(const Ins570d::BestVel* vel) {
  if (velocity_type_ != vel->velocity_type) {
    velocity_type_ = vel->velocity_type;
    AINFO << "Velocity type: " << static_cast<int>(velocity_type_);
  }
  if (!gnss_.has_velocity_latency() ||
      gnss_.velocity_latency() != vel->latency) {
    AINFO << "Velocity latency: " << static_cast<int>(vel->latency);
    gnss_.set_velocity_latency(vel->latency);
  }
  double yaw = azimuth_deg_to_yaw_rad(vel->track_over_ground);
  gnss_.mutable_linear_velocity()->set_x(vel->horizontal_speed * cos(yaw));
  gnss_.mutable_linear_velocity()->set_y(vel->horizontal_speed * sin(yaw));
  gnss_.mutable_linear_velocity()->set_z(vel->vertical_speed);

  // double seconds = gps_week * SECONDS_PER_WEEK + gps_millisecs * 1e-3;
  // if (gnss_.measurement_time() != seconds) {
  //   gnss_.set_measurement_time(seconds);
  //   return false;
  // }
  return true;
}

bool Ins570dParser::HandleCorrImuData() {
  ins_.mutable_euler_angles()->set_x(ins570d_data_.roll);
  ins_.mutable_euler_angles()->set_y(-ins570d_data_.pitch);
  ins_.mutable_euler_angles()->set_z(ins570d_data_.yaw);
  ins_.mutable_position()->set_lon(ins570d_data_.longitude);
  ins_.mutable_position()->set_lat(ins570d_data_.latitude);
  ins_.mutable_position()->set_height(ins570d_data_.altitude);
  ins_.mutable_linear_velocity()->set_x(ins570d_data_.vel_e);
  ins_.mutable_linear_velocity()->set_y(ins570d_data_.vel_n);
  ins_.mutable_linear_velocity()->set_z(ins570d_data_.vel_u);
  rfu_to_flu(ins570d_data_.acc_x,
             ins570d_data_.acc_y,
             ins570d_data_.acc_z,
             ins_.mutable_linear_acceleration());
  rfu_to_flu(ins570d_data_.gyro_x,
             ins570d_data_.gyro_y,
             ins570d_data_.gyro_z,
             ins_.mutable_angular_velocity());

  // ins_.set_position_covariance(
  //       i, static_cast<float>(cov->position_covariance[i]));
  //   ins_.set_euler_angles_covariance(
  //       INDEX[i], static_cast<float>((DEG_TO_RAD * DEG_TO_RAD) *
  //                                    cov->attitude_covariance[i]));
  //   ins_.set_linear_velocity_covariance(
  //       i, static_cast<float>(cov->velocity_covariance[i]));
  // }
  // double seconds = imu->gps_week * SECONDS_PER_WEEK + imu->gps_seconds;
  // if (ins_.measurement_time() != seconds) {
  //   ins_.set_measurement_time(seconds);
  //   return false;
  // }

  // ins_.mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  return true;
}

bool Ins570dParser::HandleInsCov(const Ins570d::InsCov* cov) {
  for (int i = 0; i < 9; ++i) {
    ins_.set_position_covariance(
        i, static_cast<float>(cov->position_covariance[i]));
    ins_.set_euler_angles_covariance(
        INDEX[i], static_cast<float>((DEG_TO_RAD * DEG_TO_RAD) *
                                     cov->attitude_covariance[i]));
    ins_.set_linear_velocity_covariance(
        i, static_cast<float>(cov->velocity_covariance[i]));
  }
  return false;
}

bool Ins570dParser::HandleInsPva(const Ins570d::InsPva* pva) {
  if (ins_status_ != pva->status) {
    ins_status_ = pva->status;
    AINFO << "INS status: " << static_cast<int>(ins_status_);
  }
  ins_.mutable_position()->set_lon(ins570d_data_.longitude);
  ins_.mutable_position()->set_lat(ins570d_data_.latitude);
  ins_.mutable_position()->set_height(ins570d_data_.altitude);
  // ins_.mutable_euler_angles()->set_x(pva->roll * DEG_TO_RAD);
  // ins_.mutable_euler_angles()->set_y(-pva->pitch * DEG_TO_RAD);
  // ins_.mutable_euler_angles()->set_z(azimuth_deg_to_yaw_rad(pva->azimuth));
  ins_.mutable_linear_velocity()->set_x(ins570d_data_.vel_e);
  ins_.mutable_linear_velocity()->set_y(ins570d_data_.vel_n);
  ins_.mutable_linear_velocity()->set_z(ins570d_data_.vel_u);

  switch (pva->status) {
    case Ins570d::InsStatus::ALIGNMENT_COMPLETE:
    case Ins570d::InsStatus::SOLUTION_GOOD:
      ins_.set_type(apollo::drivers::gnss::Ins::GOOD);
      break;
    case Ins570d::InsStatus::ALIGNING:
    case Ins570d::InsStatus::HIGH_VARIANCE:
    case Ins570d::InsStatus::SOLUTION_FREE:
      ins_.set_type(apollo::drivers::gnss::Ins::CONVERGING);
      break;
    default:
      ins_.set_type(apollo::drivers::gnss::Ins::INVALID);
  }

  // double seconds = pva->gps_week * SECONDS_PER_WEEK + pva->gps_seconds;
  // if (ins_.measurement_time() != seconds) {
  //   ins_.set_measurement_time(seconds);
  //   return false;
  // }

  ins_.mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  return true;
}

bool Ins570dParser::HandleInsPvax() {
  // double seconds = gps_week * SECONDS_PER_WEEK + gps_millisecs * 1e-3;
  // double unix_sec = apollo::drivers::util::gps2unix(seconds);
  // ins_stat_.mutable_header()->set_timestamp_sec(unix_sec);
  ins_stat_.set_ins_status(2);
  ins_stat_.set_pos_type(2);
  return true;
}

bool Ins570dParser::HandleRawImuX(const Ins570d::RawImuX* imu) {
  if (imu->imu_error != 0) {
    AWARN << "IMU error. Status: " << std::hex << std::showbase
          << imu->imuStatus;
  }
  if (is_zero(gyro_scale_)) {
    config::ImuType imu_type = imu_type_;
    Ins570d::ImuParameter param = Ins570d::GetImuParameter(imu_type);
    AINFO << "IMU type: " << config::ImuType_Name(imu_type) << "; "
          << "Gyro scale: " << param.gyro_scale << "; "
          << "Accel scale: " << param.accel_scale << "; "
          << "Sampling rate: " << param.sampling_rate_hz << ".";

    if (is_zero(param.sampling_rate_hz)) {
      AERROR_EVERY(5) << "Unsupported IMU type: "
                      << config::ImuType_Name(imu_type);
      return false;
    }
    gyro_scale_ = param.gyro_scale * param.sampling_rate_hz;
    accel_scale_ = param.accel_scale * param.sampling_rate_hz;
    imu_measurement_hz_ = static_cast<float>(param.sampling_rate_hz);
    imu_measurement_span_ = static_cast<float>(1.0 / param.sampling_rate_hz);
    imu_.set_measurement_span(imu_measurement_span_);
  }

  // double time = imu->gps_week * SECONDS_PER_WEEK + imu->gps_seconds;
  // if (imu_measurement_time_previous_ > 0.0 &&
  //     fabs(time - imu_measurement_time_previous_ - imu_measurement_span_) >
  //         1e-4) {
  //   AWARN_EVERY(5) << "Unexpected delay between two IMU measurements at: "
  //                  << time - imu_measurement_time_previous_;
  // }
  // imu_.set_measurement_time(time);
  switch (imu_frame_mapping_) {
    case 5:  // Default mapping.
      rfu_to_flu(imu->x_velocity_change * accel_scale_,
                 -imu->y_velocity_change_neg * accel_scale_,
                 imu->z_velocity_change * accel_scale_,
                 imu_.mutable_linear_acceleration());
      rfu_to_flu(imu->x_angle_change * gyro_scale_,
                 -imu->y_angle_change_neg * gyro_scale_,
                 imu->z_angle_change * gyro_scale_,
                 imu_.mutable_angular_velocity());
      break;
    case 6:
      rfu_to_flu(-imu->y_velocity_change_neg * accel_scale_,
                 imu->x_velocity_change * accel_scale_,
                 -imu->z_velocity_change * accel_scale_,
                 imu_.mutable_linear_acceleration());
      rfu_to_flu(-imu->y_angle_change_neg * gyro_scale_,
                 imu->x_angle_change * gyro_scale_,
                 -imu->z_angle_change * gyro_scale_,
                 imu_.mutable_angular_velocity());
      break;
    default:
      AERROR_EVERY(5) << "Unsupported IMU frame mapping: "
                      << imu_frame_mapping_;
  }
  // imu_measurement_time_previous_ = time;
  return true;
}
// bool Ins570dParser::HandleRawImu(const Ins570d::RawImu* imu) 
bool Ins570dParser::HandleRawImu() {
  // double gyro_scale = 0.0;
  // double accel_scale = 0.0;
  // float imu_measurement_span = 1.0f / 200.0f;
  rfu_to_flu(ins570d_data_.acc_x,
             ins570d_data_.acc_y,
             ins570d_data_.acc_z,
             imu_.mutable_linear_acceleration());
  rfu_to_flu(ins570d_data_.gyro_x,
             ins570d_data_.gyro_y,
             ins570d_data_.gyro_z,
             imu_.mutable_angular_velocity());
  return true;
}

bool Ins570dParser::HandleGpsEph(const Ins570d::GPS_Ephemeris* gps_emph) {
  gnss_ephemeris_.set_gnss_type(apollo::drivers::gnss::GnssType::GPS_SYS);

  apollo::drivers::gnss::KepplerOrbit* keppler_orbit =
      gnss_ephemeris_.mutable_keppler_orbit();

  keppler_orbit->set_gnss_type(apollo::drivers::gnss::GnssType::GPS_SYS);
  keppler_orbit->set_gnss_time_type(
      apollo::drivers::gnss::GnssTimeType::GPS_TIME);
  keppler_orbit->set_sat_prn(gps_emph->prn);
  keppler_orbit->set_week_num(gps_emph->week);
  keppler_orbit->set_af0(gps_emph->af0);
  keppler_orbit->set_af1(gps_emph->af1);
  keppler_orbit->set_af2(gps_emph->af2);
  keppler_orbit->set_iode(gps_emph->iode1);
  keppler_orbit->set_deltan(gps_emph->delta_A);
  keppler_orbit->set_m0(gps_emph->M_0);
  keppler_orbit->set_e(gps_emph->ecc);
  keppler_orbit->set_roota(sqrt(gps_emph->A));
  keppler_orbit->set_toe(gps_emph->toe);
  keppler_orbit->set_toc(gps_emph->toc);
  keppler_orbit->set_cic(gps_emph->cic);
  keppler_orbit->set_crc(gps_emph->crc);
  keppler_orbit->set_cis(gps_emph->cis);
  keppler_orbit->set_crs(gps_emph->crs);
  keppler_orbit->set_cuc(gps_emph->cuc);
  keppler_orbit->set_cus(gps_emph->cus);
  keppler_orbit->set_omega0(gps_emph->omega_0);
  keppler_orbit->set_omega(gps_emph->omega);
  keppler_orbit->set_i0(gps_emph->I_0);
  keppler_orbit->set_omegadot(gps_emph->dot_omega);
  keppler_orbit->set_idot(gps_emph->dot_I);
  keppler_orbit->set_accuracy(static_cast<unsigned int>(sqrt(gps_emph->ura)));
  keppler_orbit->set_health(gps_emph->health);
  keppler_orbit->set_tgd(gps_emph->tgd);
  keppler_orbit->set_iodc(gps_emph->iodc);
  return true;
}

bool Ins570dParser::HandleBdsEph(const Ins570d::BDS_Ephemeris* bds_emph) {
  gnss_ephemeris_.set_gnss_type(apollo::drivers::gnss::GnssType::BDS_SYS);

  apollo::drivers::gnss::KepplerOrbit* keppler_orbit =
      gnss_ephemeris_.mutable_keppler_orbit();

  keppler_orbit->set_gnss_type(apollo::drivers::gnss::GnssType::BDS_SYS);
  keppler_orbit->set_gnss_time_type(
      apollo::drivers::gnss::GnssTimeType::BDS_TIME);
  keppler_orbit->set_sat_prn(bds_emph->satellite_id);
  keppler_orbit->set_week_num(bds_emph->week);
  keppler_orbit->set_af0(bds_emph->a0);
  keppler_orbit->set_af1(bds_emph->a1);
  keppler_orbit->set_af2(bds_emph->a2);
  keppler_orbit->set_iode(bds_emph->aode);
  keppler_orbit->set_deltan(bds_emph->delta_N);
  keppler_orbit->set_m0(bds_emph->M0);
  keppler_orbit->set_e(bds_emph->ecc);
  keppler_orbit->set_roota(bds_emph->rootA);
  keppler_orbit->set_toe(bds_emph->toe);
  keppler_orbit->set_toc(bds_emph->toc);
  keppler_orbit->set_cic(bds_emph->cic);
  keppler_orbit->set_crc(bds_emph->crc);
  keppler_orbit->set_cis(bds_emph->cis);
  keppler_orbit->set_crs(bds_emph->crs);
  keppler_orbit->set_cuc(bds_emph->cuc);
  keppler_orbit->set_cus(bds_emph->cus);
  keppler_orbit->set_omega0(bds_emph->omega0);
  keppler_orbit->set_omega(bds_emph->omega);
  keppler_orbit->set_i0(bds_emph->inc_angle);
  keppler_orbit->set_omegadot(bds_emph->rra);
  keppler_orbit->set_idot(bds_emph->idot);
  keppler_orbit->set_accuracy(static_cast<unsigned int>(bds_emph->ura));
  keppler_orbit->set_health(bds_emph->health1);
  keppler_orbit->set_tgd(bds_emph->tdg1);
  keppler_orbit->set_iodc(bds_emph->aodc);
  return true;
}

bool Ins570dParser::HandleGloEph(const Ins570d::GLO_Ephemeris* glo_emph) {
  gnss_ephemeris_.set_gnss_type(apollo::drivers::gnss::GnssType::GLO_SYS);

  apollo::drivers::gnss::GlonassOrbit* glonass_orbit =
      gnss_ephemeris_.mutable_glonass_orbit();
  glonass_orbit->set_gnss_type(apollo::drivers::gnss::GnssType::GLO_SYS);
  glonass_orbit->set_gnss_time_type(
      apollo::drivers::gnss::GnssTimeType::GLO_TIME);
  glonass_orbit->set_slot_prn(glo_emph->sloto - 37);
  glonass_orbit->set_toe(glo_emph->e_time / 1000);
  glonass_orbit->set_frequency_no(glo_emph->freqo - 7);
  glonass_orbit->set_week_num(glo_emph->e_week);
  glonass_orbit->set_week_second_s(glo_emph->e_time / 1000);
  glonass_orbit->set_tk(glo_emph->Tk);
  glonass_orbit->set_clock_offset(-glo_emph->tau_n);
  glonass_orbit->set_clock_drift(glo_emph->gamma);

  if (glo_emph->health <= 3) {
    glonass_orbit->set_health(0);  // 0 means good.
  } else {
    glonass_orbit->set_health(1);  // 1 means bad.
  }
  glonass_orbit->set_position_x(glo_emph->pos_x);
  glonass_orbit->set_position_y(glo_emph->pos_y);
  glonass_orbit->set_position_z(glo_emph->pos_z);

  glonass_orbit->set_velocity_x(glo_emph->vel_x);
  glonass_orbit->set_velocity_y(glo_emph->vel_y);
  glonass_orbit->set_velocity_z(glo_emph->vel_z);

  glonass_orbit->set_accelerate_x(glo_emph->acc_x);
  glonass_orbit->set_accelerate_y(glo_emph->acc_y);
  glonass_orbit->set_accelerate_z(glo_emph->acc_z);

  glonass_orbit->set_infor_age(glo_emph->age);

  return true;
}

bool Ins570dParser::HandleHeading() {
  // heading_.set_solution_status(Ins570d::SolutionStatus::SOL_COMPUTED);
  heading_.set_position_type(SolutionType::NARROW_INT);
  // heading_.set_baseline_length(heading->length);
  heading_.set_heading(ins570d_data_.yaw);
  heading_.set_pitch(ins570d_data_.pitch);
  // heading_.set_reserved(heading->reserved);
  // heading_.set_heading_std_dev(heading->heading_std_dev);
  // heading_.set_pitch_std_dev(heading->pitch_std_dev);
  // heading_.set_station_id(heading->station_id);
  // heading_.set_satellite_tracked_number(heading->num_sats_tracked);
  // heading_.set_satellite_soulution_number(heading->num_sats_in_solution);
  // heading_.set_satellite_number_obs(heading->num_sats_ele);
  // heading_.set_satellite_number_multi(heading->num_sats_l2);
  // heading_.set_solution_source(heading->solution_source);
  // heading_.set_extended_solution_status(heading->extended_solution_status);
  // heading_.set_galileo_beidou_sig_mask(heading->galileo_beidou_sig_mask);
  // heading_.set_gps_glonass_sig_mask(heading->gps_glonass_sig_mask);
  // double seconds = gps_week * SECONDS_PER_WEEK + gps_millisecs * 1e-3;
  // heading_.set_measurement_time(seconds);
  return true;
}


}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
