#pragma once
#include <yaml-cpp/yaml.h>
#include <gtsam/base/Vector.h>  
struct NoiseConfig {
  double gravity;
  double accel_noise_sigma;
  double gyro_noise_sigma;
  double accel_bias_rw_sigma;
  double gyro_bias_rw_sigma;
  double integration_error_cov;
  double bias_acc_omega_init;
  double sample_rate;
};

struct PriorNoiseConfig {
  gtsam::Vector6 pose_sigma;      
  gtsam::Vector3 velocity_sigma;  
  gtsam::Vector6 bias_sigma;     
};

struct NoiseModels {
  PriorNoiseConfig prior;
  gtsam::Vector   motion_model_sigma; 
  gtsam::Vector3  dvl_sigma;
  gtsam::Vector3  gps_sigma;
  double   barometer_sigma;     
};

struct ExtrinsicsConfig {
  gtsam::Vector3 imu_sensor_offset;
  gtsam::Vector3 sbg_sensor_offset;
  gtsam::Vector3 dvl_sensor_offset;
  gtsam::Vector3 gps_sensor_offset;
  gtsam::Vector3 baro_sensor_offset;
};

inline gtsam::Vector3 readVector3(const YAML::Node& node) {
  auto d = node.as<std::vector<double>>();
  gtsam::Vector3 v;
  v << d[0], d[1], d[2];
  return v;
}

inline gtsam::Vector readVector(const YAML::Node& node) {
  std::vector<double> tmp = node.as<std::vector<double>>();
  gtsam::Vector v(tmp.size());
  for(size_t i=0; i<tmp.size(); ++i) v[i] = tmp[i];
  return v;
}

inline gtsam::Vector6 readVector6(const YAML::Node& node) {
  auto tmp = node.as<std::vector<double>>();
  gtsam::Vector6 v;
  for (int i = 0; i < 6; ++i) v(i) = tmp[i];
  return v;
}

struct Config {
  NoiseConfig     imu;
  NoiseConfig     sbg;
  ExtrinsicsConfig extrinsics;
  NoiseModels     noise_models;

  static Config load(const std::string& filename) {
    YAML::Node root = YAML::LoadFile(filename);
    Config c;

    // imu
    auto imu = root["imu"];
    c.imu.gravity               = imu["gravity"].as<double>();
    c.imu.accel_noise_sigma     = imu["accel_noise_sigma"].as<double>();
    c.imu.gyro_noise_sigma      = imu["gyro_noise_sigma"].as<double>();
    c.imu.accel_bias_rw_sigma   = imu["accel_bias_rw_sigma"].as<double>();
    c.imu.gyro_bias_rw_sigma    = imu["gyro_bias_rw_sigma"].as<double>();
    c.imu.integration_error_cov = imu["integration_error_cov"].as<double>();
    c.imu.bias_acc_omega_init   = imu["bias_acc_omega_init"].as<double>();
    c.imu.sample_rate           = imu["sample_rate"].as<double>();

    // sbg 
    auto sbg = root["sbg"];
    c.sbg.gravity               = sbg["gravity"].as<double>();
    c.sbg.accel_noise_sigma     = sbg["accel_noise_sigma"].as<double>();
    c.sbg.gyro_noise_sigma      = sbg["gyro_noise_sigma"].as<double>();
    c.sbg.accel_bias_rw_sigma   = sbg["accel_bias_rw_sigma"].as<double>();
    c.sbg.gyro_bias_rw_sigma    = sbg["gyro_bias_rw_sigma"].as<double>();
    c.sbg.integration_error_cov = sbg["integration_error_cov"].as<double>();
    c.sbg.bias_acc_omega_init   = sbg["bias_acc_omega_init"].as<double>();
    c.sbg.sample_rate           = sbg["sample_rate"].as<double>();

    // extrinsics
     auto ext = root["extrinsics"];
    c.extrinsics.imu_sensor_offset  = readVector3(ext["imu_sensor_pose"]);
    c.extrinsics.sbg_sensor_offset  = readVector3(ext["sbg_sensor_pose"]);
    c.extrinsics.dvl_sensor_offset  = readVector3(ext["dvl_sensor_pose"]);
    c.extrinsics.gps_sensor_offset  = readVector3(ext["gps_sensor_pose"]);
    c.extrinsics.baro_sensor_offset = readVector3(ext["baro_sensor_pose"]);

    // noise_models
    auto nm = root["noise_models"];
    auto prior = nm["prior"];
    c.noise_models.prior.pose_sigma     = readVector6(prior["pose_sigma"]);
    c.noise_models.prior.velocity_sigma = readVector3(prior["velocity_sigma"]);
    c.noise_models.prior.bias_sigma     = readVector6(prior["bias_sigma"]);

    c.noise_models.motion_model_sigma = readVector(nm["motion_model_sigma"]);
    c.noise_models.dvl_sigma          = readVector3(nm["dvl_sigma"]);
    c.noise_models.gps_sigma          = readVector3(nm["gps_sigma"]);
    c.noise_models.barometer_sigma    = nm["barometer_sigma"][0].as<double>();

    return c;
  }
};
