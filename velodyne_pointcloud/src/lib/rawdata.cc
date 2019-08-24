/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  Velodyne 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw Velodyne LIDAR packets into useful
 *  formats.
 *
 *  Derived classes accept raw Velodyne data for either single packets
 *  or entire rotations, and provide it in various formats for either
 *  on-line or off-line processing.
 *
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 *
 *  HDL-64E S2 calibration support provided by Nick Hillier
 */

#include <math.h>
#include <fstream>

#include <angles/angles.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <velodyne_pointcloud/rawdata.h>

namespace velodyne_rawdata {
////////////////////////////////////////////////////////////////////////
//
// RawData base class implementation
//
////////////////////////////////////////////////////////////////////////

RawData::RawData()
{
}

/** Update parameters: conversions and update */
void RawData::setParameters(double min_range, double max_range, double view_direction,
                            double view_width)
{
  config_.min_range = min_range;
  config_.max_range = max_range;

  // converting angle parameters into the velodyne reference (rad)
  config_.tmp_min_angle = view_direction + view_width / 2;
  config_.tmp_max_angle = view_direction - view_width / 2;

  // computing positive modulo to keep theses angles into [0;2*M_PI]
  config_.tmp_min_angle = fmod(fmod(config_.tmp_min_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
  config_.tmp_max_angle = fmod(fmod(config_.tmp_max_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);

  // converting into the hardware velodyne ref (negative yaml and degrees)
  // adding 0.5 perfomrs a centered double to int conversion
  config_.min_angle = 100 * (2 * M_PI - config_.tmp_min_angle) * 180 / M_PI + 0.5;
  config_.max_angle = 100 * (2 * M_PI - config_.tmp_max_angle) * 180 / M_PI + 0.5;
  if (config_.min_angle == config_.max_angle) {
    // avoid returning empty cloud if min_angle = max_angle
    config_.min_angle = 0;
    config_.max_angle = 36000;
  }
}

/** Set up for on-line operation. */
int RawData::setup(ros::NodeHandle private_nh)
{
  // get path to angles.config file for this device
  if (!private_nh.getParam("calibration", config_.calibrationFile)) {
    ROS_ERROR_STREAM("No calibration angles specified! Using test values!");

    // have to use something: grab unit test version as a default
    std::string pkgPath = ros::package::getPath("velodyne_pointcloud");
    config_.calibrationFile = pkgPath + "/params/64e_utexas.yaml";
  }

  ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);

  calibration_.read(config_.calibrationFile);
  if (!calibration_.initialized) {
    ROS_ERROR_STREAM("Unable to open calibration file: " << config_.calibrationFile);
    return -1;
  }

  ROS_INFO_STREAM("Number of lasers: " << calibration_.num_lasers << ".");

  if (!private_nh.getParam("device_model", config_.deviceModel)) {
    ROS_WARN_STREAM("device_model not specified");
  }

  if (calibration_.num_lasers == 16) {
    vlp_spec_ = VLP_16_SPEC;
    is_vlp_ = true;
  } else if (config_.deviceModel == "VLP32") {
    vlp_spec_ = VLP_32_SPEC;
    is_vlp_ = true;
    // Print corrections
    std::vector<float> elev_spec = {
      -25.0,  -1.0,   -1.667, -15.639, -11.31, 0.0,    -0.667, -8.843, -7.254, 0.333,  -0.333,
      -6.148, -5.333, 1.333,  0.667,   -4.0,   -4.667, 1.667,  1.0,    -3.667, -3.333, 3.333,
      2.333,  -2.667, -3.0,   7.0,     4.667,  -2.333, -2.0,   15.0,   10.333, -1.333
    };
    std::vector<float> az_spec = { 1.4,  -4.2, 1.4,  -1.4, 1.4,  -1.4, 4.2,  -1.4, 1.4,  -4.2, 1.4,
                                   -1.4, 4.2,  -1.4, 4.2,  -1.4, 1.4,  -4.2, 1.4,  -4.2, 4.2,  -1.4,
                                   1.4,  -1.4, 1.4,  -1.4, 1.4,  -4.2, 4.2,  -1.4, 1.4,  -1.4 };
    for (int i = 0; i < 32; ++i) {
      std::printf("Laser: %d Elevation angle: %.3f, spec: %.3f. Azimuth offset: %.3f, spec: %.3f\n",
                  i, 180. / M_PI * std::asin(calibration_.laser_corrections[i].sin_vert_correction),
                  elev_spec[i],
                  180. / M_PI * std::asin(calibration_.laser_corrections[i].sin_rot_correction),
                  az_spec[i]);
    }
    // The calibration file has negative angles for azimuth correction -> w/o that output is crap...
  } else {
    is_vlp_ = false;
  }

  // Set up cached values for sin and cos of all the possible headings
  for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
    float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
    cos_rot_table_[rot_index] = cosf(rotation);
    sin_rot_table_[rot_index] = sinf(rotation);
  }
  return 0;
}

/** @brief convert raw packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
float RawData::unpack(const velodyne_msgs::VelodynePacket& pkt, VPointCloud& pc, size_t* valid_pnts) const
{
  ROS_DEBUG_STREAM("Received packet, time: " << pkt.stamp);

  /** special parsing for the VLP16 and VLP32 **/
  if (is_vlp_) {
    return unpack_vlp(pkt, pc, valid_pnts);
  }

  const raw_packet_t* raw = (const raw_packet_t*)&pkt.data[0];

  for (int i = 0; i < BLOCKS_PER_PACKET; i++) {
    // upper bank lasers are numbered [0..31]
    // NOTE: this is a change from the old velodyne_common implementation
    int bank_origin = 0;
    if (raw->blocks[i].header == LOWER_BANK) {
      // lower bank lasers are [32..63]
      bank_origin = 32;
    }

    for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {
      float x, y, z;
      float intensity;
      uint8_t laser_number; ///< hardware laser number

      laser_number = j + bank_origin;
      const velodyne_pointcloud::LaserCorrection& corrections =
          calibration_.laser_corrections.at(laser_number);

      /** Position Calculation */

      union two_bytes tmp;
      tmp.bytes[0] = raw->blocks[i].data[k];
      tmp.bytes[1] = raw->blocks[i].data[k + 1];
      /*condition added to avoid calculating points which are not
        in the interesting defined area (min_angle < area < max_angle)*/
      if ((raw->blocks[i].rotation >= config_.min_angle &&
           raw->blocks[i].rotation <= config_.max_angle && config_.min_angle < config_.max_angle) ||
          (config_.min_angle > config_.max_angle &&
           (raw->blocks[i].rotation <= config_.max_angle ||
            raw->blocks[i].rotation >= config_.min_angle))) {
        float distance = tmp.uint * DISTANCE_RESOLUTION;
        distance += corrections.dist_correction;

        float cos_vert_angle = corrections.cos_vert_correction;
        float sin_vert_angle = corrections.sin_vert_correction;
        float cos_rot_correction = corrections.cos_rot_correction;
        float sin_rot_correction = corrections.sin_rot_correction;

        // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
        // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
        float cos_rot_angle = cos_rot_table_[raw->blocks[i].rotation] * cos_rot_correction +
                              sin_rot_table_[raw->blocks[i].rotation] * sin_rot_correction;
        float sin_rot_angle = sin_rot_table_[raw->blocks[i].rotation] * cos_rot_correction -
                              cos_rot_table_[raw->blocks[i].rotation] * sin_rot_correction;

        float horiz_offset = corrections.horiz_offset_correction;
        float vert_offset = corrections.vert_offset_correction;

        // Compute the distance in the xy plane (w/o accounting for rotation)
        /**the new term of 'vert_offset * sin_vert_angle'
         * was added to the expression due to the mathemathical
         * model we used.
         */
        float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

        // Calculate temporal X, use absolute value.
        float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
        // Calculate temporal Y, use absolute value
        float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
        if (xx < 0)
          xx = -xx;
        if (yy < 0)
          yy = -yy;

        // Get 2points calibration values,Linear interpolation to get distance
        // correction for X and Y, that means distance correction use
        // different value at different distance
        float distance_corr_x = 0;
        float distance_corr_y = 0;
        if (corrections.two_pt_correction_available) {
          distance_corr_x = (corrections.dist_correction - corrections.dist_correction_x) *
                                (xx - 2.4) / (25.04 - 2.4) +
                            corrections.dist_correction_x;
          distance_corr_x -= corrections.dist_correction;
          distance_corr_y = (corrections.dist_correction - corrections.dist_correction_y) *
                                (yy - 1.93) / (25.04 - 1.93) +
                            corrections.dist_correction_y;
          distance_corr_y -= corrections.dist_correction;
        }

        float distance_x = distance + distance_corr_x;
        /**the new term of 'vert_offset * sin_vert_angle'
         * was added to the expression due to the mathemathical
         * model we used.
         */
        xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle;
        /// the expression wiht '-' is proved to be better than the one with '+'
        x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

        float distance_y = distance + distance_corr_y;
        xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle;
        /**the new term of 'vert_offset * sin_vert_angle'
         * was added to the expression due to the mathemathical
         * model we used.
         */
        y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

        // Using distance_y is not symmetric, but the velodyne manual
        // does this.
        /**the new term of 'vert_offset * cos_vert_angle'
         * was added to the expression due to the mathemathical
         * model we used.
         */
        z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle;

        /** Use standard ROS coordinate system (right-hand rule) */
        float x_coord = y;
        float y_coord = -x;
        float z_coord = z;

        /** Intensity Calculation */

        float min_intensity = corrections.min_intensity;
        float max_intensity = corrections.max_intensity;

        intensity = raw->blocks[i].data[k + 2];

        float focal_offset = 256 * (1 - corrections.focal_distance / 13100) *
                             (1 - corrections.focal_distance / 13100);
        float focal_slope = corrections.focal_slope;
        intensity += focal_slope * (abs(focal_offset -
                                        256 * (1 - static_cast<float>(tmp.uint) / 65535) *
                                            (1 - static_cast<float>(tmp.uint) / 65535)));
        intensity = (intensity < min_intensity) ? min_intensity : intensity;
        intensity = (intensity > max_intensity) ? max_intensity : intensity;

        if (pointInRange(distance)) {
          // convert polar coordinates to Euclidean XYZ
          VPoint point;
          point.ring = corrections.laser_ring;
          point.x = x_coord;
          point.y = y_coord;
          point.z = z_coord;
          point.intensity = intensity;

          // append this point to the cloud
          pc.points.push_back(point);
          ++pc.width;
        }
      }
    }
  }
  return -1.0;
}

/** @brief convert raw VLP16 and VLP32 packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
float RawData::unpack_vlp(const velodyne_msgs::VelodynePacket& pkt, VPointCloud& pc,
                          size_t* valid_pnts) const
{
  float azimuth;
  float azimuth_diff;
  float last_azimuth_diff = 0;
  float azimuth_corrected_f;
  int azimuth_corrected;
  float x, y, z;
  float intensity;
  float slice_angle = 0.0;

  if (vlp_spec_.lasers_per_firing_seq == 32) {
    ROS_INFO_THROTTLE(60, "Unpacking VLP32");
    assert(vlp_spec_.firing_seqs_per_block == 1);
  } else {
    ROS_INFO_THROTTLE(60, "Unpacking VLP16");
  }

  const raw_packet_t* raw = (const raw_packet_t*)&pkt.data[0];

  ROS_INFO_THROTTLE(60, "Return Mode: 0x%X, Product ID: 0x%X", pkt.data[1204], pkt.data[1205]);

  std::set<std::tuple<float, float, float>> pnts_set; // expensive -- just bug-hunting!!!
  size_t duplicates = 0;
  size_t num_ok = 0;
  size_t num_filtered_azimuth = 0;
  size_t num_filtered_range = 0;

  for (int block = 0; block < BLOCKS_PER_PACKET; block++) {
    // Each block has 2b flag, 2b az, 32*(2b dist, 1b reflectivity)

    // ignore packets with mangled or otherwise different contents
    if (UPPER_BANK != raw->blocks[block].header) { // b 0,1
      // Do not flood the log with messages, only issue at most one
      // of these warnings per minute.
      ROS_WARN_STREAM_THROTTLE(60, "skipping invalid VLP packet: block "
                                       << block << " header value is "
                                       << raw->blocks[block].header);
      return -1; // bad packet: skip the rest
    }

    // Calculate difference between current and next block's azimuth angle.
    azimuth = (float)(raw->blocks[block].rotation); // b 2,3

    if (block < (BLOCKS_PER_PACKET - 1)) {
      azimuth_diff =
          (float)((36000 + raw->blocks[block + 1].rotation - raw->blocks[block].rotation) % 36000);
      slice_angle += azimuth_diff;
      last_azimuth_diff = azimuth_diff;
    } else {
      azimuth_diff = last_azimuth_diff;
    }

    /* configuration for VLP32C:

       firing_seqs_per_block = 1
       lasers_per_firing_seq = 32
       lasers_per_firing     = 2
       firing_duration;      = 2.304  [us]
       firing_seq_duration   = 55.296 [us]
       block_duration        = 55.296 [us] = firing_seq_duration * firing_seqs_per_block
       distance_resolution   = 0.004  [m]

       RAW_SCAN_SIZE = 3 (bytes)
    */
    // Go trough the 32, 3 byte laser hits
    for (int firing_seq = 0, k = 0; firing_seq < vlp_spec_.firing_seqs_per_block; firing_seq++) {
      for (int laser = 0; laser < vlp_spec_.lasers_per_firing_seq; laser++, k += RAW_SCAN_SIZE) {
        const velodyne_pointcloud::LaserCorrection& corrections = calibration_.laser_corrections.at(laser);

        /** Position Calculation */
        union two_bytes tmp;
        tmp.bytes[0] = raw->blocks[block].data[k];
        tmp.bytes[1] = raw->blocks[block].data[k + 1];

        /** correct for the laser rotation as a function of timing during the firings **/
        float firing_offset = (laser / vlp_spec_.lasers_per_firing) * vlp_spec_.firing_duration;
        float firing_seq_offset = firing_seq * vlp_spec_.firing_seq_duration;
        azimuth_corrected_f = azimuth + (azimuth_diff * (firing_offset + firing_seq_offset) /
                                         vlp_spec_.block_duration);
        azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;

        /*condition added to avoid calculating points which are not
          in the interesting defined area (min_angle < area < max_angle)*/
        if ((azimuth_corrected >= config_.min_angle && azimuth_corrected <= config_.max_angle &&
             config_.min_angle < config_.max_angle) ||
            (config_.min_angle > config_.max_angle &&
             (azimuth_corrected <= config_.max_angle || azimuth_corrected >= config_.min_angle))) {
          // convert polar coordinates to Euclidean XYZ
          float distance = tmp.uint * vlp_spec_.distance_resolution;
          distance += corrections.dist_correction;

          float cos_vert_angle = corrections.cos_vert_correction;
          float sin_vert_angle = corrections.sin_vert_correction;
          float cos_rot_correction = corrections.cos_rot_correction;
          float sin_rot_correction = corrections.sin_rot_correction;

          // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
          // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
          float cos_rot_angle = cos_rot_table_[azimuth_corrected] * cos_rot_correction +
                                sin_rot_table_[azimuth_corrected] * sin_rot_correction;
          float sin_rot_angle = sin_rot_table_[azimuth_corrected] * cos_rot_correction -
                                cos_rot_table_[azimuth_corrected] * sin_rot_correction;

          float horiz_offset = corrections.horiz_offset_correction;
          float vert_offset = corrections.vert_offset_correction;

          // Compute the distance in the xy plane (w/o accounting for rotation)
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

          // Calculate temporal X, use absolute value.
          float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
          // Calculate temporal Y, use absolute value
          float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
          if (xx < 0)
            xx = -xx;
          if (yy < 0)
            yy = -yy;

          // Get 2points calibration values,Linear interpolation to get distance
          // correction for X and Y, that means distance correction use
          // different value at different distance
          float distance_corr_x = 0;
          float distance_corr_y = 0;
          if (corrections.two_pt_correction_available) {
            distance_corr_x = (corrections.dist_correction - corrections.dist_correction_x) *
                                  (xx - 2.4) / (25.04 - 2.4) +
                              corrections.dist_correction_x;
            distance_corr_x -= corrections.dist_correction;
            distance_corr_y = (corrections.dist_correction - corrections.dist_correction_y) *
                                  (yy - 1.93) / (25.04 - 1.93) +
                              corrections.dist_correction_y;
            distance_corr_y -= corrections.dist_correction;
          }

          float distance_x = distance + distance_corr_x;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle;
          x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

          float distance_y = distance + distance_corr_y;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle;
          y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

          // Using distance_y is not symmetric, but the velodyne manual
          // does this.
          /**the new term of 'vert_offset * cos_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle;


          /** Use standard ROS coordinate system (right-hand rule) */
          float x_coord = y;
          float y_coord = -x;
          float z_coord = z;

          /** Intensity Calculation */
          float min_intensity = corrections.min_intensity;
          float max_intensity = corrections.max_intensity;

          intensity = raw->blocks[block].data[k + 2];

          float focal_offset = 256 * (1 - corrections.focal_distance / 13100) *
                               (1 - corrections.focal_distance / 13100);
          float focal_slope = corrections.focal_slope;
          intensity += focal_slope *
                       (abs(focal_offset - 256 * (1 - tmp.uint / 65535) * (1 - tmp.uint / 65535)));
          intensity = (intensity < min_intensity) ? min_intensity : intensity;
          intensity = (intensity > max_intensity) ? max_intensity : intensity;

          if (pointInRange(distance)) {
            // append this point to the cloud
            VPoint point;
            point.ring = corrections.laser_ring;
            point.x = x_coord;
            point.y = y_coord;
            point.z = z_coord;
            point.intensity = intensity;

            auto pnt_tuple = std::make_tuple(point.x, point.y, point.z);
            if (pnts_set.count(pnt_tuple) == 0) {
              pnts_set.insert(pnt_tuple);
            } else {
              duplicates++;
            }

            num_ok++;
            pc.points.push_back(point);
            ++pc.width;
          } else {
            num_filtered_range++;
          }
        } else {
          num_filtered_azimuth++;
        }
      } // end for laser
    }   // end for firing_seq
  }     // end for block

  if (valid_pnts != nullptr) {
    *valid_pnts = num_ok;
  }
  //  std::printf("%lu OK pnts in PKT, filtered: AZ: %lu RNG: %lu, duplicates: %lu, total=%lu, "
  //              "slice now has %lu pnts\n",
  //              num_ok, num_filtered_azimuth, num_filtered_range, duplicates,
  //              num_ok + num_filtered_azimuth + num_filtered_range, pc.points.size());
  return slice_angle;
}

} // namespace velodyne_rawdata
