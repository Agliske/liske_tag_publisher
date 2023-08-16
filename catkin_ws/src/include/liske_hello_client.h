/*******************************************************************************
 * Copyright 2021 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/



#include <ros/ros.h>
#include "liske_tag_publisher/tag_detection_msg.h"
#include "std_msgs/String.h"
#include "generic_interface.h"

#define BUF_LEN 64
#define AI_DETECTION_MAGIC_NUMBER (0x564F584C)

// typedef struct {
//     uint32_t magic_number;                  ///< Unique 32-bit number used to signal the beginning of a VIO packet while parsing a data stream.
//     int32_t id;                            ///< id number of the tag
//     float size_m;                        ///< size of the tag in meters
//     int64_t timestamp_ns;                  ///< timestamp at the middle of the frame exposure in monotonic time
//     char name[TAG_NAME_LEN];            ///< optional name of the tag
//     int loc_type;                      ///< location type
//     float T_tag_wrt_cam[3];              ///< location of the tag with respect to camera frame in meters.
//     float R_tag_to_cam[3][3];            ///< rotation matrix from tag frame to camera frame
//     float T_tag_wrt_fixed[3];            ///< only set if location type is LOCATION_FIXED
//     float R_tag_to_fixed[3][3];          ///< only set if location type is LOCATION_FIXED
//     char cam[MODAL_PIPE_MAX_DIR_LEN];   ///< camera pipe where the detection was made
//     int reserved;                      ///< reserved field
// } __attribute__((packed)) tag_detection_t;
