/**
 * BSD 3-Clause License
 * 
 * Copyright (c) 2025, Peter Christoffersen
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "feetech_sts_models.hpp"

namespace feetech_sts_hardware
{

static constexpr double KG_CM_TO_NM { 9.80665 * 0.01 };

/** Function to convert Feetech Kt spec kg*cm/A to Nm/A */
static constexpr double ft_to_kt(double val) {
    return val * KG_CM_TO_NM;
};


/* Feetech Server models (Only a few here ) */
constexpr STSModel MODELS[] = {
    { 0xFFFF/* TODO*/,  "ST-3215", ft_to_kt(8.0) },       // 7.4V version of ST-3215
    { 0x0309,  "ST-3215-C018", ft_to_kt(11.0) } // 12V version of ST-3215
};


const STSModel *STSModel::find(uint16_t model_id)
{
    for (auto &model : MODELS) {
        if (model.model_number()==model_id) {
            return &model;
        }
    }
    return nullptr;
}

const STSModel *STSModel::unknown()
{
    static constexpr STSModel model { 0x0000, "Unknown", 0.0 };
    return &model;
}


}