/**
 * SPDX-License-Identifier: GPLv2
 *
 * Copyright (C) 2025  Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/**
 * @file ha_switch.h
 *
 * @brief ha_switch Class encapsulates mqtt_device_trigger and mqtt_switch.
 *
 * @author Vinicius Silva <silva.viniciusr@gmail.com>
 *
 * @date September 04th 2025
 *
 */

#pragma once

#include "esp_err.h"

class HaSwitch;
class HaVirtualSwitch;
typedef void (*user_cb)(HaSwitch *user_ctx);

class HaSwitch {
public:
  const user_cb m_user_callback;
  HaSwitch(bool gui_switch = 0, user_cb user_callback = nullptr);
  ~HaSwitch();
  bool get();
  esp_err_t set();
  esp_err_t reset();
  esp_err_t toggle();
  esp_err_t Connect();

private:
  HaVirtualSwitch *m_switch_p;
  static unsigned s_m_count;
};
