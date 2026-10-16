#include "Sub.h"

// Helper untuk mengambil referensi objek PID sesuai sumbu yang dipilih di g2.atun_axis
AC_PID& ModeAutoTune::_get_active_rate_pid() {
    switch (g2.atun_axis.get()) {
        case 1:
            return attitude_control->get_rate_pitch_pid();
        case 2:
            return attitude_control->get_rate_yaw_pid();
        case 0:
        default:
            return attitude_control->get_rate_roll_pid();
    }
}

const char* ModeAutoTune::_get_axis_name() const {
    switch (g2.atun_axis.get()) {
        case 1: return "PITCH";
        case 2: return "YAW";
        case 0:
        default: return "ROLL";
    }
}

bool ModeAutoTune::init(bool ignore_checks) {
    if (!sub.motors.armed()) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "ATune: Harus ARMED terlebih dahulu!");
        return false;
    }

    _atune_state = STATE_INIT;
    gcs().send_text(MAV_SEVERITY_INFO, "ATune: Aktif pada sumbu %s", _get_axis_name());
    return true;
}

void ModeAutoTune::run() {
    uint32_t now = AP_HAL::millis();

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    motors.set_throttle(0.5f);
    motors.set_forward(0.0f);
    motors.set_lateral(0.0f);

    AC_PID& active_pid = _get_active_rate_pid();
    int8_t axis = g2.atun_axis.get();

    switch (_atune_state) {
        case STATE_IDLE:
            break;

        case STATE_INIT: {
            // Nol-kan I dan D pada sumbu yang sedang di-tune
            active_pid.kI(0.0f);
            active_pid.kD(0.0f);
            active_pid.reset_I();

            // Baca nilai awal langsung dari parameter QGroundControl (ParametersG2)
            _p_test_val  = MAX(0.001f, g2.atun_kp_init.get());
            _step_val    = MAX(0.001f, g2.atun_step.get());
            _lower_bound = 0.0f;
            _upper_bound = 0.0f;
            _found_ceiling = false;

            _ku = 0.0f;
            _tu = 0.5f;

            _tune_start_time_ms = now;
            _last_cross_time_ms = 0;
            _cross_count = 0;
            _is_positive_cycle = true;

            _atune_state = STATE_TUNE_RATE_P;
            gcs().send_text(MAV_SEVERITY_INFO, "ATune [%s]: Mulai P=%.4f Step=%.4f",
                            _get_axis_name(), _p_test_val, _step_val);
            break;
        }

        case STATE_TUNE_RATE_P: {
            active_pid.kP(_p_test_val);

            // Beri impuls eksitasi selama 150ms pertama di awal setiap step
            bool in_kick = (now - _tune_start_time_ms < 150);
            float kick_cd = in_kick ? 1500.0f : 0.0f; // 15 deg atau 15 deg/s

            Vector3f gyro_latest = ahrs.get_gyro_latest();
            float rate_val = 0.0f;

            if (axis == 1) { // PITCH
                attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, kick_cd, 0.0f);
                rate_val = gyro_latest.y;
            } else if (axis == 2) { // YAW
                attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, kick_cd);
                rate_val = gyro_latest.z;
            } else { // ROLL (Default)
                attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(kick_cd, 0.0f, 0.0f);
                rate_val = gyro_latest.x;
            }

            if (!in_kick) {
                _update_measurement(rate_val, now);
            }

            // Jika terdeteksi >= 8 kali zero-crossing (osilasi)
            if (_cross_count >= 8) {
                _found_ceiling = true;
                _upper_bound = _p_test_val;
                _p_test_val = (_lower_bound + _upper_bound) * 0.5f;
                _step_val = _upper_bound - _p_test_val;

                gcs().send_text(MAV_SEVERITY_INFO, "ATune [%s]: Osilasi (Tu=%.2fs). Turun P: %.4f",
                                _get_axis_name(), _tu, _p_test_val);
                _tune_start_time_ms = now;
                _last_cross_time_ms = 0;
                _cross_count = 0;
            }

            // Baca timeout dari parameter QGC (default 4000 ms)
            uint32_t timeout_ms = (uint32_t)MAX(1000, g2.atun_timeout.get());
            if (now - _tune_start_time_ms > timeout_ms) {
                _lower_bound = _p_test_val;
                if (!_found_ceiling) {
                    _p_test_val += _step_val;
                } else {
                    _step_val = (_upper_bound - _lower_bound) * 0.5f;
                    if (_step_val <= 0.005f) {
                        _ku = _upper_bound;
                        _atune_state = STATE_CALC_ZIG_NIC;
                        break;
                    } else {
                        _p_test_val = _lower_bound + _step_val;
                    }
                }
                gcs().send_text(MAV_SEVERITY_INFO, "ATune [%s]: Stabil. Naik P: %.4f",
                                _get_axis_name(), _p_test_val);
                _tune_start_time_ms = now;
                _last_cross_time_ms = 0;
                _cross_count = 0;
            }
            break;
        }

        case STATE_CALC_ZIG_NIC: {
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);
            _calculate_and_save_pid();
            _atune_state = STATE_DONE;
            break;
        }

        case STATE_DONE: {
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);
            break;
        }
    }
}

void ModeAutoTune::_update_measurement(float rate_val, uint32_t now) {
    // Ambil deadband secara dinamis dari parameter QGC
    const float deadband = MAX(0.005f, g2.atun_deadband.get());

    if (_is_positive_cycle && rate_val < -deadband) {
        _is_positive_cycle = false;
        _cross_count++;
    } else if (!_is_positive_cycle && rate_val > deadband) {
        _is_positive_cycle = true;
        _cross_count++;
    } else {
        return;
    }

    if (_cross_count == 1) {
        _last_cross_time_ms = now;
    } else if (_cross_count == 8 && now > _last_cross_time_ms) {
        float total_time_sec = (now - _last_cross_time_ms) * 0.001f;
        _tu = total_time_sec / 3.5f;
    }
}

void ModeAutoTune::_calculate_and_save_pid() {
    if (_tu < 0.05f) {
        _tu = 0.05f;
    }

    float final_p = 0.2f * _ku;
    float final_i = 0.4f * (_ku / _tu);
    float final_d = 0.0f;

    AC_PID& active_pid = _get_active_rate_pid();
    active_pid.kP(final_p);
    active_pid.kI(final_i);
    active_pid.kD(final_d);
    active_pid.save_gains();

    gcs().send_text(MAV_SEVERITY_INFO, "ATune [%s] Selesai! Ku:%.3f Tu:%.2f -> P:%.4f I:%.4f",
                    _get_axis_name(), _ku, _tu, final_p, final_i);
}