#include "Sub.h"

#if AUTOTUNE_ENABLED == ENABLED

bool AutoTune::init()
{
    const Mode::Number from = sub.control_mode;
    if (from != Mode::Number::STABILIZE &&
        from != Mode::Number::ALT_HOLD &&
        from != Mode::Number::POSHOLD) {
        gcs().send_text(MAV_SEVERITY_WARNING, "AutoTune: enter from STABILIZE/ALT_HOLD/POSHOLD");
        return false;
    }

    if (!sub.motors.armed()) {
        return false;
    }

    if (!sub.control_check_barometer()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "AutoTune: needs depth sensor");
        return false;
    }

    // ROV can't hold position by leaning, so never use copter-style poshold
    return init_internals(false,
                          &sub.attitude_control,
                          &sub.pos_control,
                          &sub.ahrs_view,
                          &sub.inertial_nav);
}

void AutoTune::run()
{
    sub.motors.set_forward(0.0f);
    sub.motors.set_lateral(0.0f);
    AC_AutoTune::run();
}

float AutoTune::get_pilot_desired_climb_rate_cms(void) const
{
    const float climb = sub.get_pilot_desired_climb_rate(sub.channel_throttle->get_control_in());
    return constrain_float(climb, -sub.get_pilot_speed_dn(), sub.g.pilot_speed_up);
}

void AutoTune::get_pilot_desired_rp_yrate_cd(float &des_roll_cd, float &des_pitch_cd, float &yaw_rate_cds)
{
    sub.get_pilot_desired_lean_angles(sub.channel_roll->get_control_in(),
                                      sub.channel_pitch->get_control_in(),
                                      des_roll_cd, des_pitch_cd,
                                      sub.attitude_control.get_althold_lean_angle_max_cd());

    const float yaw_in = sub.channel_yaw->pwm_to_angle_dz_trim(sub.channel_yaw->get_dead_zone() * sub.gain,
                                                               sub.channel_yaw->get_radio_trim());
    yaw_rate_cds = sub.get_pilot_desired_yaw_rate(yaw_in);
}

void AutoTune::init_z_limits()
{
    sub.pos_control.set_max_speed_accel_z(-sub.get_pilot_speed_dn(), sub.g.pilot_speed_up, sub.g.pilot_accel_z);
    sub.pos_control.set_correction_speed_accel_z(-sub.get_pilot_speed_dn(), sub.g.pilot_speed_up, sub.g.pilot_accel_z);
}

#if HAL_LOGGING_ENABLED
void AutoTune::log_pids()
{
    sub.logger.Write_PID(LOG_PIDR_MSG, sub.attitude_control.get_rate_roll_pid().get_pid_info());
    sub.logger.Write_PID(LOG_PIDP_MSG, sub.attitude_control.get_rate_pitch_pid().get_pid_info());
    sub.logger.Write_PID(LOG_PIDY_MSG, sub.attitude_control.get_rate_yaw_pid().get_pid_info());
}
#endif

bool ModeAutoTune::init(bool ignore_checks)
{
    return autotune.init();
}

void ModeAutoTune::run()
{
    autotune.run();
}

void ModeAutoTune::exit()
{
    autotune.stop();
}

void ModeAutoTune::save_tuning_gains()
{
    autotune.save_tuning_gains();
}

void ModeAutoTune::reset()
{
    autotune.reset();
}

#endif
