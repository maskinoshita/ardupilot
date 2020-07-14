#include "mode.h"
#include "Rover.h"

bool ModeDrift::_enter()
{
    _is_drifting = false;
    return true;
}

void ModeDrift::update()
{
    // 現在のスピードの取得
    float speed;
    if (!attitude_control.get_forward_speed(speed)) {
        g2.motors.set_throttle(0.0f);
        g2.motors.set_steering(0.0f);
        gcs().send_text(MAV_SEVERITY_WARNING, "Cannnot get speeed.");
        return;
    }

    // パイロットの入力ステアリング[-4500,4500]とスピード(m/s)を取得
    float desired_steering, desired_speed;
    get_pilot_desired_steering_and_speed(desired_steering, desired_speed);
    const float mag_desired_steering = (desired_steering < 0.0f)? -desired_steering : desired_steering;

    const bool AUTO_THROTTLE_CONTROL = g2.drift_throttle_control; // ドリフト制御中にスロットル制御をするかどうか (0:Disabled, 1:Enabled)
    const float STEERING_THRESHOLD = g2.drift_steering_threshold; // ドリフト制御を開始するパイロットのステアリング入力の閾値
    const float STEERING_ANGLE_MAX = g2.drift_steering_angle_max; // ステアリングの最大角
    const float DRIFT_TURN_ANGLE_MAX = g2.drift_turn_angle_max; // ドリフト制御で曲がる最大角 (パイロットのステアリング入力が最大値(4500)の場合曲がる角)

    if(mag_desired_steering <= STEERING_THRESHOLD) {
        // パイロットのステアリング入力が閾値より小さい場合、ドリフト制御をせずにマニュアルモードと同じ制御を行う

        // ステアリング入力のスティックを戻すと、ドリフト制御をやめる
        if(_is_drifting) {
            _is_drifting = false;
            gcs().send_text(MAV_SEVERITY_NOTICE, "Stop Drifting");
        }

        // マニュアルモードと同じ制御
        float desired_throttle, desired_lateral;
        get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
        get_pilot_desired_lateral(desired_lateral);
        set_steering(desired_steering);
        calc_throttle(desired_speed, true);
        return;
    } else {
        // パイロットのステアリング入力が閾値より大きい場合、ドリフト制御を開始する
        if(!_is_drifting) {
            _is_drifting = true;
            // ドリフト制御を開始したボディ角度を取得する
            ahrs.get_quat_body_to_ned(_body_quat_ned_at_start);
            gcs().send_text(MAV_SEVERITY_NOTICE, "Start Drifting");
        }

        // 横加速度を取得
        Vector3f accel_ef = ahrs.get_accel_ef_blended();
        Vector3f accel_body = ahrs.earth_to_body(accel_ef);
        float lat_accel = accel_body.y;
        const float TURN_MAX_G = g.turn_max_g * GRAVITY_MSS;

        // 現在のボディ角度を取得
        Quaternion body_quat_ned_current;
        ahrs.get_quat_body_to_ned(body_quat_ned_current);

        // ドリフト開始時のボディ角度からパイロット入力のステアリング値を加算した目標のボディ角度を計算
        const float target_turn_angle = (desired_steering / 4500.0f) * DRIFT_TURN_ANGLE_MAX;
        Quaternion target_quat = _body_quat_ned_at_start;
        target_quat.rotate(Vector3f(0.0f, 0.0f, target_turn_angle));

        // 現在のボディ角度と目標のボディ角度の差を取得
        const float diff_target_yaw_angle = target_quat.angular_difference(body_quat_ned_current).get_euler_yaw();

        // ヨーレートを取得
        Vector3f gyro = ahrs.get_gyro();
        float yaw_rate = gyro.z;

        // 横加速度が大きくなると横転の可能性が高くなるので、スロットル制御を行う場合横加速度に応じた補正を行う
        if(AUTO_THROTTLE_CONTROL) {
            const float c = constrain_float((TURN_MAX_G - fabs(lat_accel))/TURN_MAX_G, -1.0, 1.0);
            desired_speed = c * desired_speed;
        }

        float steering_out;
        if(fabs(diff_target_yaw_angle) < STEERING_ANGLE_MAX) {
            // ターゲットのボディ角度がステアリング角度の最大値より小さい場合は、通常通りステアリングを制御する
            steering_out = diff_target_yaw_angle * 4500.0f / STEERING_ANGLE_MAX;
            steering_out = constrain_float(steering_out, -4500.0f, 4500.0f);
            set_steering(steering_out);
        } else {
            // ターゲットのボディ角度がステアリング角度の最大値より大きい場合は、横加速度を限界ぎりぎりになるように制御する
            const float desired_lat_accel = (is_positive(yaw_rate))? TURN_MAX_G : -TURN_MAX_G;
            bool reversed = is_negative(desired_speed);
            calc_steering_from_lateral_acceleration(desired_lat_accel, reversed);
        }

        calc_throttle(desired_speed, true);
    }
}