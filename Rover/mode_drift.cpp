#include "mode.h"
#include "Rover.h"

bool ModeDrift::_enter()
{
    _is_drifting = false;
    return true;
}

void ModeDrift::update()
{
    // get speed forward
    float speed;
    if (!attitude_control.get_forward_speed(speed)) {
        // no valid speed so stop
        g2.motors.set_throttle(0.0f);
        g2.motors.set_steering(0.0f);
        return;
    }

    // Vector3f gyro = ahrs.get_gyro(); // rad/s
    // float yaw_rate_earth = ahrs.get_yaw_rate_earth();
    // float roll = ahrs.get_roll();
    // float pitch = ahrs.get_pitch();
    // float yaw = ahrs.get_pitch();
    // float error_rp = ahrs.get_error_rp();
    // float error_yaw = ahrs.get_error_yaw();

    // uint32_t t = AP_HAL::millis();
    // gcs().send_text(MAV_SEVERITY_NOTICE, "t:%u, (%f,%f,%f),(%f,%f,%f),%f,%f,%f", t, gyro.x, gyro.y, gyro.z, roll, pitch, yaw, yaw_rate_eath, error_rp, error_yaw);
    //gcs().send_text(MAV_SEVERITY_NOTICE, "t:%u, speed:%f, steering:%f, desired_speed:%f", t, speed, desired_steering, desired_speed);

    float desired_steering, desired_speed;
    get_pilot_desired_steering_and_speed(desired_steering, desired_speed);
    // bool reversed = is_negative(desired_speed);

    const bool AUTO_THROTTLE_CONTROL = true;
    const float STEERING_THRESHOLD = 50.0f;
    const float DRIFT_TURN_RATE = radians(30.0f);
    const float DRIFT_TURN_ANGLE_MAX = radians(90.0f); // radians(180.0f); //
    const float mag_desired_steering = (desired_steering < 0.0f)? -desired_steering : desired_steering;
    if(mag_desired_steering <= STEERING_THRESHOLD) {
        if(_is_drifting) {
            _is_drifting = false;
            gcs().send_text(MAV_SEVERITY_NOTICE, "Stop Drifting");
        }

        /* --- control same as manual mode --- */
        float desired_throttle, desired_lateral;
        get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
        get_pilot_desired_lateral(desired_lateral);

        // copy RC scaled inputs to outputs
        g2.motors.set_throttle(desired_throttle);
        g2.motors.set_steering(desired_steering, false);
        g2.motors.set_lateral(desired_lateral);
        return;
    } else {
        if(!_is_drifting) {
            _is_drifting = true;
            ahrs.get_quat_body_to_ned(_quat_at_start);
            _quat_at_start.rotation_matrix_norm(_rotation_at_start);
            //ahrs.reset_gyro_drift();
            gcs().send_text(MAV_SEVERITY_NOTICE, "Start Drifting");
        } else {
            Quaternion quat_current;
            ahrs.get_quat_body_to_ned(quat_current);

            const float target_turn_angle = (desired_steering / 4500.0f) * DRIFT_TURN_ANGLE_MAX;
            Quaternion target_quat = _quat_at_start;
            target_quat.rotate(Vector3f(0.0f, 0.0f, target_turn_angle));
            Quaternion diff = quat_current.angular_difference(target_quat);

            const float diff_yaw = diff.get_euler_yaw();
            const float mag_diff_yaw = (diff_yaw < 0.0f)? -diff_yaw : diff_yaw;
            const float steering_direction = (diff_yaw < 0.0f)? 1.0f : -1.0f;

            float target_turn_rate = 0.0;
            if(mag_diff_yaw >= DRIFT_TURN_RATE) {
                target_turn_rate = steering_direction * DRIFT_TURN_RATE;
            } else {
                target_turn_rate = steering_direction * mag_diff_yaw; // -diff_yaw
            }

            const float steering_out = attitude_control.get_steering_out_rate(
                target_turn_rate,
                g2.motors.limit.steer_left,
                g2.motors.limit.steer_right,
                rover.G_Dt
            );

            set_steering(steering_out * 4500.0f);

            if(AUTO_THROTTLE_CONTROL) {
                float lat_accel;
                attitude_control.get_lat_accel(lat_accel);
                Vector3f accel_ef = ahrs.get_accel_ef_blended();
                Vector3f gyro = ahrs.get_gyro();
                Vector3f gyro_ned = ahrs.get_rotation_body_to_ned().mul_transpose(gyro);

                Vector3f accel_body = ahrs.earth_to_body(accel_ef);
                Vector3f accel_s = _rotation_at_start.mul_transpose(accel_ef);
                gcs().send_text(MAV_SEVERITY_NOTICE,
                    "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f",
                    accel_ef.x, accel_ef.y, accel_ef.z,
                    accel_body.x, accel_body.y, accel_body.z,
                    accel_s.x, accel_s.y, accel_s.z,
                    lat_accel, speed,
                    gyro.x, gyro.y, gyro.z,
                    gyro_ned.x, gyro_ned.y, gyro_ned.z
                );
                // Vector3f gyro = ahrs.get_gyro();
                // Matrix3f body_to_ned = ahrs.get_rotation_body_to_ned();
                // float acl_roll = speed * (gyro * body_to_ned.a);
                // float acl_pitch = speed * (gyro * body_to_ned.b);
                // float acl_yaw = speed * (gyro * body_to_ned.c);
                
                // gcs().send_text(MAV_SEVERITY_NOTICE,
                //     "(%f, %f, %f) - (%f, %f, %f) - %f",
                //     accel.x, accel.y, accel.z,
                //     acl_roll, acl_pitch, acl_yaw,
                //     speed
                // );
            }

            // gcs().send_text(
            //     MAV_SEVERITY_NOTICE,
            //     "Now (roll:%f, pitch:%f, yaw: %f), (%f, %f, %f), %f, %f",
            //     quat_current.get_euler_roll(), quat_current.get_euler_pitch(), quat_current.get_euler_yaw(),
            //     diff.get_euler_roll(), diff.get_euler_pitch(), diff.get_euler_yaw(), cos(diff.get_euler_yaw()), sin(diff.get_euler_yaw()));
        }
        //g2.motors.set_throttle(0.0f);
    }

    // // determine if pilot is requesting pivot turn
    // if (g2.motors.have_skid_steering() && is_zero(desired_speed)) {
    //     // pivot turning using turn rate controller
    //     // convert pilot steering input to desired turn rate in radians/sec
    //     const float target_turn_rate = (desired_steering / 4500.0f) * radians(g2.acro_turn_rate);

    //     // run steering turn rate controller and throttle controller
    //     const float steering_out = attitude_control.get_steering_out_rate(target_turn_rate,
    //                                                                       g2.motors.limit.steer_left,
    //                                                                       g2.motors.limit.steer_right,
    //                                                                       rover.G_Dt);
    //     set_steering(steering_out * 4500.0f);
    // } else {
    //     // In steering mode we control lateral acceleration directly.
    //     // For regular steering vehicles we use the maximum lateral acceleration
    //     //  at full steering lock for this speed: V^2/R where R is the radius of turn.
    //     float max_g_force = speed * speed / MAX(g2.turn_radius, 0.1f);
    //     max_g_force = constrain_float(max_g_force, 0.1f, g.turn_max_g * GRAVITY_MSS);

    //     // convert pilot steering input to desired lateral acceleration
    //     float _desired_lat_accel = max_g_force * (desired_steering / 4500.0f);

    //     // run lateral acceleration to steering controller
    //     calc_steering_from_lateral_acceleration(_desired_lat_accel, reversed);
    // }

    // run speed to throttle controller
    calc_throttle(desired_speed, true);
}
