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
        g2.motors.set_throttle(0.0f);
        g2.motors.set_steering(0.0f);
        gcs().send_text(MAV_SEVERITY_WARNING, "Cannnot get speeed.");
        return;
    }

    float desired_steering, desired_speed;
    get_pilot_desired_steering_and_speed(desired_steering, desired_speed);
    const float mag_desired_steering = (desired_steering < 0.0f)? -desired_steering : desired_steering;

    // Parameters (Temporary)
    const bool AUTO_THROTTLE_CONTROL = true; // true; // TODO: Unused
    const float STEERING_THRESHOLD = 50.0f; // Start Drift Control when desired_steering <= STEERING_THRESHOLD or STEERING_THRESHOLD <= desired_steering)
    const float STEERING_ANGLE_MAX = radians(30.0f); // Max steering angle
    // const float SLIP_ANGLE_MAX = atan(0.5 * tan(STEERING_ANGLE_MAX));
    const float DRIFT_TURN_ANGLE_MAX = radians(90.0f); // Max drift turn angle from start drifting point 

    // uint32_t now = AP_HAL::millis();

    if(mag_desired_steering <= STEERING_THRESHOLD) {
        if(_is_drifting) {
            _is_drifting = false;
            gcs().send_text(MAV_SEVERITY_NOTICE, "Stop Drifting");
        }

        /* --- control same as manual mode --- */
        float desired_throttle, desired_lateral;
        get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
        get_pilot_desired_lateral(desired_lateral);
        set_steering(desired_steering);
        calc_throttle(desired_speed, true);
        return;
    } else {
        if(!_is_drifting) {
            _is_drifting = true;
            if(!ahrs.get_quaternion(_body_quat_at_start)) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Cannnot get quat.");
                return;
            };
            ahrs.get_quat_body_to_ned(_body_quat_ned_at_start);

            // Vector3f accel_ef = ahrs.get_accel_ef_blended();
            // Vector3f accel_body = ahrs.earth_to_body(accel_ef);
            // _last_lat_accel = accel_body.y;

            //ahrs.reset_gyro_drift();
            gcs().send_text(MAV_SEVERITY_NOTICE, "Start Drifting");
        }

        Vector3f accel_ef = ahrs.get_accel_ef_blended();
        Vector3f accel_body = ahrs.earth_to_body(accel_ef);
        float lat_accel = accel_body.y;
        const float TURN_MAX_G = g.turn_max_g * GRAVITY_MSS;

        Vector3f velocity_ned;
        if(!ahrs.get_velocity_NED(velocity_ned)) {
            g2.motors.set_throttle(0.0f);
            g2.motors.set_steering(0.0f);
            gcs().send_text(MAV_SEVERITY_WARNING, "Cannnot get velocity.");
            return;
        }
        // const float velocity_yaw_angle = atan2(velocity_ned.y, velocity_ned.x);

        Quaternion body_quat_current;
        if(!ahrs.get_quaternion(body_quat_current)) {
            g2.motors.set_throttle(0.0f);
            g2.motors.set_steering(0.0f);
            gcs().send_text(MAV_SEVERITY_WARNING, "Cannnot get body quaternion.");
            return;
        }

        Quaternion body_quat_ned_current;
        ahrs.get_quat_body_to_ned(body_quat_ned_current);

        const float target_turn_angle = (desired_steering / 4500.0f) * DRIFT_TURN_ANGLE_MAX;
        Quaternion target_quat = _body_quat_ned_at_start;
        target_quat.rotate(Vector3f(0.0f, 0.0f, target_turn_angle));

        const float diff_target_yaw_angle = target_quat.angular_difference(body_quat_ned_current).get_euler_yaw();
        // const float diff_target_velo_yaw_angle = wrap_PI(velocity_yaw_angle - target_quat.get_euler_yaw());
        
        // const float steering = g2.motors.get_steering();
        // const float steering_angle = (steering / 4500.0f) * STEERING_ANGLE_MAX;
        // const float slip_angle = atan(0.5 * tan(steering_angle));

        // const float diff_velocity_yaw_angle_from_body = velocity_yaw_angle - body_quat_ned_current.get_euler_yaw();

        Vector3f gyro = ahrs.get_gyro();
        float yaw_rate = gyro.z;

        // const float target_yaw_rate = (is_positive(yaw_rate))? TURN_MAX_G / speed : -TURN_MAX_G / speed;
        // const float diff_target_yaw_rate = yaw_rate - target_yaw_rate;

        float lat_accel2;
        if(!attitude_control.get_lat_accel(lat_accel2)) {
            return;
        }

        const float c = constrain_float((TURN_MAX_G - fabs(lat_accel))/TURN_MAX_G, -1.0, 1.0);
        float steering_out;
        bool b = false;
        if(fabs(diff_target_yaw_angle) < STEERING_ANGLE_MAX) {
            steering_out = diff_target_yaw_angle * 4500.0f / STEERING_ANGLE_MAX;
            steering_out = constrain_float(steering_out, -4500.0f, 4500.0f);
            set_steering(steering_out);
            b = false;
        } else {
            steering_out = (is_positive(diff_target_yaw_angle))? 4500.0f : -4500.0f;
            const float desired_lat_accel = (is_positive(yaw_rate))? TURN_MAX_G : -TURN_MAX_G;
            bool reversed = false; // TODO
            calc_steering_from_lateral_acceleration(desired_lat_accel, reversed);
            b = true;
        }

        gcs().send_text(MAV_SEVERITY_INFO, "%f, %f, %f, %f, %f, %f, %f, %d",
            speed, body_quat_ned_current.get_euler_yaw(), diff_target_yaw_angle, lat_accel, lat_accel2,
            yaw_rate, ahrs.get_yaw_rate_earth(), b
        );

        if(AUTO_THROTTLE_CONTROL) {
            calc_throttle(c * desired_speed, true);
        } else {
            calc_throttle(desired_speed, true);
        }
    }
}