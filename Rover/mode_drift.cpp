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

    float desired_steering, desired_speed;
    get_pilot_desired_steering_and_speed(desired_steering, desired_speed);
    const float mag_desired_steering = (desired_steering < 0.0f)? -desired_steering : desired_steering;

    // Parameters (Temporary)
    const bool AUTO_THROTTLE_CONTROL = false; // true; // TODO: Unused
    const float STEERING_THRESHOLD = 50.0f; // Start Drift Control when desired_steering <= STEERING_THRESHOLD or STEERING_THRESHOLD <= desired_steering)
    // const float DRIFT_TURN_RATE = radians(30.0f); // Turn rate when steering value max
    const float DRIFT_TURN_ANGLE_MAX = radians(180.0f); // Max drift turn angle from start drifting point  

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
            //ahrs.get_quat_body_to_ned(_quat_at_start);
            //_quat_at_start.rotation_matrix_norm(_rotation_at_start);
            if(!ahrs.get_quaternion(_quat_at_start)) {
                return;
            };
            //ahrs.reset_gyro_drift();
            gcs().send_text(MAV_SEVERITY_NOTICE, "Start Drifting");
        }

        Quaternion quat_current;
        if(!ahrs.get_quaternion(quat_current)) {
            return;
        }

        const float target_turn_angle = (desired_steering / 4500.0f) * DRIFT_TURN_ANGLE_MAX;
        Quaternion target_quat = _quat_at_start;
        target_quat.rotate(Vector3f(0.0f, 0.0f, target_turn_angle));
        Quaternion diff = target_quat.angular_difference(quat_current);

        gcs().send_text(MAV_SEVERITY_NOTICE, "%f, %f, %f, %f",
          degrees(_quat_at_start.get_euler_yaw()),
          degrees(target_quat.get_euler_yaw()),
          degrees(quat_current.get_euler_yaw()),
          degrees(diff.get_euler_yaw())
        );

        float target_turn_rate = diff.get_euler_yaw();

        const float steering_out = attitude_control.get_steering_out_rate(
            target_turn_rate,
            g2.motors.limit.steer_left,
            g2.motors.limit.steer_right,
            rover.G_Dt
        );

        set_steering(steering_out * 4500.0f);

        if(AUTO_THROTTLE_CONTROL) {
            // TODO: control throttle
            // TODO: set throttle zero while drifting
            // g2.motors.set_throttle(0.0f);
        } else {
            // run speed to throttle controller
            calc_throttle(desired_speed, true);
        }
    }
}
