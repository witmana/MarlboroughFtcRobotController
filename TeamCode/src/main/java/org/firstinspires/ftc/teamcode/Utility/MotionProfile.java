package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MotionProfile {

    ElapsedTime elapsedTime;
    double profileTime;
    double distance;
    double max_acceleration;
    double max_velocity;
    public boolean profileComplete;

    public MotionProfile(double maxAccelerationIn, double maxVelocityIn, double distanceIn){
        elapsedTime = new ElapsedTime();
        max_acceleration = maxAccelerationIn;
        max_velocity = maxVelocityIn;
        distance = distanceIn;
        profileTime = returnProfileDuration(max_acceleration,max_velocity,distance);
        elapsedTime.reset();
        profileComplete = false;
    }

    public double returnInstantTarget() {

        if(elapsedTime.seconds() > profileTime){
            profileComplete = true;
        }

        //Return the current reference position based on the given motion profile times, maximum acceleration, velocity, and current time.
        if(distance < 0){
            distance*=-1;
            //direction = -1;
        }

        // calculate the time it takes to accelerate to max velocity
        double acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = distance / 2;
        double acceleration_distance = 0.5 * max_acceleration * (acceleration_dt * acceleration_dt);

        if (acceleration_distance > halfway_distance)
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));

        acceleration_distance = 0.5 * max_acceleration * (acceleration_dt * acceleration_dt);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        double deacceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        double cruise_distance = distance - 2 * acceleration_distance;
        double cruise_dt = cruise_distance / max_velocity;
        double deacceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deacceleration_dt;
        if (elapsedTime.seconds() > entire_dt){
            return distance;
        }

        // if we're accelerating
        if (elapsedTime.seconds() < acceleration_dt)
            // use the kinematic equation for acceleration
            return 0.5 * max_acceleration * (elapsedTime.seconds() * elapsedTime.seconds());

            // if we're cruising
        else if (elapsedTime.seconds() < deacceleration_time) {
            acceleration_distance = 0.5 * max_acceleration * (acceleration_dt * acceleration_dt);
            double cruise_current_dt = elapsedTime.seconds() - acceleration_dt;

            // use the kinematic equation for constant velocity
            return (acceleration_distance + max_velocity * cruise_current_dt);
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * max_acceleration * (acceleration_dt * acceleration_dt);
            cruise_distance = max_velocity * cruise_dt;
            deacceleration_time = elapsedTime.seconds() - deacceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return (acceleration_distance + cruise_distance + max_velocity * deacceleration_time - 0.5 * max_acceleration * (deacceleration_time * deacceleration_time));
        }
    }

    //returns the duration of the motion profile
    public static double returnProfileDuration(double max_acceleration, double max_velocity, double distance) {

        //Return the current reference position based on the given motion profile times, maximum acceleration, velocity, and current time.
        if(distance < 0){
            distance*=-1;
            //direction = -1;
        }

        // calculate the time it takes to accelerate to max velocity
        double acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = distance / 2;
        double acceleration_distance = 0.5 * max_acceleration * (acceleration_dt * acceleration_dt);

        if (acceleration_distance > halfway_distance)
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));

        acceleration_distance = 0.5 * max_acceleration * (acceleration_dt * acceleration_dt);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        double deacceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        double cruise_distance = distance - 2 * acceleration_distance;
        double cruise_dt = cruise_distance / max_velocity;
        double deacceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deacceleration_dt;
        return entire_dt;
    }


}
