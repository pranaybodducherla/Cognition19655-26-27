package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

public class HardwarePushbot {

    // Distance between module centers (only the ratio matters)
    private static final double WHEELBASE = 1.0;   // front-to-back
    private static final double TRACK_WIDTH = 1.0; // left-to-right

    // Axon MAX Mk2 in standard mode: 350 deg over 500-2500 us
    private static final double SERVO_RANGE_DEG = 350;
    private static final double SERVO_DIRECTION = -1;  // Axon turns clockwise with increasing PWM; flip to 1 if wheels steer the wrong way
    private static final double DEADBAND = 0.02;

    private Module[] modules;

    public HardwarePushbot() {}

    public void init(HardwareMap hw) {
        double x = WHEELBASE / 2;
        double y = TRACK_WIDTH / 2;

        // x = forward, y = left, last value trims each module's straight-ahead servo position
        modules = new Module[] {
                new Module(hw, "upperLeft",  "upperLeftServo",   x,  y, 0),
                new Module(hw, "upperRight", "upperRightServo",  x, -y, 0),
                new Module(hw, "lowerLeft",  "lowerLeftServo",  -x,  y, 0),
                new Module(hw, "lowerRight", "lowerRightServo", -x, -y, 0)
        };
    }

    /** Inputs in [-1, 1]. Forward = +, strafe left = +, turn counter-clockwise = +. */
    public void drive(double forward, double strafe, double turn) {
        double[] speed = new double[modules.length];
        double[] angle = new double[modules.length];
        double max = 1;

        for (int i = 0; i < modules.length; i++) {
            double vx = forward - turn * modules[i].y;
            double vy = strafe + turn * modules[i].x;
            speed[i] = Math.hypot(vx, vy);
            angle[i] = Math.toDegrees(Math.atan2(vy, vx));
            max = Math.max(max, speed[i]);
        }

        for (int i = 0; i < modules.length; i++) {
            modules[i].set(speed[i] / max, angle[i]);
        }
    }

    public void stop() {
        drive(0, 0, 0);
    }

    private static class Module {
        final DcMotor motor;
        final ServoImplEx servo;
        final double x, y, servoOffset;

        Module(HardwareMap hw, String motorName, String servoName, double x, double y, double servoOffset) {
            this.motor = hw.get(DcMotor.class, motorName);
            this.servo = hw.get(ServoImplEx.class, servoName);
            servo.setPwmRange(new PwmRange(500, 2500));
            this.x = x;
            this.y = y;
            this.servoOffset = servoOffset;
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        void set(double speed, double angleDeg) {
            if (speed < DEADBAND) {
                motor.setPower(0); // hold current steering angle
                return;
            }

            // Steer to the nearest equivalent angle within +/-90 deg, reversing the wheel if needed
            if (Math.abs(angleDeg) > 90) {
                angleDeg -= Math.copySign(180, angleDeg);
                speed = -speed;
            }

            double position = 0.5 + servoOffset + SERVO_DIRECTION * angleDeg / SERVO_RANGE_DEG;
            servo.setPosition(Range.clip(position, 0, 1));
            motor.setPower(speed);
        }
    }
}