package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/** Four-motor mecanum drivetrain. Hardware names: a=front-left, b=front-right,
 * c=back-left, d=back-right. */
public class HardwarePushbot {
    public final DcMotor frontLeft;
    public final DcMotor frontRight;
    public final DcMotor backLeft;
    public final DcMotor backRight;
    public final DcMotor intake;
    public final IMU imu;

    public HardwarePushbot(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, "a");
        frontRight = hardwareMap.get(DcMotor.class, "b");
        backLeft = hardwareMap.get(DcMotor.class, "c");
        backRight = hardwareMap.get(DcMotor.class, "d");
        intake = hardwareMap.get(DcMotor.class, "intake");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        // Adjust these four directions to match the motor mounting and wiring.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE); // a
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD); // b
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD); // c
        backRight.setDirection(DcMotorSimple.Direction.REVERSE); // d

        DcMotor[] motors = {frontLeft, frontRight, backLeft, backRight, intake};
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
        }
    }

    /** Same axis signs and mecanum mix as the proven drive.java: forward, right strafe, right turn. */
    public void drive(double axial, double lateral, double yaw) {
        double fl = axial + lateral + yaw;
        double fr = (axial - lateral) - yaw;
        double bl = (axial - lateral) + yaw;
        double br = (axial + lateral) - yaw;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    public void stop() {
        drive(0, 0, 0);
    }

    public void setBrakeEnabled(boolean enabled) {
        DcMotor.ZeroPowerBehavior behavior = enabled
                ? DcMotor.ZeroPowerBehavior.BRAKE
                : DcMotor.ZeroPowerBehavior.FLOAT;
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }

    /** Positive power runs the intake forward; negative power runs it backward. */
    public void runIntake(double power) {
        intake.setPower(Math.max(-1.0, Math.min(1.0, power)));
    }

    public double headingRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}
