package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * 4-motor square drivetrain, no servos.
 *
 * gamepad1:
 *   left stick up/down       drive forward / backward
 *   right stick to the RIGHT left motors forward (more power), right motors backward -> turns right
 *   right stick to the LEFT  right motors forward (more power), left motors backward -> turns left
 *   left bumper (hold)       slow mode
 */
@TeleOp(name = "Tank Drive")
public class TankTeleOp extends LinearOpMode {

    // Power on the side that pushes forward during a turn (the outside of the turn).
    private static final double OUTSIDE_POWER = 1.0;
    // Power on the side that goes backward during a turn, as a fraction of the outside.
    // 1.0 = both sides equal (tightest spin). Lower = inside side pulls back less.
    private static final double INSIDE_FRACTION = 0.5;
    private static final double SLOW = 0.35;
    private static final double DEADZONE = 0.05;

    @Override
    public void runOpMode() {
        // Names must match your robot configuration on the Driver Hub.
        DcMotor topLeft     = hardwareMap.get(DcMotor.class, "a");
        DcMotor topRight    = hardwareMap.get(DcMotor.class, "b");
        DcMotor bottomLeft  = hardwareMap.get(DcMotor.class, "c");
        DcMotor bottomRight = hardwareMap.get(DcMotor.class, "d");

        // Motors on opposite sides face opposite ways, so one side is reversed.
        // If a wheel spins backwards when you push the left stick forward, flip that motor's line.
        topLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        topRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRight.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor[] all = {topLeft, topRight, bottomLeft, bottomRight};
        for (DcMotor m : all) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setPower(0);
        }

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y; // stick up is negative, so flip it
            double turn = gamepad1.right_stick_x;    // right = positive, left = negative
            if (Math.abs(forward) < DEADZONE) forward = 0;
            if (Math.abs(turn) < DEADZONE) turn = 0;

            double leftPower;
            double rightPower;

            if (turn > 0) {
                // TURN RIGHT: left motors forward with more power, right motors backward
                leftPower = turn * OUTSIDE_POWER;
                rightPower = -turn * OUTSIDE_POWER * INSIDE_FRACTION;
            } else if (turn < 0) {
                // TURN LEFT: right motors forward with more power, left motors backward
                double amount = -turn; // make it positive
                rightPower = amount * OUTSIDE_POWER;
                leftPower = -amount * OUTSIDE_POWER * INSIDE_FRACTION;
            } else {
                // STRAIGHT: all four motors the same
                leftPower = forward;
                rightPower = forward;
            }

            if (gamepad1.left_bumper) {
                leftPower *= SLOW;
                rightPower *= SLOW;
            }

            topLeft.setPower(leftPower);
            bottomLeft.setPower(leftPower);
            topRight.setPower(rightPower);
            bottomRight.setPower(rightPower);

            telemetry.addData("left motors", "%.2f", leftPower);
            telemetry.addData("right motors", "%.2f", rightPower);
            telemetry.update();
        }

        for (DcMotor m : all) m.setPower(0);
    }
}
