package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Mecanum TeleOp", group = "TeleOp")
public class Teleop extends LinearOpMode {
    private static final double TURN_DEADBAND = 0.08;
    private static final double SNAP_KP = 1.8;
    private static final double SNAP_MAX_TURN = 0.45;
    private static final double SNAP_TOLERANCE = Math.toRadians(2.0);

    @Override
    public void runOpMode() {
        HardwarePushbot robot = new HardwarePushbot(hardwareMap);
        telemetry.addLine("Left stick: move | Right stick X: turn | A/B: intake forward/reverse");
        telemetry.addLine("LB: toggle slow | RB: toggle 90-degree snap | Back: toggle brake/coast");
        telemetry.update();
        waitForStart();

        boolean slowMode = false;
        boolean snapMode = false;
        boolean previousLeftBumper = false;
        boolean previousRightBumper = false;
        boolean previousBack = false;
        boolean previousA = false;
        boolean previousB = false;
        boolean brakeEnabled = true;
        int intakeMode = 0; // 0=off, 1=forward, -1=reverse
        boolean wasManuallyTurning = false;
        double snapHeading = 0;

        while (opModeIsActive()) {
            if (gamepad1.left_bumper && !previousLeftBumper) slowMode = !slowMode;
            if (gamepad1.right_bumper && !previousRightBumper) {
                snapMode = !snapMode;
                wasManuallyTurning = false;
                if (snapMode) snapHeading = nearestRightAngle(robot.headingRadians());
            }
            if (gamepad1.back && !previousBack) {
                brakeEnabled = !brakeEnabled;
                robot.setBrakeEnabled(brakeEnabled);
            }
            previousLeftBumper = gamepad1.left_bumper;
            previousRightBumper = gamepad1.right_bumper;
            previousBack = gamepad1.back;

            boolean aPressed = gamepad1.a && !previousA;
            boolean bPressed = gamepad1.b && !previousB;
            if (aPressed && bPressed) {
                intakeMode = 0;
            } else if (aPressed) {
                intakeMode = intakeMode == 1 ? 0 : 1;
            } else if (bPressed) {
                intakeMode = intakeMode == -1 ? 0 : -1;
            }
            previousA = gamepad1.a;
            previousB = gamepad1.b;

            double scale = slowMode ? 0.35 : 1.0;
            double forward = -gamepad1.left_stick_y * scale;
            double strafe = gamepad1.left_stick_x * scale;
            double turn = gamepad1.right_stick_x * scale;

            if (Math.abs(turn) < TURN_DEADBAND) {
                if (snapMode) {
                    if (wasManuallyTurning) snapHeading = nearestRightAngle(robot.headingRadians());
                    double error = wrapRadians(snapHeading - robot.headingRadians());
                    turn = Math.abs(error) < SNAP_TOLERANCE
                            ? 0 : Range.clip(-SNAP_KP * error, -SNAP_MAX_TURN, SNAP_MAX_TURN);
                } else {
                    turn = 0;
                }
                wasManuallyTurning = false;
            } else {
                wasManuallyTurning = true;
            }

            robot.drive(forward, strafe, turn);
            robot.runIntake(intakeMode);
            telemetry.addData("Slow mode", slowMode ? "ON" : "OFF");
            telemetry.addData("90-degree snap", snapMode ? "ON" : "OFF");
            telemetry.addData("Zero-power behavior", brakeEnabled ? "BRAKE" : "FLOAT");
            telemetry.addData("Intake", intakeMode > 0 ? "FORWARD" : intakeMode < 0 ? "REVERSE" : "OFF");
            telemetry.addData("Heading", "%.1f deg", Math.toDegrees(robot.headingRadians()));
            telemetry.update();
        }
        robot.stop();
        robot.runIntake(0);
    }

    private static double nearestRightAngle(double heading) {
        return Math.rint(heading / (Math.PI / 2.0)) * (Math.PI / 2.0);
    }

    private static double wrapRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}
