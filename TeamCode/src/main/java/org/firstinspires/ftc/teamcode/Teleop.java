package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends LinearOpMode {

    private static final double STICK_DEADZONE = 0.05;
    private static final double AIM_KP = 0.02;      // auto-aim strength, tune this
    private static final double SLOW_SPEED = 0.35;  // slow mode speed

    private static final double SNAP_KP = 1.5;          // snap turn strength, tune this
    private static final double SNAP_MAX_TURN = 0.6;    // max turn speed while snapping
    private static final double SNAP_TOLERANCE_DEG = 2; // "close enough" to stop snapping

    private final HardwarePushbot robot = new HardwarePushbot();

    // Target heading in radians, or null when not snapping
    private Double snapTarget = null;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Left stick: move. Right stick X: rotate.
            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            if (Math.hypot(forward, strafe) < STICK_DEADZONE) {
                forward = 0;
                strafe = 0;
            }
            if (Math.abs(turn) < STICK_DEADZONE) {
                turn = 0;
            }

            // OPTIONS: reset "forward" to the direction the robot is facing right now
            if (gamepad1.options) {
                robot.imu.resetYaw();
                snapTarget = null;
            }

            // D-PAD: snap to an angle (relative to the direction set at the last heading reset)
            //   up = 0 (away from driver), left = 90, down = 180, right = -90
            if (gamepad1.dpad_up)    snapTarget = Math.toRadians(0);
            if (gamepad1.dpad_left)  snapTarget = Math.toRadians(90);
            if (gamepad1.dpad_down)  snapTarget = Math.toRadians(180);
            if (gamepad1.dpad_right) snapTarget = Math.toRadians(-90);

            // Touching the turn stick cancels a snap so the driver is always in control
            if (turn != 0) {
                snapTarget = null;
            }

            double headingError = 0;
            if (snapTarget != null) {
                headingError = angleWrap(snapTarget - robot.getHeading());
                if (Math.abs(headingError) < Math.toRadians(SNAP_TOLERANCE_DEG)) {
                    snapTarget = null;   // arrived
                } else {
                    turn = Range.clip(SNAP_KP * headingError, -SNAP_MAX_TURN, SNAP_MAX_TURN);
                }
            }

            // RIGHT BUMPER (hold): auto-turn to face the AprilTag (overrides snapping)
            LLResult target = robot.getTarget();
            if (gamepad1.right_bumper && target != null) {
                snapTarget = null;
                turn = Range.clip(-AIM_KP * target.getTx(), -0.5, 0.5);
            }

            // LEFT TRIGGER (hold): slow mode
            double scale = gamepad1.left_trigger > 0.5 ? SLOW_SPEED : 1.0;

            robot.driveFieldCentric(forward * scale, strafe * scale, turn * scale);

            telemetry.addData("Heading (deg)", Math.toDegrees(robot.getHeading()));
            telemetry.addData("Snapping to", snapTarget == null ? "-" : Math.toDegrees(snapTarget));
            telemetry.addData("Tag seen", target != null);
            if (target != null) {
                telemetry.addData("tx", target.getTx());
            }
            telemetry.update();
        }

        robot.stop();
    }

    /** Keeps an angle between -180 and 180 degrees (in radians) so it turns the short way. */
    private static double angleWrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}