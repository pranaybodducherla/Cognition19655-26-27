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

    private final HardwarePushbot robot = new HardwarePushbot();

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
            }

            // RIGHT BUMPER (hold): auto-turn to face the AprilTag
            LLResult target = robot.getTarget();
            if (gamepad1.right_bumper && target != null) {
                turn = Range.clip(-AIM_KP * target.getTx(), -0.5, 0.5);
            }

            // LEFT TRIGGER (hold): slow mode
            double scale = gamepad1.left_trigger > 0.5 ? SLOW_SPEED : 1.0;

            robot.driveFieldCentric(forward * scale, strafe * scale, turn * scale);

            telemetry.addData("Heading (deg)", Math.toDegrees(robot.getHeading()));
            telemetry.addData("Tag seen", target != null);
            if (target != null) {
                telemetry.addData("tx", target.getTx());
            }
            telemetry.update();
        }

        robot.stop();
    }
}