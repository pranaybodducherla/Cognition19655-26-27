package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends LinearOpMode {

    private static final double STICK_DEADZONE = 0.05;

    private final HardwarePushbot robot = new HardwarePushbot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Left stick: direction and speed. Right stick X: rotation. Both can be used together.
            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            // Zero out each stick separately so drift on one doesn't affect the other
            if (Math.hypot(forward, strafe) < STICK_DEADZONE) {
                forward = 0;
                strafe = 0;
            }
            if (Math.abs(turn) < STICK_DEADZONE) {
                turn = 0;
            }

            robot.drive(forward, strafe, turn);

            telemetry.addData("Forward", forward);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Turn", turn);
            telemetry.update();
        }

        robot.stop();
    }
}