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
            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double magnitude = Math.hypot(forward, strafe);

            if (magnitude < STICK_DEADZONE) {
                robot.stop();
            } else {
                // All wheels point where the stick points; speed scales with how far it is pushed
                robot.drive(forward, strafe, 0);
            }

            telemetry.addData("Stick angle", Math.toDegrees(Math.atan2(strafe, forward)));
            telemetry.addData("Speed", Math.min(magnitude, 1));
            telemetry.update();
        }

        robot.stop();
    }
}