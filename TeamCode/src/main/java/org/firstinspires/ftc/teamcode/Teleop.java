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
            double turn = -gamepad1.right_stick_x; // Added right stick for rotation

            double magnitude = Math.hypot(forward, strafe);

            // If sticks are centered and there's no rotation command, stop the robot
            if (magnitude < STICK_DEADZONE && Math.abs(turn) < STICK_DEADZONE) {
                robot.stop();
            } else {
                // Pass forward, strafe, and turn into the robot drive method
                robot.drive(forward, strafe, turn);
            }

            telemetry.addData("Stick angle", Math.toDegrees(Math.atan2(strafe, forward)));
            telemetry.addData("Speed", Math.min(magnitude, 1));
            telemetry.addData("Turn", turn);
            telemetry.update();
        }

        robot.stop();
    }
}