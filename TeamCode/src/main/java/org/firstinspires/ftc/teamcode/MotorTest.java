package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/** Press and hold one button to test that wheel. Right bumper tests all wheels. */
@TeleOp(name = "Motor Test", group = "Test")
public class MotorTest extends LinearOpMode {
    private static final double POWER = 0.25;

    @Override
    public void runOpMode() {
        HardwarePushbot robot = new HardwarePushbot(hardwareMap);
        DcMotor[] motors = {robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight};
        String[] names = {"front-left (a)", "front-right (b)", "back-left (c)", "back-right (d)"};

        telemetry.addLine("Hold Y/B/X/A to test FL/FR/BL/BR. Hold RB for all.");
        telemetry.addLine("Test with wheels raised; confirm direction before driving.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            boolean all = gamepad1.right_bumper;
            boolean[] pressed = {gamepad1.y, gamepad1.b, gamepad1.x, gamepad1.a};
            for (int i = 0; i < motors.length; i++) {
                motors[i].setPower(all || pressed[i] ? POWER : 0);
                telemetry.addData(names[i], "power %.2f, ticks %d",
                        all || pressed[i] ? POWER : 0, motors[i].getCurrentPosition());
            }
            telemetry.update();
        }
        robot.stop();
    }
}
