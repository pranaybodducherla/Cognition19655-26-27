package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "ServoTest", group = "Test")
public class ServoTest extends LinearOpMode {

    private static final String[] NAMES = {"as", "bs", "cs", "ds"};

    @Override
    public void runOpMode() {
        ServoImplEx[] servos = new ServoImplEx[NAMES.length];
        for (int i = 0; i < NAMES.length; i++) {
            servos[i] = hardwareMap.get(ServoImplEx.class, NAMES[i]);
            servos[i].setPwmRange(new PwmRange(500, 2500));
        }

        telemetry.addLine("D-pad left = 0.25, up = 0.5, right = 0.75");
        telemetry.update();
        waitForStart();

        double position = 0.5;
        while (opModeIsActive()) {
            if (gamepad1.dpad_left) position = 0.25;
            if (gamepad1.dpad_up) position = 0.5;
            if (gamepad1.dpad_right) position = 0.75;

            for (ServoImplEx s : servos) s.setPosition(position);

            telemetry.addData("Position", position);
            telemetry.update();
        }
    }
}