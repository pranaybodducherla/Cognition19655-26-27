package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MotorTest", group = "Test")
public class MotorTest extends LinearOpMode {

    private static final double POWER = 0.3;

    @Override
    public void runOpMode() {
        DcMotor upperLeft = hardwareMap.get(DcMotor.class, "a");
        DcMotor upperRight = hardwareMap.get(DcMotor.class, "b");
        DcMotor lowerLeft = hardwareMap.get(DcMotor.class, "c");
        DcMotor lowerRight = hardwareMap.get(DcMotor.class, "d");

        telemetry.addLine("Y = upperLeft, B = upperRight, X = lowerLeft, A = lowerRight");
        telemetry.addLine("Right bumper = all four");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            boolean all = gamepad1.right_bumper;

            upperLeft.setPower(all || gamepad1.y ? POWER : 0);
            upperRight.setPower(all || gamepad1.b ? POWER : 0);
            lowerLeft.setPower(all || gamepad1.x ? POWER : 0);
            lowerRight.setPower(all || gamepad1.a ? POWER : 0);

            telemetry.addData("upperLeft", upperLeft.getCurrentPosition());
            telemetry.addData("upperRight", upperRight.getCurrentPosition());
            telemetry.addData("lowerLeft", lowerLeft.getCurrentPosition());
            telemetry.addData("lowerRight", lowerRight.getCurrentPosition());
            telemetry.update();
        }
    }
}