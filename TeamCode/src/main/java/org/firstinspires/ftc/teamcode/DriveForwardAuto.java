package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Drive Forward", group = "Auto")
public class DriveForwardAuto extends LinearOpMode {

    // Tweak these to change how far it goes
    static final double DRIVE_POWER = 0.5;   // 0.0 - 1.0
    static final long   DRIVE_TIME_MS = 2000; // milliseconds

    @Override
    public void runOpMode() {
        // Names must match your Robot Configuration on the Driver Station
        DcMotor a = hardwareMap.get(DcMotor.class, "a"); // front left
        DcMotor b = hardwareMap.get(DcMotor.class, "b"); // front right
        DcMotor c = hardwareMap.get(DcMotor.class, "c"); // back left
        DcMotor d = hardwareMap.get(DcMotor.class, "d"); // back right

        // Left side motors (a, c) are mounted mirrored, so reverse them.
        // If the robot spins instead of going straight, swap which side is reversed.
        a.setDirection(DcMotorSimple.Direction.REVERSE);
        c.setDirection(DcMotorSimple.Direction.REVERSE);
        b.setDirection(DcMotorSimple.Direction.FORWARD);
        d.setDirection(DcMotorSimple.Direction.FORWARD);

        a.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        c.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        d.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Go forward
        a.setPower(DRIVE_POWER);
        b.setPower(DRIVE_POWER);
        c.setPower(DRIVE_POWER);
        d.setPower(DRIVE_POWER);

        sleep(DRIVE_TIME_MS);

        // Stop
        a.setPower(0);
        b.setPower(0);
        c.setPower(0);
        d.setPower(0);
    }
}
