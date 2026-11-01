package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "RedAutoBottom", group = "Autonomous")
public class RedAutoBottom extends LinearOpMode {

    // SAME HARDWARE AS TELEOP
    private Limelight3A Webcam1;
    private DcMotor upperLeft, upperRight, lowerLeft, lowerRight;
    private DcMotor conveyorBelt;
    private DcMotor turretLeft, turretRight; // OUTTAKE MOTORS
    private DcMotor intakeMotor;
    private CRServo claw;

    LLResult result;

    @Override
    public void runOpMode() throws InterruptedException {

        // -------------------------------
        // HARDWARE MAP
        // -------------------------------
        Webcam1 = hardwareMap.get(Limelight3A.class, "Webcam1");

        upperLeft = hardwareMap.get(DcMotor.class, "upperLeft");
        upperRight = hardwareMap.get(DcMotor.class, "upperRight");
        lowerLeft = hardwareMap.get(DcMotor.class, "lowerLeft");
        lowerRight = hardwareMap.get(DcMotor.class, "lowerRight");

        conveyorBelt = hardwareMap.get(DcMotor.class, "conveyorBelt");
        turretLeft = hardwareMap.get(DcMotor.class, "turretLeft");
        turretRight = hardwareMap.get(DcMotor.class, "turretRight");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        claw = hardwareMap.get(CRServo.class, "claw");

        // -------------------------------
        // MOTOR DIRECTIONS (MATCH TELEOP)
        // -------------------------------
        upperLeft.setDirection(DcMotor.Direction.REVERSE);
        upperRight.setDirection(DcMotor.Direction.FORWARD);
        lowerLeft.setDirection(DcMotor.Direction.FORWARD);
        lowerRight.setDirection(DcMotor.Direction.REVERSE);

        conveyorBelt.setDirection(DcMotor.Direction.REVERSE);
        turretLeft.setDirection(DcMotor.Direction.REVERSE);
        turretRight.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        // -------------------------------
        // LIMELIGHT INIT
        // -------------------------------
        Webcam1.pipelineSwitch(0);
        Webcam1.start();

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        // -------------------------------
        // MOVE FORWARD
        // -------------------------------
        //conveyorBelt.setPower(1);

        setDrivePower(0.5, 0, 0);
        sleep(1600);
        setDrivePower(0, 0, 0);
        setDrivePower(0.5, 0, 0);
        sleep(300);
        setDrivePower(0, 0, 0);
        // -------------------------------
        // TURN LEFT
        // -------------------------------
        setDrivePower(0, 0, -0.15);
        sleep(1200);
        setDrivePower(0, 0, 0);

        // -------------------------------
        // AUTO AIM (DRIVETRAIN YAW — SAME AS TELEOP)
        // -------------------------------
        long startTime = System.currentTimeMillis();

        while (opModeIsActive()
                && System.currentTimeMillis() - startTime < 4000) {

            result = Webcam1.getLatestResult();

            if (result != null && result.getBotposeAvgDist() > 0) {

                double tx = result.getTx();
                double autoyaw = tx * 0.01;

                if (Math.abs(tx) > 1.0) {
                    setDrivePower(0, 0, autoyaw);
                } else {
                    setDrivePower(0, 0, 0);
                }

                telemetry.addData("tx", tx);
            }

            telemetry.update();
        }

        setDrivePower(0, 0, 0);
        sleep(300);

        // -------------------------------
        // CALCULATE OUTTAKE POWER (TELEOP FORMULA)
        // -------------------------------
        double shootPower = 0.7; // fallback

        result = Webcam1.getLatestResult();
        if (result != null && result.getBotposeAvgDist() > 0) {
            shootPower = Math.min(
                    Math.max(result.getBotposeAvgDist() * 0.155357 + 0.354, 0.53),
                    1
            );
        }

        // -------------------------------
        // SPIN UP SHOOTER
        // -------------------------------
        turretLeft.setPower(-shootPower);
        turretRight.setPower(-shootPower);
        sleep(1500); // spin-up time

        // -------------------------------
        // SHOOT #1
        // -------------------------------
        claw.setPower(1);
        sleep(600);
        claw.setPower(-1);
        sleep(600);
        claw.setPower(0);

        // -------------------------------
        // SHOOT #2
        // -------------------------------
        sleep(800);
        claw.setPower(1);
        sleep(600);
        claw.setPower(-1);
        sleep(600);
        claw.setPower(0);

        // -------------------------------
        // SHUTDOWN
        // -------------------------------
        turretLeft.setPower(0);
        turretRight.setPower(0);
        conveyorBelt.setPower(0);

        telemetry.addData("Status", "Auto Complete");
        telemetry.update();
    }

    // -------------------------------
    // SAME DRIVE MATH AS TELEOP
    // -------------------------------
    private void setDrivePower(double axial, double lateral, double yaw) {
        double fl = axial + lateral + yaw;
        double fr = (axial - lateral) - yaw;
        double bl = (axial - lateral) + yaw;
        double br = (axial + lateral) - yaw;

        double max = Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))
        );

        if (max > 1) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        upperLeft.setPower(fl);
        upperRight.setPower(fr);
        lowerLeft.setPower(bl);
        lowerRight.setPower(br);
    }
}
