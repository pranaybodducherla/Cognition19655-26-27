package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "RedAutoTop", group = "Autonomous")
public class RedAutoTop extends LinearOpMode {

    // SAME HARDWARE AS TELEOP
    private Limelight3A Webcam1;
    private DcMotor upperLeft, upperRight, lowerLeft, lowerRight;
    private DcMotor conveyorBelt;
    private DcMotor turretLeft, turretRight;
    private DcMotor intakeMotor; // unused
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
        // MOTOR DIRECTIONS (SAME AS TELEOP)
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
        // MOVE BACKWARD
        // -------------------------------
        //conveyorBelt.setPower(1);

        setDrivePower(-0.5, 0, 0);
        sleep(1700);
        setDrivePower(0, 0, 0);

        // -------------------------------
        // AUTO AIM + OUTTAKE (TELEOP EXACT)
        // -------------------------------
        long startTime = System.currentTimeMillis();

        while (opModeIsActive()
                && System.currentTimeMillis() - startTime < 4000) {

            result = Webcam1.getLatestResult();

            if (result != null && result.getBotposeAvgDist() > 0) {

                double tx = result.getTx();
                double distance = result.getBotposeAvgDist();

                // SAME AS TELEOP
                double autoyaw = tx * 0.01;

                double outtakePower = Math.min(
                        Math.max(distance * 0.155357 + 0.354, 0.53),
                        1
                );

                // 🔹 HORIZONTAL AIM = DRIVE YAW
                if (Math.abs(tx) > 1.0) {
                    setDrivePower(0, 0, autoyaw);
                } else {
                    setDrivePower(0, 0, 0);
                }

                // 🔹 OUTTAKE POWER (SAME AS TELEOP Turrent)
                turretLeft.setPower(-outtakePower);
                turretRight.setPower(-outtakePower);

                telemetry.addData("tx", tx);
                telemetry.addData("dist", distance);
                telemetry.addData("outtakePower", outtakePower);
            }

            telemetry.update();
        }

        // STOP ROTATION (KEEP OUTTAKE SPINNING)
        setDrivePower(0, 0, 0);
        sleep(2000);

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
