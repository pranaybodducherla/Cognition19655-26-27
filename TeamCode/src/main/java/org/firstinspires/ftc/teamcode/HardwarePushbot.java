package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class HardwarePushbot {
    public DcMotor upperLeft = null;
    public DcMotor upperRight = null;
    public DcMotor lowerLeft = null;
    public DcMotor lowerRight = null;
    public DcMotor intakeMotor = null;
    public DcMotor turretLeft = null;
    public DcMotor turretRight = null;
    public DcMotor conveyorBelt = null;
    public CRServo aimingServo = null;
    public Servo turretServo = null;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    final double INTAKE_OFF = 0.0;
    final double conveyorBeltSpeed = -1.0;
    final double TURRET_SERVO_START = 0.5;
    final double TURRET_SERVO_UP = 0.8;
    final double TURRET_SERVO_STOP = 0.5;
    final double TURRET_SERVO_DOWN = 0.2;

    public HardwarePushbot() {}

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        upperLeft = hwMap.dcMotor.get("upperLeft");
        upperRight = hwMap.dcMotor.get("upperRight");
        lowerLeft = hwMap.dcMotor.get("lowerLeft");
        lowerRight = hwMap.dcMotor.get("lowerRight");

        intakeMotor = hwMap.dcMotor.get("intakeMotor");
        turretLeft = hwMap.dcMotor.get("turretLeft");
        turretRight = hwMap.dcMotor.get("turretRight");
        conveyorBelt = hwMap.dcMotor.get("conveyorBelt");
        aimingServo = hwMap.crservo.get("aimingServo");
        turretServo = hwMap.servo.get("turretServo");

        // Motor directions for TeleOp layout
        upperLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        upperRight.setDirection(DcMotorSimple.Direction.FORWARD);
        lowerLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        lowerRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Zero motors
        zero();

        // Initialize servos
        turretServo.setPosition(TURRET_SERVO_START);
        conveyorBelt.setPower(0);
        intakeMotor.setPower(INTAKE_OFF);
    }

    // --- MOVEMENT METHODS ---
    public void zero() {
        upperLeft.setPower(0);
        upperRight.setPower(0);
        lowerLeft.setPower(0);
        lowerRight.setPower(0);
    }

    public void forward(double speed) {
        upperLeft.setPower(speed);
        upperRight.setPower(speed);
        lowerLeft.setPower(speed);
        lowerRight.setPower(speed);
    }

    public void backward(double speed) {
        forward(-speed);
    }

    public void turnLeft(double speed) {
        upperLeft.setPower(speed);
        upperRight.setPower(-speed);
        lowerLeft.setPower(speed);
        lowerRight.setPower(-speed);
    }

    public void turnRight(double speed) {
        turnLeft(-speed);
    }

    // --- INTAKE ---
    public void intakeStart() { intakeMotor.setPower(0.8); }
    public void intakeStop() { intakeMotor.setPower(0); }

    // --- OUTTAKE ---
    public void outTakeOn(double power) {
        turretRight.setPower(power);
        turretLeft.setPower(-power);
    }

    public void outTakeOff() {
        turretRight.setPower(0);
        turretLeft.setPower(0);
    }

    // --- CONVEYOR ---
    public void conveyorBeltOn() { conveyorBelt.setPower(conveyorBeltSpeed); }
    public void conveyorBeltOff() { conveyorBelt.setPower(0); }

    // --- TURRET SERVO ---
    public void turretServoBack() { turretServo.setPosition(TURRET_SERVO_UP); }
    public void turretServoForward() { turretServo.setPosition(TURRET_SERVO_DOWN); }
    public void turretServoStop() { turretServo.setPosition(TURRET_SERVO_STOP); }

    // --- AIMING SERVO ---
    public void turretAim(double power) { aimingServo.setPower(power); }
    public void turretAimStop() { aimingServo.setPower(0); }
}
