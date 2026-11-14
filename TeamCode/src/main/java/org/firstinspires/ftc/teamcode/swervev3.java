package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "swervev3 (Blocks to Java)")
public class swervev3 extends LinearOpMode {

    private CRServo as;
    private CRServo bs;
    private CRServo cs;
    private CRServo ds;
    private DcMotor a;
    private DcMotor b;
    private DcMotor c;
    private DcMotor d;
    private AnalogInput aa;
    private AnalogInput ba;
    private AnalogInput ca;
    private AnalogInput da;

    List servoangle;
    double JoystickAngle;

    /**
     * Describe this function...
     */
    private void set_power(double servoname, double power) {
        double bp;

        bp = power;
        power = 0.123123;
        power = bp;
        if (servoname == 1) {
            as.setPower(power);
        } else if (servoname == 2) {
            bs.setPower(power);
        } else if (servoname == 3) {
            cs.setPower(power);
        } else if (servoname == 4) {
            ds.setPower(power);
        }
    }

    /**
     * This OpMode offers POV (point-of-view) style TeleOp control for a direct drive robot.
     *
     * In this POV mode, the left joystick (up and down) moves the robot forward and back, and the
     * right joystick (left and right) spins the robot left (counterclockwise) and right (clockwise).
     */
    @Override
    public void runOpMode() {
        List diff;
        double i;
        List offsets;

        as = hardwareMap.get(CRServo.class, "as");
        bs = hardwareMap.get(CRServo.class, "bs");
        cs = hardwareMap.get(CRServo.class, "cs");
        ds = hardwareMap.get(CRServo.class, "ds");
        a = hardwareMap.get(DcMotor.class, "a");
        b = hardwareMap.get(DcMotor.class, "b");
        c = hardwareMap.get(DcMotor.class, "c");
        d = hardwareMap.get(DcMotor.class, "d");
        aa = hardwareMap.get(AnalogInput.class, "aa");
        ba = hardwareMap.get(AnalogInput.class, "ba");
        ca = hardwareMap.get(AnalogInput.class, "ca");
        da = hardwareMap.get(AnalogInput.class, "da");

        // Reverse one of the drive motors.
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        a.setDirection(DcMotor.Direction.FORWARD);
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        b.setDirection(DcMotor.Direction.FORWARD);
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        c.setDirection(DcMotor.Direction.FORWARD);
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        d.setDirection(DcMotor.Direction.REVERSE);
        as.setDirection(CRServo.Direction.FORWARD);
        bs.setDirection(CRServo.Direction.FORWARD);
        cs.setDirection(CRServo.Direction.FORWARD);
        ds.setDirection(CRServo.Direction.FORWARD);
        servoangle = JavaUtil.createListWith(0, 0, 0, 0);
        diff = JavaUtil.createListWith(0, 0, 0, 0);
        offsets = JavaUtil.createListWith(50, 70, 10, 70);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                // Use left stick to drive and right stick to turn
                JoystickAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) / Math.PI * 180;
                servoangle = JavaUtil.createListWith(aa.getVoltage() * (360 / aa.getMaxVoltage()) - 180, ba.getVoltage() * (360 / ba.getMaxVoltage()) - 180, ca.getVoltage() * (360 / ca.getMaxVoltage()) - 180, da.getVoltage() * (360 / da.getMaxVoltage()) - 180);
                for (i = 1; i <= 4; i++) {
                    JavaUtil.inListSet(servoangle, JavaUtil.AtMode.FROM_START, (int) (i - 1), false, ((Double) JavaUtil.inListGet(servoangle, JavaUtil.AtMode.FROM_START, (int) (i - 1), false)).doubleValue() + ((Double) JavaUtil.inListGet(offsets, JavaUtil.AtMode.FROM_START, (int) (i - 1), false)).doubleValue());
                    if (JavaUtil.inListGet(servoangle, JavaUtil.AtMode.FROM_START, (int) (i - 1), false) > 180) {
                        JavaUtil.inListSet(servoangle, JavaUtil.AtMode.FROM_START, (int) (i - 1), false, ((Double) JavaUtil.inListGet(servoangle, JavaUtil.AtMode.FROM_START, (int) (i - 1), false)).doubleValue() - 360);
                    } else if (JavaUtil.inListGet(servoangle, JavaUtil.AtMode.FROM_START, (int) (i - 1), false) < -180) {
                        JavaUtil.inListSet(servoangle, JavaUtil.AtMode.FROM_START, (int) (i - 1), false, ((Double) JavaUtil.inListGet(servoangle, JavaUtil.AtMode.FROM_START, (int) (i - 1), false)).doubleValue() + 360);
                    }
                    JavaUtil.inListSet(diff, JavaUtil.AtMode.FROM_START, (int) (i - 1), false, JoystickAngle - ((Double) JavaUtil.inListGet(servoangle, JavaUtil.AtMode.FROM_START, (int) (i - 1), false)).doubleValue());
                    if (JavaUtil.inListGet(diff, JavaUtil.AtMode.FROM_START, (int) (i - 1), false) > 180) {
                        JavaUtil.inListSet(diff, JavaUtil.AtMode.FROM_START, (int) (i - 1), false, ((Double) JavaUtil.inListGet(diff, JavaUtil.AtMode.FROM_START, (int) (i - 1), false)).doubleValue() - 360);
                    } else if (JavaUtil.inListGet(diff, JavaUtil.AtMode.FROM_START, (int) (i - 1), false) < -180) {
                        JavaUtil.inListSet(diff, JavaUtil.AtMode.FROM_START, (int) (i - 1), false, ((Double) JavaUtil.inListGet(diff, JavaUtil.AtMode.FROM_START, (int) (i - 1), false)).doubleValue() + 360);
                    }
                    if (Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2)) >= 0.2 && Math.abs(((Double) JavaUtil.inListGet(diff, JavaUtil.AtMode.FROM_START, (int) (i - 1), false)).doubleValue()) >= 20) {
                        set_power(i, (((Double) JavaUtil.inListGet(diff, JavaUtil.AtMode.FROM_START, (int) (i - 1), false)).doubleValue() / Math.abs(((Double) JavaUtil.inListGet(diff, JavaUtil.AtMode.FROM_START, (int) (i - 1), false)).doubleValue())) * 0.3);
                    } else {
                        set_power(i, 0);
                    }
                }
                if (gamepad1.rightTriggerWasPressed()) {
                    a.setPower(0.5);
                    b.setPower(0.5);
                    c.setPower(0.5);
                    d.setPower(0.5);
                }
                if (gamepad1.rightTriggerWasReleased()) {
                    a.setPower(0);
                    b.setPower(0);
                    c.setPower(0);
                    d.setPower(0);
                }
                telemetry2();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void telemetry2() {
        telemetry.addData("joystick angle", JoystickAngle);
        telemetry.addData("joystick magnitude", Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2)));
        telemetry.addData("servo 1 angle", ((Double) JavaUtil.inListGet(servoangle, JavaUtil.AtMode.FROM_START, (int) 0, false)).doubleValue());
        telemetry.addData("servo 2 angle", ((Double) JavaUtil.inListGet(servoangle, JavaUtil.AtMode.FROM_START, (int) 1, false)).doubleValue());
        telemetry.addData("servo 3 angle", ((Double) JavaUtil.inListGet(servoangle, JavaUtil.AtMode.FROM_START, (int) 2, false)).doubleValue());
        telemetry.addData("servo 4 angle", ((Double) JavaUtil.inListGet(servoangle, JavaUtil.AtMode.FROM_START, (int) 3, false)).doubleValue());
        telemetry.update();
    }
}
