package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueAutoTop", group = "Autonomous")
public class BlueAutoTop extends AutoBase {
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        driveFor(-0.5, 0, 0, 1700);
        // aimAtTag(2000);

        robot.stop();
    }
}