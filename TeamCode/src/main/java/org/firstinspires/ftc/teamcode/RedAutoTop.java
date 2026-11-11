package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedAutoTop", group = "Autonomous")
public class RedAutoTop extends AutoBase {
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        driveFor(-0.5, 0, 0, 1700);
        // aimAtTag(2000);   // uncomment to aim at the tag at the end

        robot.stop();
    }
}