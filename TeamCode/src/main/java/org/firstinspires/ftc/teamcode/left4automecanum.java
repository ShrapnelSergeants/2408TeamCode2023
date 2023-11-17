package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name="left4Auto Mecanum", group="Auto")
@Disabled

public class left4automecanum extends LinearOpMode{


    RobotHardware robot = new RobotHardware(this);

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode(){


        robot.init();
        waitForStart();



        robot.driveMecanum(1);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 1.7) {
            telemetry.addData("Step 1", "Current runtime:" + runtime.seconds() + "/1");
            telemetry.update();

        }

        robot.resetWheels();
        robot.stop();

        telemetry.addData("Robot Status:", "Complete");
        telemetry.update();


        sleep(1000);
    }






}
