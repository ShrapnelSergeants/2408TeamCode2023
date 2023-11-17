package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name="left1Auto Mecanum", group="Auto")
@Disabled

public class left1automecanum extends LinearOpMode{

    RobotHardware robot = new RobotHardware(this);

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode(){

        robot.init();
        waitForStart();


        robot.spinMecanum(.15);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 2.4){
            telemetry.addData("Step 1", "Current runtime:"+ runtime.seconds()+ "/3");
            telemetry.update();

        }
        robot.driveMecanum(.5);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds()<1.5){
            telemetry.addData("Step 2", "Current runtime: "+ runtime.seconds()+"/3");
            telemetry.update();
        }

        robot.resetWheels();
        robot.stop();

        telemetry.addData("Robot Status:", "Complete");
        telemetry.update();

        sleep(1000);


    }
}
