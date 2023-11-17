//package org.firstinspires.ftc.teamcode;

/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
/**
 * Demonstrates empty OpMode
 */
@Autonomous(name="Auto Mecanum w IMU", group="Auto")
@Disabled

public class AutoWithIMU extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    private ElapsedTime runtime = new ElapsedTime();
    IMU imu= null;


    private double headingError= 0;

    private double targetHeading= 0;
    private double driveSpeed= 0;
    private double turnSpeed= 0;
    private double leftSpeed= 0;
    private double rightSpeed= 0;
    private int leftTarget= 0;
    private int rightTarget= 0;

    static final double DRIVER_SPEED = 0.4;
    static final double TURN_SPEED = 0.2;
    static final double HEADING_THRESHOLD = 1.0;
    static final double P_TURN_GAIN = 0.02;
    static final double P_DRIVE_GAIN = 0.03;


    @Override
    public void runOpMode() {

        robot.init();

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection= RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection= RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot= new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        while(opModeInInit()){
            telemetry.addData(">","robot Heading = %4.0f", getHeading());
            telemetry.update();
        }
        imu.resetYaw();
        waitForStart();

        robot.driveMecanum(.5);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds()<0.90){//2
            telemetry.addData("Step 1", "Current runtime: "+ runtime.seconds()+"/3");//3
            telemetry.update();
        }
        /*
        robot.spinMecanum(.15);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 2.2){//2
            telemetry.addData("Step 2", "Current runtime: "+ runtime.seconds()+"/1");//1
            telemetry.update();
        }*/
        turnToHeading(0.5,90);
        runtime.reset();
        

        robot.resetWheels();
        robot.stop();

        telemetry.addData("Robot Status:", "Complete");
        telemetry.update();

        sleep(1000);

    }

    public void turnToHeading(double maxTurnSpeed,double heading){
        getSteeringCorrection(heading,P_TURN_GAIN);
        while(opModeIsActive()&&(Math.abs(headingError)>HEADING_THRESHOLD)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            robot.spinMecanum(.15);

        }
        robot.spinMecanum(0);
    }
    public double getHeading(){
        YawPitchRollAngles orientation= imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    public double getSteeringCorrection(double desiredHeading, double proportionalGain){
        targetHeading=desiredHeading;
        headingError=targetHeading-getHeading();

        while(headingError>180) headingError-=360;
        while(headingError<=180) headingError+=360;

        return Range.clip(headingError*proportionalGain,-1,1);
    }
    public void holdHeading(double maxTurnSpeed,double heading,double holdTime){
        ElapsedTime holdTimer=new ElapsedTime();
        holdTimer.reset();

        while(opModeIsActive()&& holdTimer.time()<holdTime){
            turnSpeed= getSteeringCorrection(heading,P_TURN_GAIN);
            turnSpeed= Range.clip(turnSpeed,-maxTurnSpeed,maxTurnSpeed);

            robot.spinMecanum(turnSpeed);
        }
        robot.spinMecanum(0);
    }

}


//driveStraight?
//moveRobot
