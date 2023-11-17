package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Mecanum Teleop", group="Linear Opmode")
//@Disabled
    public class mecanumteleop extends LinearOpMode {

        // Declare OpMode members for each of the 4 motors.
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor leftFrontDrive = null;
        private DcMotor leftBackDrive = null;
        private DcMotor rightFrontDrive = null;
        private DcMotor rightBackDrive = null;
        private DcMotor elevatorDrive = null;
        //private DcMotor arm_left = null;
        //private DcMotor arm_right = null;
        //private DcMotor arm_upper = null;
        private Servo claw = null;

        @Override
        public void runOpMode() {

            // Initialize the hardware variables. Note that the strings used here must correspond
            // to the names assigned during the robot configuration step on the DS or RC devices.
            leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");//"left_front_drive
            leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");//left_back_drive
            rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");//right_front_drive
            rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");//right_back_drive
            //elevatorDrive = hardwareMap.get(DcMotor.class, "elevator_drive");
            //arm_right = hardwareMap.get(DcMotor.class, "arm_right");
            //arm_left = hardwareMap.get(DcMotor.class, "arm_left");
            //arm_upper = hardwareMap.get(DcMotor.class, "arm_upper");
            //claw = hardwareMap.get(Servo.class, "claw");


            // ########################################################################################
            // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
            // ########################################################################################
            // Most robots need the motors on one side to be reversed to drive forward.
            // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
            // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
            // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
            // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
            // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
            // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);//forward
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);//for// ward
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);//reverse
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);//reverse
            //arm_right.setDirection(DcMotor.Direction.REVERSE);
            //arm_upper.setDirection(DcMotor.Direction.REVERSE);

            // Wait for the game to start (driver presses PLAY)
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            waitForStart();
            runtime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                double max;

                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                double axial   = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                double lateral = -gamepad1.left_stick_x; //Note: added negative to correct controls
                double yaw     =  gamepad1.right_stick_x;


                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                // Set up a variable for each drive wheel to save the power level for telemetry.
                double leftFrontPower  = axial + lateral - yaw;//+ +
                double rightFrontPower = axial - lateral + yaw;//- -
                double leftBackPower   = axial - lateral - yaw;//- +
                double rightBackPower  = axial + lateral + yaw;//+ -

                // Normalize the values so no wheel power exceeds 100%
                // This ensures that the robot maintains the desired motion.
                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max > 1.0) {
                    leftFrontPower  /= max;
                    rightFrontPower /= max;
                    leftBackPower   /= max;
                    rightBackPower  /= max;
                }

                // This is test code:
                //
                // Uncomment the following code to test your motor directions.
                // Each button should make the corresponding motor run FORWARD.
                //   1) First get all the motors to take to correct positions on the robot
                //      by adjusting your Robot Configuration if necessary.
                //   2) Then make sure they run in the correct direction by modifying the
                //      the setDirection() calls above.
                // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */
                RobotHardware robot = new RobotHardware(this);

                /*if(gamepad2.a) {
                    arm_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    arm_left.setPower(0.25);
                    arm_right.setPower(0.25);
                    arm_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                } else if(gamepad2.b){
                    arm_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    arm_left.setPower(-0.25);
                    arm_right.setPower(-0.25);
                    arm_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                } else if(gamepad2.right_bumper){
                    arm_upper.setPower(0.5);
                    arm_upper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                } else if(gamepad2.left_bumper){
                    arm_upper.setPower(-0.5);
                    arm_upper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                } else {
                    arm_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    arm_left.setPower(0);
                    arm_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    arm_right.setPower(0);
                    arm_upper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    arm_upper.setPower(0);
                    '][

                }
                if(gamepad2.right_trigger >= 0.3) {
       //             claw.setPosition(1.0);
                } else {
       //             claw.setPosition(0);//place holder
                }*/

                // Send calculated power to wheels
                leftFrontDrive.setPower(leftFrontPower /2);
                rightFrontDrive.setPower(rightFrontPower /2);
                leftBackDrive.setPower(leftBackPower /2);
                rightBackDrive.setPower(rightBackPower /2);

               // if(gamepad1.left_trigger > .1){
                 //   elevatorDrive.setPower(gamepad1.left_trigger);
              //  } else if(gamepad1.right_trigger > .1){
                   // elevatorDrive.setPower(-gamepad1.right_trigger);
              //  }



                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
                telemetry.update();
            }
        }
}
