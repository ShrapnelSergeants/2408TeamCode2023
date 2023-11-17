package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


@Autonomous(name="Left 1", group="New")
//@Disabled
public class left1 extends LinearOpMode {

    /* Declare OpMode members.*/
    private DcMotor         left_back_drive   = null;
    private DcMotor         left_front_drive  = null;
    private DcMotor         right_back_drive   = null;
    private DcMotor         right_front_drive   = null;

    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.

    static final double     COUNTS_PER_MOTOR_REV    = 140 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        left_back_drive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        left_front_drive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        right_front_drive = hardwareMap.get(DcMotor.class, "right_front_drive");
        right_back_drive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        left_back_drive.setDirection(DcMotor.Direction.REVERSE);
        right_back_drive.setDirection(DcMotor.Direction.FORWARD);
        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        right_front_drive.setDirection(DcMotor.Direction.FORWARD);

        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                left_front_drive.getCurrentPosition(),
                left_back_drive.getCurrentPosition(),
                right_back_drive.getCurrentPosition(),
                right_front_drive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(TURN_SPEED,   -59, 59, 4.0);  // S2: Turn left 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED,  189,  189, 5.0);  // S1: Forward 48 Inches with 5 Sec timeout


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }


    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        //
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = left_front_drive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftBackTarget = left_back_drive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = right_front_drive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightBackTarget = right_back_drive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            left_front_drive.setTargetPosition(newLeftFrontTarget);
            left_back_drive.setTargetPosition(newLeftFrontTarget);
            right_front_drive.setTargetPosition(newRightFrontTarget);
            right_back_drive.setTargetPosition(newRightBackTarget);


            // Turn On RUN_TO_POSITION
            left_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            left_front_drive.setPower(Math.abs(speed));
            left_back_drive.setPower(Math.abs(speed));
            right_front_drive.setPower(Math.abs(speed));
            right_back_drive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (left_front_drive.isBusy() && left_back_drive.isBusy() && right_front_drive.isBusy() && right_back_drive.isBusy())) {

                //Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftFrontTarget,  newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        left_front_drive.getCurrentPosition(), left_back_drive.getCurrentPosition(), right_front_drive.getCurrentPosition(), right_back_drive.getCurrentPosition());
                telemetry.update();

            }

            // Stop all motion;
            left_front_drive.setPower(0);
            left_back_drive.setPower(0);
            right_front_drive.setPower(0);
            right_back_drive.setPower(0);

            // Turn off RUN_TO_POSITION
            left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.

        }

    }


}
