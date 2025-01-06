package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Mecanum Auto OpMode", group = "Autonomous")
public class MechBotAutoModeEncoder extends LinearOpMode {

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: NeveRest40Gearmotor Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    // Update power and travel time accordingly
    static final double ROBOT_TRAVEL_POWER = 0.5;  // in range between -1 to 1

    static final double ROBOT_TRAVEL_TIME_IN_SECONDS = 12.0;
    static final double ROBOT_TRAVEL_DISTANCE_IN_INCHES = 24.0;


    static final double VIPER_POWER = 0.5; // in range between -1 to 1
    static final int VIPER_SLIDE_TARGET_VERTICAL_POSITION = 100;
    static final int VIPER_SLIDE_TARGET_HORIZONTAL_POSITION = 10;
    static final double VIPER_SLIDE_UP_TIME_IN_SECONDS = 1.0;


    static final double TRAVEL_FROM_WALL_TO_CENTER_IN_SECONDS = 5.0;


    static final double MAX_POWER = 1.0;  //Do NOT change this value

    DcMotor back_left_motor;
    DcMotor back_right_motor;
    DcMotor front_left_motor;
    DcMotor front_right_motor;
    DcMotor leftViper;
    DcMotor rightViper;
    DcMotor viperSlide;


    // Initialize drive, strafe, and rotate variables
    private double drive = 0;
    private double strafe = 0;
    private double rotate = 0;


    @Override
    public void runOpMode() {
        ElapsedTime runTime = new ElapsedTime();

        back_left_motor = hardwareMap.get(DcMotor.class, "backLeft");
        back_right_motor = hardwareMap.get(DcMotor.class, "backRight");
        front_left_motor = hardwareMap.get(DcMotor.class, "frontLeft");
        front_right_motor = hardwareMap.get(DcMotor.class, "frontRight");
        leftViper = hardwareMap.get(DcMotor.class, "leftViper");
        rightViper = hardwareMap.get(DcMotor.class, "rightViper");
        viperSlide = hardwareMap.get(DcMotor.class, "viperSlide");


        // Set motor run mode to RUN_USING_ENCODER
        back_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Starting position at",  "%7d :%7d :%7d :%7d",
                back_left_motor.getCurrentPosition(), back_right_motor.getCurrentPosition(),
                front_left_motor.getCurrentPosition(), front_right_motor.getCurrentPosition());


        // Set zero power behavior
        viperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set viper motor mode
        // https://javadoc.io/doc/org.firstinspires.ftc/RobotCore/latest/com/qualcomm/robotcore/hardware/DcMotor.html
        int viperInitVerticalPosition = viperSlide.getCurrentPosition();
        int leftViperInitHorizontalPosition = leftViper.getCurrentPosition();
        int rightViperInitHorizontalPosition = rightViper.getCurrentPosition();


        // Status update before the start
        telemetry.addData("Status", "Setting motor run mode to RUN_USING_ENCODER completed");
        telemetry.addData("Status", "Ready to start");
        telemetry.update();

        // Wait for the driver to press start
        waitForStart();

        // Step: robot travels from wall to the center in distance
        int newChamberTarget = back_left_motor.getCurrentPosition() + (int)(ROBOT_TRAVEL_DISTANCE_IN_INCHES * COUNTS_PER_INCH);
        back_left_motor.setTargetPosition(newChamberTarget);
        back_right_motor.setTargetPosition(newChamberTarget);
        front_left_motor.setTargetPosition(newChamberTarget);
        front_right_motor.setTargetPosition(newChamberTarget);

        back_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runTime.reset();
//        moveRobot(ROBOT_TRAVEL_POWER, 0, 0);
        back_left_motor.setPower(ROBOT_TRAVEL_POWER);
        back_right_motor.setPower(ROBOT_TRAVEL_POWER);
        front_left_motor.setPower(ROBOT_TRAVEL_POWER);
        front_right_motor.setPower(ROBOT_TRAVEL_POWER);

        telemetry.addData("Target position: ", "%7d", newChamberTarget);

        while (opModeIsActive() && (runTime.seconds() < ROBOT_TRAVEL_TIME_IN_SECONDS) &&
                (back_left_motor.isBusy() && back_right_motor.isBusy() &&
                        front_left_motor.isBusy() && front_right_motor.isBusy())) {
            telemetry.addData("Status", "Current position: %7d", back_left_motor.getCurrentPosition());
            telemetry.addData("Status", "Moving from wall to center for %f seconds", ROBOT_TRAVEL_TIME_IN_SECONDS);
            addMotorPowerData();
            telemetry.update();
        }
        resetPower();

        // Step: robot viper up
        runTime.reset();
        viperSlide.setTargetPosition(VIPER_SLIDE_TARGET_VERTICAL_POSITION);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(VIPER_POWER);
        while (runTime.seconds() < VIPER_SLIDE_UP_TIME_IN_SECONDS &&
                opModeIsActive() && viperSlide.isBusy()) {
            telemetry.addData("Status: ", "Viper Slide Up.");
            telemetry.addData("Viper Slide Power", "%2f", viperSlide.getPower());
            telemetry.addData("Viper Slide Position", "%7d", viperSlide.getCurrentPosition());
            telemetry.update();
        }

        // Step: robot viper leans towards the high chamber
        runTime.reset();
        leftViper.setTargetPosition(-VIPER_SLIDE_TARGET_HORIZONTAL_POSITION);
        rightViper.setTargetPosition(VIPER_SLIDE_TARGET_HORIZONTAL_POSITION);
        leftViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftViper.setPower(VIPER_POWER);
        rightViper.setPower(VIPER_POWER);
        while (runTime.seconds() < VIPER_SLIDE_UP_TIME_IN_SECONDS &&
                opModeIsActive() && leftViper.isBusy() && rightViper.isBusy()) {
            telemetry.addData("Status: ","Viper Slide Leaning Towards High Chamber.");
            telemetry.addData("Left Viper Power", "%2f", leftViper.getPower());
            telemetry.addData("Left Viper Position", "%7d", leftViper.getCurrentPosition());
            telemetry.addData("Right Viper Power", "%2f", rightViper.getPower());
            telemetry.addData("Right Viper Position", "%7d", rightViper.getCurrentPosition());
            telemetry.update();
        }

        sleep(2000); //Sleep 2 seconds

        // Step: robot viper resets to initial horizontal position
        runTime.reset();
        leftViper.setTargetPosition(leftViperInitHorizontalPosition);
        rightViper.setTargetPosition(rightViperInitHorizontalPosition);
        leftViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftViper.setPower(VIPER_POWER);
        rightViper.setPower(VIPER_POWER);
        while (runTime.seconds() < VIPER_SLIDE_UP_TIME_IN_SECONDS &&
                opModeIsActive() && leftViper.isBusy() && rightViper.isBusy()) {
            telemetry.addData("Status: ","Viper Slide Resetting to Initial Horizontal Position.");
            telemetry.addData("Left Viper Power", "%2f", leftViper.getPower());
            telemetry.addData("Left Viper Position", "%7d", leftViper.getCurrentPosition());
            telemetry.addData("Right Viper Power", "%2f", rightViper.getPower());
            telemetry.addData("Right Viper Position", "%7d", rightViper.getCurrentPosition());
            telemetry.update();
        }

        // Step: robot viper slides down
        runTime.reset();
        viperSlide.setTargetPosition(viperInitVerticalPosition);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(VIPER_POWER);
        while (runTime.seconds() < VIPER_SLIDE_UP_TIME_IN_SECONDS &&
                opModeIsActive() && viperSlide.isBusy()) {
            telemetry.addData("Status: ","Viper Slide Down.");
            telemetry.addData("Viper Slide Power", "%2f", viperSlide.getPower());
            telemetry.addData("Viper Slide Position", "%7d", viperSlide.getCurrentPosition());
            telemetry.update();
        }

        sleep(2000);


        resetPower();
        telemetry.addData("Status", "Completed");
        telemetry.update();

    }


    /**
     * Calculate wheel power based on drive, strafe, and rotate inputs.
     * Reference: https://github.com/FTC7393/EVLib/wiki/Mecanum-Wheels
     *
     * @param drive  Forward/backward movement (-1.0 to 1.0).
     * @param strafe Sideways movement (-1.0 to 1.0).
     * @param rotate Rotational movement (-1.0 to 1.0).
     */
    private void moveRobot(double drive, double strafe, double rotate) {
        // Calculate wheel power using mecanum formulas
        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double rearLeftPower = drive - strafe + rotate;
        double rearRightPower = drive + strafe - rotate;

        // Normalize power to ensure it is within the range [-1.0, 1.0]
        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(rearLeftPower), Math.abs(rearRightPower))
        );

        if (maxPower > MAX_POWER) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            rearLeftPower /= maxPower;
            rearRightPower /= maxPower;
        }

        back_left_motor.setPower(rearLeftPower);
        back_right_motor.setPower(rearRightPower);
        front_left_motor.setPower(frontLeftPower);
        front_right_motor.setPower(frontRightPower);
    }

    private void resetPower() {
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);

        drive = 0;
        strafe = 0;
        rotate = 0;
    }

    private void addMotorPowerData() {
        telemetry.addData("Back Left Motor Power", back_left_motor.getPower());
        telemetry.addData("Back Right Motor Power", back_right_motor.getPower());
        telemetry.addData("Front Left Motor Power", front_left_motor.getPower());
        telemetry.addData("Front Right Motor Power", front_right_motor.getPower());
    }
}
