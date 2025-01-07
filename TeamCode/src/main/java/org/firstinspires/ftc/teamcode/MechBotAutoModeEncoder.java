package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Mecanum Auto OpMode", group = "Autonomous")
public class MechBotAutoModeEncoder extends LinearOpMode {

    // Robot travels distance in inches
    static final double ROBOT_TRAVEL_DISTANCE_IN_INCHES = 24.0;
    // Viper extension length in inches
    static final double VIPER_EXTENSION_LENGTH_IN_INCHES = 12.0;
    // Chamber height in inches
    static final double CHAMBER_HEIGHT_IN_INCHES = 10.0;



    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: NeveRest40Gearmotor Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    // Update power and travel time accordingly
    static final double ROBOT_TRAVEL_POWER = 0.5;  // in range between -1 to 1

    static final double ROBOT_TRAVEL_TIME_IN_SECONDS = 5.0;


    static final double VIPER_POWER = 0.1; // in range between -1 to 1
    static final double VIPER_SLIDE_UP_TIME_IN_SECONDS = 1.0;


    static final double MAX_POWER = 1.0;  //Do NOT change this value

    DcMotor back_left_motor;
    DcMotor back_right_motor;
    DcMotor front_left_motor;
    DcMotor front_right_motor;
    DcMotor leftViper;
    DcMotor rightViper;
    DcMotor viperSlide;


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

        // Set viper zero power behavior
        setVipersZeroPowerBehavior();

        // Set Wheels zero power behavior
        setWheelsZeroPowerBehavior();

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



        // Set viper motor mode
        // https://javadoc.io/doc/org.firstinspires.ftc/RobotCore/latest/com/qualcomm/robotcore/hardware/DcMotor.html
        viperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftViper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightViper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftViper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightViper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        telemetry.addData("Target position: ", "%7d", newChamberTarget);

        back_left_motor.setTargetPosition(newChamberTarget);
        back_right_motor.setTargetPosition(newChamberTarget);
        front_left_motor.setTargetPosition(newChamberTarget);
        front_right_motor.setTargetPosition(newChamberTarget);

        back_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runTime.reset();
        moveRobot(ROBOT_TRAVEL_POWER, 0, 0);


        while (opModeIsActive() && (runTime.seconds() < ROBOT_TRAVEL_TIME_IN_SECONDS) &&
                (back_left_motor.isBusy() && back_right_motor.isBusy() &&
                        front_left_motor.isBusy() && front_right_motor.isBusy())) {
            telemetry.addData("Status", "Current position: %7d", back_left_motor.getCurrentPosition());
            telemetry.addData("Status", "Moving from wall to center for %f seconds", ROBOT_TRAVEL_TIME_IN_SECONDS);
            addMotorPowerData();
            telemetry.update();
        }
        resetWheelsPower();

        // Step: robot viper up
        runTime.reset();
        int viperSlideExtensionTarget = viperSlide.getCurrentPosition() + (int)(VIPER_EXTENSION_LENGTH_IN_INCHES * COUNTS_PER_INCH);
        viperSlide.setTargetPosition(viperSlideExtensionTarget);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(VIPER_POWER);
        while (runTime.seconds() < VIPER_SLIDE_UP_TIME_IN_SECONDS &&
                opModeIsActive() && viperSlide.isBusy()) {
            telemetry.addData("Status: ", "Viper Slide Up.");
            telemetry.addData("Viper Slide Power", "%2f", viperSlide.getPower());
            telemetry.addData("Viper Slide Position", "%7d", viperSlide.getCurrentPosition());
            telemetry.update();
        }
        viperSlide.setPower(0);  //viper stops and brakes

        // Step: robot viper leans towards the high chamber

        // Calculate side b using the Pythagorean Theorem: b = sqrt(c^2 - a^2)
        double parkingDistanceToChamberInInches = Math.sqrt(Math.pow(VIPER_EXTENSION_LENGTH_IN_INCHES, 2) - Math.pow(CHAMBER_HEIGHT_IN_INCHES, 2));
        int leftViperSlideToTarget = leftViper.getCurrentPosition() + (int)(parkingDistanceToChamberInInches * COUNTS_PER_INCH);
        int rightViperSlideToTarget = rightViper.getCurrentPosition() - (int)(parkingDistanceToChamberInInches * COUNTS_PER_INCH);

        telemetry.addData("Left Viper Target Position", "%7d", leftViperSlideToTarget);
        telemetry.addData("Right Viper Target Position", "%7d", rightViperSlideToTarget);

        runTime.reset();
        leftViper.setTargetPosition(leftViperSlideToTarget);
        rightViper.setTargetPosition(rightViperSlideToTarget);
        leftViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftViper.setPower(-VIPER_POWER);
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
        // Viper stops and brakes
        leftViper.setPower(0);
        rightViper.setPower(0);

        sleep(2000); //Sleep 2 seconds

        // Step: robot viper resets to initial horizontal position
        runTime.reset();
        leftViper.setTargetPosition(leftViperInitHorizontalPosition);
        rightViper.setTargetPosition(rightViperInitHorizontalPosition);

        telemetry.addData("Left Viper Target Position", "%7d", leftViperInitHorizontalPosition);
        telemetry.addData("Right Viper Target Position", "%7d", rightViperInitHorizontalPosition);

        leftViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftViper.setPower(VIPER_POWER);
        rightViper.setPower(-VIPER_POWER);

        while (runTime.seconds() < VIPER_SLIDE_UP_TIME_IN_SECONDS &&
                opModeIsActive() && leftViper.isBusy() && rightViper.isBusy()) {
            telemetry.addData("Status: ","Viper Slide Resetting to Initial Horizontal Position.");
            telemetry.addData("Left Viper Power", "%2f", leftViper.getPower());
            telemetry.addData("Left Viper Position", "%7d", leftViper.getCurrentPosition());
            telemetry.addData("Right Viper Power", "%2f", rightViper.getPower());
            telemetry.addData("Right Viper Position", "%7d", rightViper.getCurrentPosition());
            telemetry.update();
        }
        leftViper.setPower(0);
        rightViper.setPower(0);

        // Step: robot viper slides down
        runTime.reset();
        viperSlide.setTargetPosition(viperInitVerticalPosition);
        telemetry.addData("Viper Slide Target Position", "%7d", viperInitVerticalPosition);


        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(-VIPER_POWER);
        while (runTime.seconds() < VIPER_SLIDE_UP_TIME_IN_SECONDS &&
                opModeIsActive() && viperSlide.isBusy()) {
            telemetry.addData("Status: ","Viper Slide Down.");
            telemetry.addData("Viper Slide Power", "%2f", viperSlide.getPower());
            telemetry.addData("Viper Slide Position", "%7d", viperSlide.getCurrentPosition());
            telemetry.update();
        }
        viperSlide.setPower(0);

        sleep(2000);


        resetWheelsPower();
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

    private void resetWheelsPower() {
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
    }

    private void setVipersZeroPowerBehavior() {
        viperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void setWheelsZeroPowerBehavior() {
        back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void addMotorPowerData() {
        telemetry.addData("Back Left Motor Power", back_left_motor.getPower());
        telemetry.addData("Back Right Motor Power", back_right_motor.getPower());
        telemetry.addData("Front Left Motor Power", front_left_motor.getPower());
        telemetry.addData("Front Right Motor Power", front_right_motor.getPower());
    }
}
