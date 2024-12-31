package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Mecanum Auto OpMode Self Test", group = "Autonomous")
public class MechBotAutoSelfTestMode extends LinearOpMode {

    // Update power and travel time accordingly
    static final double ROBOT_TRAVEL_POWER = 0.5;
    static final double ROBOT_TRAVEL_TIME_IN_SECONDS = 2.0;


    static final double MAX_POWER = 1.0;  //Do NOT change this value

    DcMotor back_left_motor;
    DcMotor back_right_motor;
    DcMotor front_left_motor;
    DcMotor front_right_motor;


    // Initialize drive, strafe, and rotate variables
    private double drive = 0;
    private double strafe = 0;
    private double rotate = 0;


    @Override
    public void runOpMode() {
        ElapsedTime runTime = new ElapsedTime();

        back_left_motor = hardwareMap.get(DcMotor.class, "back_left_motor");
        back_right_motor = hardwareMap.get(DcMotor.class, "back_right_motor");
        front_left_motor = hardwareMap.get(DcMotor.class, "front_left_motor");
        front_right_motor = hardwareMap.get(DcMotor.class, "front_right_motor");


        // Set motor run mode to RUN_USING_ENCODER
        back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Status update before the start
        telemetry.addData("Status", "Setting motor run mode to RUN_USING_ENCODER completed");
        telemetry.addData("Status", "Ready to start");
        telemetry.update();

        // Wait for the driver to press start
        waitForStart();


        runTime.reset();
        resetPower();
        drive = ROBOT_TRAVEL_POWER;
        moveRobot(drive, strafe, rotate);
        while (runTime.seconds() < ROBOT_TRAVEL_TIME_IN_SECONDS && opModeIsActive()) {
            telemetry.addData("Status", "Moving forward for 5 seconds");
            addMotorPowerData();
            telemetry.update();
        }


        resetPower();
        runTime.reset();
        drive = -ROBOT_TRAVEL_POWER;
        moveRobot(drive, strafe, rotate);
        while (runTime.seconds() < ROBOT_TRAVEL_TIME_IN_SECONDS && opModeIsActive()) {
            telemetry.addData("Status", "Moving reverse for 5 seconds");
            addMotorPowerData();
            telemetry.update();
        }


        resetPower();
        runTime.reset();
        strafe = ROBOT_TRAVEL_POWER;
        moveRobot(drive, strafe, rotate);
        while (runTime.seconds() < ROBOT_TRAVEL_TIME_IN_SECONDS && opModeIsActive()) {
            telemetry.addData("Status", "Moving left for 5 seconds");
            addMotorPowerData();
            telemetry.update();
        }


        resetPower();
        runTime.reset();
        strafe = -ROBOT_TRAVEL_POWER;
        moveRobot(drive, strafe, rotate);
        while (runTime.seconds() < ROBOT_TRAVEL_TIME_IN_SECONDS && opModeIsActive()) {
            telemetry.addData("Status", "Moving right for 5 seconds");
            addMotorPowerData();
            telemetry.update();
        }


        resetPower();
        runTime.reset();
        rotate = ROBOT_TRAVEL_POWER;
        moveRobot(drive, strafe, rotate);
        while (runTime.seconds() < ROBOT_TRAVEL_TIME_IN_SECONDS && opModeIsActive()) {
            telemetry.addData("Status", "Rotating left for 5 seconds");
            addMotorPowerData();
            telemetry.update();
        }

        resetPower();
        runTime.reset();
        rotate = -ROBOT_TRAVEL_POWER;
        moveRobot(drive, strafe, rotate);
        while (runTime.seconds() < ROBOT_TRAVEL_TIME_IN_SECONDS && opModeIsActive()) {
            telemetry.addData("Status", "Rotating right for 5 seconds");
            addMotorPowerData();
            telemetry.update();
        }


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
