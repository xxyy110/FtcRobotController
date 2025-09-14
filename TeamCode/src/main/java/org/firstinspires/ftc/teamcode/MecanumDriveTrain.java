
package org.firstinspires.ftc.teamcode;



import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;



public class MecanumDriveTrain {

    enum opmode {
        AUTO,
        TELEOP
    }
    private final LinearOpMode opMode;

    private final IMU imu;


    private final double WHEEL_DIAMETER_INCHES = 100.0 / 25.4; // 100mm to inches (MECANUM WHEELS 100mm DIAMETER)
    private final double WHEEL_CIRCUMFERENCE_INCHES = Math.PI * WHEEL_DIAMETER_INCHES; //CHANGE THESE VALUES IF NEEDED
    private final int TICKS_PER_REVOLUTION = 1120; // NeveRest 40
    private final double TICKS_PER_INCH = TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE_INCHES;
    public final DcMotor frontLeft, frontRight, backLeft, backRight;
    public MecanumDriveTrain(HardwareMap hardwareMap, LinearOpMode opmode) {
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        this.opMode = opmode;
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        resetYaw();

        resetEncoders();


        if (opmode == null) {
            runWithoutEncoders();
        } else {
            runUsingEncoders();
        }

    }

    public void drive(double x, double y, double rx) {

        double frontLeftPower = y + x + rx;
        double frontRightPower = y - x - rx;
        double backLeftPower = y - x + rx;
        double backRightPower = y + x - rx;


        //Math.max can only take in two arguments which is why we write it like a chain

        double maxPower = Math.max(
                Math.abs(frontLeftPower),
                Math.max(
                        Math.abs(frontRightPower),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        // Normalize the powers so that no motor exceeds 1.0
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runUsingEncoders() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runWithoutEncoders() {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTargetPosition(int targetPosition) {
        frontLeft.setTargetPosition(targetPosition);
        frontRight.setTargetPosition(targetPosition);
        backLeft.setTargetPosition(targetPosition);
        backRight.setTargetPosition(targetPosition);
    }

    public void setPower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public void setWheelsPower(double flPower, double frPower, double blPower, double brPower) {
        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }

    public void stop() {
        setPower(0);
    }

    public void drive(double distance, double power) { // DISTANCE IN INCHES, distance
        int targetPosition = (int) (distance * TICKS_PER_INCH);

        resetEncoders();
        setTargetPosition(targetPosition);

        runToPosition();
        setPower(power);

    }

    public void resetYaw() {
        imu.resetYaw();

    }

    public double getYaw() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void turn(double angle, double power, opmode mode) { // for the opmode parameter, use 0 for autonomous and 1 for teleop
        resetYaw();
        if (mode == opmode.AUTO) {
            if (angle > 0) {
                while (getYaw() < angle && opMode.opModeIsActive()) {
                    setWheelsPower(power, -power, power, -power);
                }
                stop();
            } else {
                while (getYaw() > angle && opMode.opModeIsActive()) {
                    setWheelsPower(-power, power, -power, power);
                }
                stop();
            }
        } else if (mode == opmode.TELEOP) {
            if (angle > 0) {
                while (getYaw() < angle) {
                    setWheelsPower(power, -power, power, -power);
                }
                stop();
            } else {
                while (getYaw() > angle) {
                    setWheelsPower(-power, power, -power, power);
                }
                stop();
            }
        }
    }

    public void strafe(double distance, double power) {
        int targetPosition = (int) (distance * TICKS_PER_INCH);

        resetEncoders();
        frontLeft.setTargetPosition(targetPosition);
        frontRight.setTargetPosition(-targetPosition);
        backLeft.setTargetPosition(-targetPosition);
        backRight.setTargetPosition(targetPosition);

        runToPosition();
        setWheelsPower(power, -power, -power, power);
    }


}