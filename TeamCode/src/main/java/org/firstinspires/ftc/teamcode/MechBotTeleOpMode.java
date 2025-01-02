package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Mechbot Teleop", group = "Teleop")
public class MechBotTeleOpMode extends LinearOpMode {

    static final double VIPER_SLIDE_POWER_UP = 0.2;
    static final double VIPER_SLIDE_POWER_DOWN = 0.2;


    private DcMotor viperSlide;
    private DcMotor leftViper;
    private DcMotor rightViper;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private CRServo clawArmServo;
    private CRServo rotationClawServo;

    @Override
    public void runOpMode() {
        // Initialize hardware
        viperSlide = hardwareMap.get(DcMotor.class, "viperSlide");
        leftViper = hardwareMap.get(DcMotor.class, "leftViper");
        rightViper = hardwareMap.get(DcMotor.class, "rightViper");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        clawArmServo = hardwareMap.get(CRServo.class, "clawarmservo");
        rotationClawServo = hardwareMap.get(CRServo.class, "rotationclawservo");

        // Set zero power behavior
        viperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Ready to start");
        telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();

        telemetry.addData("Status", "Started");
        telemetry.update();

        // Main loop
        while (opModeIsActive()) {
            // Drive control
            float movement = gamepad1.right_trigger - gamepad1.left_trigger;
            float strafe = gamepad1.left_stick_x;
            float rotate = gamepad1.right_stick_x * 2; // Adjusted rotation sensitivity

            // Calculate motor powers and clamp them to [-1, 1]
            backLeft.setPower(clampPower(strafe + rotate + movement));
            backRight.setPower(clampPower(movement - strafe - rotate));
            frontLeft.setPower(clampPower(movement + strafe - rotate));
            frontRight.setPower(clampPower(movement - strafe + rotate));

            // Viper slide control
            if (gamepad2.dpad_down) {
                viperSlide.setPower(-1);
            } else if (gamepad2.dpad_up) {
                viperSlide.setPower(1);
            } else {
                viperSlide.setPower(0);
            }

            // Viper arm control
            if (gamepad2.dpad_left) {
                leftViper.setPower(-VIPER_SLIDE_POWER_UP);
                rightViper.setPower(VIPER_SLIDE_POWER_UP);
            } else if (gamepad2.dpad_right) {
                leftViper.setPower(VIPER_SLIDE_POWER_DOWN);
                rightViper.setPower(-VIPER_SLIDE_POWER_DOWN);
            } else {
                leftViper.setPower(0);
                rightViper.setPower(0);
            }

            // Claw arm control
            if (gamepad2.right_bumper) {
                clawArmServo.setDirection(CRServo.Direction.REVERSE);
                clawArmServo.setPower(0.2);
            } else if (gamepad2.left_bumper) {
                clawArmServo.setDirection(CRServo.Direction.FORWARD);
                clawArmServo.setPower(0.2);
            } else {
                clawArmServo.setPower(0);
            }



            // Rotation claw control
            if (gamepad2.y) {
                rotationClawServo.setDirection(CRServo.Direction.REVERSE);
                rotationClawServo.setPower(0.1);
            } else if (gamepad2.b) {
                rotationClawServo.setDirection(CRServo.Direction.FORWARD);
                rotationClawServo.setPower(0.1);
            } else {
                rotationClawServo.setPower(0);
            }

            telemetry.addData("backLeft", "%.1f", backLeft.getPower());
            telemetry.addData("backRight", "%.1f", backRight.getPower());
            telemetry.addData("frontLeft", "%.1f", frontLeft.getPower());
            telemetry.addData("frontRight", "%.1f", frontRight.getPower());
            telemetry.addData("clawArmServo", "%.1f", clawArmServo.getPower());
            telemetry.addData("rotationClawServo", "%.1f", rotationClawServo.getPower());
            telemetry.update();
        }
    }

    // Helper method to clamp motor power to the range [-1, 1]
    private double clampPower(double power) {
        return Math.max(-1, Math.min(1, power));
    }
}
