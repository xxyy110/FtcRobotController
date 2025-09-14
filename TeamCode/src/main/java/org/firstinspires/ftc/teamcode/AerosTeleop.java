
package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class AerosTeleop extends OpMode {
    MecanumDriveTrain driveTrain;
    AprilTagVision vision;

    boolean streaming = false;
    @Override
    public void init() {
        driveTrain = new MecanumDriveTrain(hardwareMap, null);
        vision = new AprilTagVision(hardwareMap, gamepad1);
        vision.visionPortalStreaming(false);
        telemetry.addData("Status", "Teleop Initialized");
        telemetry.addData("Vision Portal Status:", "Streaming Off");
        telemetry.update();
    }
    @Override
    public void loop() {
        telemetry.addData("Status", "Teleop Running");
        if (streaming) {
            telemetry.addData("Vision Portal Status:", "Streaming On");
        } else {
            telemetry.addData("Vision Portal Status:", "Streaming Off");
        }
        telemetry.update();
        driveTrain.drive(
                -gamepad1.left_stick_x, // moves left/right
                gamepad1.right_trigger - gamepad1.left_trigger, // moves forward and back
                gamepad1.right_stick_x  // rotation
        );

        if (gamepad1.dpad_up) {
            vision.visionPortalStreaming(true);
            streaming = true;
        }
        if (gamepad1.dpad_down) {
            vision.visionPortalStreaming(false);
            streaming = false;
        }



    }
    @Override
    public void stop() {
        driveTrain.stop();
        vision.closeVisionPortal();
        telemetry.addData("Status", "Teleop Stopped");
        telemetry.update();
    }
}