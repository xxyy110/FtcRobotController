
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//THIS FILE IS FOR THE OFFENSE AUTO - LIKE THROWING THE ARTIFACTS
@Autonomous
public class AerosOffenseAuto extends LinearOpMode {
    private MecanumDriveTrain drive;
    private AprilTagVision vision;

    @Override
    public void runOpMode() {
        // Initialize hardware before waitForStart()
        drive = new MecanumDriveTrain(hardwareMap, this);
        vision = new AprilTagVision(hardwareMap, gamepad1);

        telemetry.addData("Status", "Waiting for start");
        telemetry.update();

        waitForStart();
        drive.resetYaw();
        if (isStopRequested()) {
            vision.closeVisionPortal();
            drive.stop();
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {
            vision.detectAprilTags();

            telemetry.addData("Detections", vision.detectedIDs);
            telemetry.update();
            if (!drive.frontLeft.isBusy() && !drive.frontRight.isBusy() && !drive.backLeft.isBusy() && !drive.backRight.isBusy()) {
                drive.stop();
            }
        }
    }
}
