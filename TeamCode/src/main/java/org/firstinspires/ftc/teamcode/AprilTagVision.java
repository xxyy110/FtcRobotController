
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import java.util.ArrayList;


public class AprilTagVision {
    //DO NO CHANGE THESE VALUES I SPENT LIKE 2 HOURS TRYING TO CALIBRATE THE CAMERA

    private final double fx = 723.8194965447678;    // Focal length in pixels along the x-axis

    private final double fy = 723.4364215770782;    // Focal length in pixels along the y-axis

    private final double cx = 336.36531074370555;   // Principal point x-coordinate in pixels

    private final double cy = 259.79772856338064;   // Principal point y-coordinate in pixels
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    public ArrayList<Integer> detectedIDs;
    public AprilTagDetection ID21, ID22, ID23, ID20, ID24;
    public AprilTagVision(HardwareMap hardwareMap, Gamepad gamepad1) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(fx, fy, cx, cy
                        /*
                        723.8194965447678,
                        723.4364215770782,
                        336.36531074370555,
                        259.79772856338064
                        */

                )
                .build();
        detectedIDs = new ArrayList<Integer>();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();


    }

    public void detectAprilTags() {
        detectedIDs.clear();
        ID21 = ID22 = ID23 = ID20 = ID24 = null;

        if (aprilTagProcessor.getDetections().size() > 0) {
            for (int i = 0; i < aprilTagProcessor.getDetections().size(); i++) {
                AprilTagDetection tag = aprilTagProcessor.getDetections().get(i);
                int id = tag.id;

                if (id == 21) {
                    ID21 = tag;
                    detectedIDs.add(21);

                } else if (id == 22) {
                    ID22 = tag;
                    detectedIDs.add(22);
                } else if (id == 23) {
                    ID23 = tag;
                    detectedIDs.add(23);
                } else if (id == 20) {
                    ID20 = tag;
                    detectedIDs.add(20);
                } else if (id == 24) {
                    ID24 = tag;
                    detectedIDs.add(24);
                }
            }
        }
    }

    public AprilTagDetection getDetection(int id) {
        switch (id) {
            case 21: return ID21;
            case 22: return ID22;
            case 23: return ID23;
            case 20: return ID20;
            case 24: return ID24;
            default: return null;
        }
    }

    public void closeVisionPortal() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    public void visionPortalStreaming(boolean stream) {
        if (visionPortal != null) {
            if (stream) {
                visionPortal.resumeStreaming();
            } else {
                visionPortal.stopStreaming();
            }
        }
    }
}