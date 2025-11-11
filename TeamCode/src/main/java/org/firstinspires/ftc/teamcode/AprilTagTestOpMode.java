package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
@Disabled
@TeleOp(name = "AprilTagTesting", group = "Robot")
public class AprilTagTestOpMode extends OpMode {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void init() {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                // Add any desired customization, like setting the tag library or drawing options
                // .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .setDrawAxes(true)
                .build();

        // Create the VisionPortal, passing in the camera and the processor.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "shooter cam")) // Replace "Webcam 1" with your camera name
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .build();

    }

    @Override
    public void loop() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                // Access information about the detected tag
                int tagId = detection.id;
                String tagName = detection.metadata.name;
                double xPosition = detection.ftcPose.x;
                double yPosition = detection.ftcPose.y;
                double zPosition = detection.ftcPose.z;// X position relative to the camera
                // ... and other pose data (y, z, yaw, pitch, roll)

                // Implement logic based on the detected tag, e.g.,
                 if (tagId == 24) {
                     // Perform actions based on this specific tag
                     double offset_x = 0 - xPosition;
                     double threshhold = 2;
                     if(Math.abs(offset_x) < threshhold) {
                         telemetry.addLine("Centered");
                     }
                     else if(offset_x < 0) {
                         telemetry.addLine("Move Right");
                     }
                     else if(offset_x > 0) {
                         telemetry.addLine("Move Left");
                     }
                 }

                telemetry.addData("Tag ID", tagId);
                telemetry.addData("Tag Name", tagName);
                telemetry.addData("X", xPosition);
                telemetry.addData("Y", yPosition);
                telemetry.addData("Z", zPosition);
                // Add more telemetry for other relevant data
            }
            else {
                telemetry.addLine("No Tag");
            }
        }

        telemetry.update();

    }

}
