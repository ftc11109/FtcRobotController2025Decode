/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Collections;
import java.util.List;

@Autonomous(name="Decode Autonomus", group="Robot")
public class DecodeAutonomus extends LinearOpMode {

    /* Declare OpMode members. */
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    IMU imu;

    AprilTagProcessor frontTags;
    AprilTagProcessor shooterTags;
    VisionPortal frontPortal;
    VisionPortal shooterPortal;

    int ALLIANCE_TAG;


    private ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 12.0 ;     // 12:1
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.14159);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double     TURN_ANGLE_TOLERANCE    = 0.25;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        //AprilTag setup
        //Multi-camera stuff
        int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL);
        int portalShooterViewId = viewIds[0];
        int portalFrontViewId = viewIds[1];
        //Individual cameras
        shooterTags = new AprilTagProcessor.Builder().build();
        shooterPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "shooter cam"))
                .setLiveViewContainerId(portalShooterViewId)
                .addProcessor(shooterTags)
                .build();
        frontTags = new AprilTagProcessor.Builder().build();
        frontPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "front cam"))
                .setLiveViewContainerId(portalFrontViewId)
                .addProcessor(frontTags)
                .build();

        String alliance = "";
        String start_pos = "";
        final int bestRoute = 2;
        final int maxRoute = 4;
        int selectedRoute = bestRoute;
        //Robot data class (for IMU direction)
        while (!isStarted() && !isStopRequested()) {
            //Alliance
            telemetry.addLine("Alliance selection:");
            telemetry.addLine("Press B for red alliance");
            telemetry.addLine("Press X for blue alliance");
            if (gamepad1.b) {
                alliance = "Red";
            }
            if (gamepad1.x) {
                alliance = "Blue";
            }
            if(alliance != "") {
                telemetry.addLine(alliance + " alliance selected");
            }
            //Positioning
            telemetry.addLine("Starting Position Selection:");
            telemetry.addLine("Press Y for wall start");
            telemetry.addLine("Press A for line start");
            if (gamepad1.y) {
                start_pos = "Wall";
            }
            if (gamepad1.a) {
                start_pos = "Line";
            }
            if(start_pos != "") {
                telemetry.addLine(start_pos + " starting position selected");
            }

            //Route selection
            if(gamepad1.dpadUpWasPressed()) {
                selectedRoute++;
            }
            if(gamepad1.dpadDownWasPressed()) {
                selectedRoute--;
            }
            //Looping
            if(selectedRoute > maxRoute) {
                selectedRoute = 0;
            }
            if(selectedRoute < 0) {
                selectedRoute = maxRoute;
            }
            telemetry.addLine("Use D-pad up and down to select route");
            telemetry.addLine("Selected: " +  selectedRoute);
            telemetry.update();

        }
        ALLIANCE_TAG = alliance == "Red" ? 24 : 20;
        // Send telemetry message to indicate successful Encoder reset
//        telemetry.addData("Starting at",  "%7d :%7d :%7d :%7d",
//                frontLeftDrive.getCurrentPosition(),
//                frontRightDrive.getCurrentPosition(),
//                backLeftDrive.getCurrentPosition(),
//                backRightDrive.getCurrentPosition());
//        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        if(alliance == "Red") {
            //Red alliance
            RobotData.ALLIANCE = "Red";
            if(start_pos == "wall") {
                switch (selectedRoute) {
                    case 0:
                        //Drive away from goal (get off line
                        encoderDrive(DRIVE_SPEED, 6, 6, 6, 6, 2);
                        break;
                    case 1:
                        //Get into the rough aiming position
                        encoderDrive(DRIVE_SPEED, 6, 6, 6, 6, 2);
                        turnToHeading(90);
                        List<AprilTagDetection> tags = getAprilTags(shooterTags);
                        if(tags.size() > 0) {
                            //Align to goal
                            alignToTag(tags.get(0));
                        }
                        //Shooter code here
                }
            }
            else {

            }
        }
        else {
            //Blue alliance
            RobotData.ALLIANCE = "Blue";
            if(start_pos == "wall") {
                switch (selectedRoute) {
                    case 0:
                        encoderDrive(DRIVE_SPEED, 6, 6, 6, 6, 2);
                        break;
                    case 1:
                        encoderDrive(DRIVE_SPEED, 6, 6, 6, 6, 2);
                        List<AprilTagDetection> tags = getAprilTags(shooterTags);
                        if(tags.size() > 0) {
                            //Align to goal
                            alignToTag(tags.get(0));
                        }
                        //Shooter code here
                }
            }
        }

        shooterPortal.close();
        frontPortal.close();

        telemetry.addData("Path", "Complete");
        telemetry.addLine("Current Robot Heading:" + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    //Returns a list of the found AprilTag IDs
    public List<AprilTagDetection> getAprilTags(AprilTagProcessor camera) {
        List<AprilTagDetection> detections = camera.getDetections();
        List<AprilTagDetection> tags = Collections.emptyList();
        for(int i = 0; i < detections.size(); i++) {
            AprilTagDetection detection = detections.get(i);
            tags.set(i, detection);
        }

        return tags;
    }

    public boolean alignToTag(AprilTagDetection tag) {
        //Make sure we are looking at the alliance goal tag
        if(tag.id == ALLIANCE_TAG) {
            //Check X value
            double x = tag.rawPose.x;
            encoderDrive(DRIVE_SPEED, -x, x, -x, x, 50);
            return true;
        }
        else {
            return false;
        }
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
                             double frontLeftInches, double frontRightInches, double backLeftInches, double backRightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;
        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeftDrive.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = backRightDrive.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);
            frontLeftDrive.setTargetPosition(newFrontLeftTarget);
            frontRightDrive.setTargetPosition(newFrontRightTarget);
            backLeftDrive.setTargetPosition(newBackLeftTarget);
            backRightDrive.setTargetPosition(newBackRightTarget);
            // Turn On RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            frontLeftDrive.setPower(Math.abs(speed));
            frontRightDrive.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (frontLeftDrive.isBusy() && frontRightDrive.isBusy() && backLeftDrive.isBusy() && backRightDrive.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Front motors running to",  " %7d :%7d", newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Front motors currently at",  " at %7d :%7d", frontLeftDrive.getCurrentPosition(), frontRightDrive.getCurrentPosition());
                telemetry.addData("Back motors running to",  " %7d :%7d", newBackLeftTarget,  newBackRightTarget);
                telemetry.addData("Front motors currently at",  " at %7d :%7d", backLeftDrive.getCurrentPosition(), backRightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
            // Turn off RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RobotData.END_AUTO_HEADING = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            sleep(250);   // optional pause after each move.
        }
    }
    public void turnToHeading(int target_heading) {
        double target_heading_radians = (float) (target_heading * 0.0174533);
        double current_heading = imu.getRobotYawPitchRollAngles().getYaw();

        double angle_error = target_heading_radians - current_heading;

        double adjustment_turn_power = angle_error * TURN_SPEED;
        adjustment_turn_power = Range.clip(adjustment_turn_power, -1.0, 1.0);

        frontLeftDrive.setPower(adjustment_turn_power);
        backLeftDrive.setPower(adjustment_turn_power);
        frontRightDrive.setPower(-adjustment_turn_power);
        backRightDrive.setPower(-adjustment_turn_power);

        if (Math.abs(angle_error) < TURN_ANGLE_TOLERANCE) {
            // Stop motors or transition to next action
            frontLeftDrive.setPower(0);
            backLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backRightDrive.setPower(0);
            // ... and so on for other motors
        }
    }

}
