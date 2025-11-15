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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.sql.Time;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name="Decode Autonomus", group="Robot")
public class DecodeAutonomus extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    Gate gate;
    Kicker kicker;
    Shooter shooter;
    IMU imu;

    AprilTagProcessor frontTags;
    AprilTagProcessor shooterTags;
    VisionPortal frontPortal;
    VisionPortal shooterPortal;
    private enum DIRECTION {
        FORWARD(1),
        BACKWARD(-1);
        private final int num;
        DIRECTION(int num) {
            this.num = num;
        };
        int getValue() {
            return this.num;
        };
    }

    int ALLIANCE_TAG;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;
    static final double     DRIVE_GEAR_REDUCTION    = 12.0 ;     // 12:1
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.14159);
    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.2;
    static final double     TURN_ANGLE_TOLERANCE    = 3;
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

        gate = new Gate(hardwareMap);
        kicker = new Kicker(hardwareMap, null, runtime, true, gate);
        shooter = new Shooter(hardwareMap, null, null, runtime);

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
        List<String> route_desc = new ArrayList<>();
        //Route 0
        route_desc.add("Drive off line");
        //Route 1
        route_desc.add("Drive off line, shoot three balls");
        final int bestRoute = 1;
        final int maxRoute = 1;
        int selectedRoute = bestRoute;

        //Alliance and route selection
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
            telemetry.addLine("Press A for goal wall start");
            if (gamepad1.y) {
                start_pos = "Far wall";
            }
            if (gamepad1.a) {
                start_pos = "Goal wall";
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
            telemetry.addLine(route_desc.get(selectedRoute) + " selected");
            telemetry.update();

        }
        ALLIANCE_TAG = alliance == "Red" ? 24 : 20;

        // Wait for the game to start (driver presses START)
        waitForStart();
        // Step through each leg of the path,
        switch (selectedRoute) {
            case 0:
                if(start_pos == "Goal") {
                    driveStraight(DRIVE_SPEED, 6, "Driving off of goal");
                }
                else if(start_pos == "Wall") {
                    driveStraight(DRIVE_SPEED, 6, "Driving off of wall");
                }
                break;
            case 1:
                //Move away from goal
                driveStraight(DRIVE_SPEED, -22, "Driving away from goal");
                //Turn shooter towards goal
                if(alliance == "Red") {
                    frontLeftDrive.setPower(DRIVE_SPEED);
                    backLeftDrive.setPower(-DRIVE_SPEED);
                    frontRightDrive.setPower(-DRIVE_SPEED);
                    backRightDrive.setPower(DRIVE_SPEED);
                }
                else if(alliance == "Blue") {
                    frontLeftDrive.setPower(-DRIVE_SPEED);
                    backLeftDrive.setPower(DRIVE_SPEED);
                    frontRightDrive.setPower(DRIVE_SPEED);
                    backRightDrive.setPower(-DRIVE_SPEED);
                }
                long driveStartTime = runtime.now(TimeUnit.MILLISECONDS);
                while(runtime.now(TimeUnit.MILLISECONDS) < driveStartTime + 500) {
                    telemetry.addLine("Strafing from wall");
                    telemetry.update();
                }
                frontLeftDrive.setPower(0);
                backLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                backRightDrive.setPower(0);

                if(alliance == "Red") {
                    turnToHeading(-73, "Turning towards goal");
                }
                else {
                    turnToHeading(-107, "Turning towards goal");
                }
                long waitTime = runtime.now(TimeUnit.MILLISECONDS);
                while(runtime.now(TimeUnit.MILLISECONDS) < waitTime + 3000) {
                    telemetry.addLine("Briefly pausing");
                    telemetry.update();
                    if(getAprilTags(shooterTags).size() > 0) {
                        break;
                    }
                }
                boolean triedToAlign = false;
                telemetry.addLine("Aligning to tag");
                telemetry.update();
                if(getAprilTags(shooterTags).size() > 0) {
                    alignToTag(getAprilTags(shooterTags).get(0));
                    triedToAlign = true;
                    //telemetry.addLine("Tag: " + getAprilTags(shooterTags).get(0).id);
                }

                //Spin up shooter and wait for it to spin up
                shooter.startMed();
                while(shooter.shooterMotor.getVelocity() - 10 < shooter.targetTps) {
                    telemetry.addLine("Spinning up shooter");
                    telemetry.addLine("Shooter Speed: " + shooter.shooterMotor.getVelocity());
                    telemetry.addLine("Aligned? " + triedToAlign);
                    telemetry.update();
                }
                //Kick!
                for(int i = 0; i < 3; i++) {
                    kicker.kick();
                    while(kicker.state != Kicker.STATE.IDLE) {
                        kicker.tick();
                    }
                }
        }

        shooterPortal.close();
        frontPortal.close();

        telemetry.addData("Path", "Complete");
        telemetry.addLine("Current Robot Heading:" + imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.update();
        sleep(3000);  // pause to display final telemetry message.
    }
    //Returns a list of the found AprilTags as a list of AprilTagDetection objects
    public List<AprilTagDetection> getAprilTags(AprilTagProcessor camera) {
        List<AprilTagDetection> detections = camera.getDetections();
        List<AprilTagDetection> tags = new ArrayList<>();
        for(int i = 0; i < detections.size(); i++) {
            AprilTagDetection detection = detections.get(i);
            tags.add(detection);
        }

        return tags;
    }
    public void turnRight(double power) {
        frontLeftDrive.setPower(power);
        backLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
        backRightDrive.setPower(-power);
    }
    public void turnLeft(double power) {
        frontLeftDrive.setPower(-power);
        backLeftDrive.setPower(-power);
        frontRightDrive.setPower(power);
        backRightDrive.setPower(power);
    }
    public void stopMotors() {
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
    }
    /*
    * Method to align the robot to a tag
    * Returns false if the tag couldn't be found
    * Uses the bearing of a tag and TurnToHeading
    */
    public boolean alignToTag(AprilTagDetection tag) {
        //Make sure we are looking at the alliance goal tag
        if(tag.id == ALLIANCE_TAG) {
            //Check X value
            double tagBearing = tag.ftcPose.bearing;
            double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            turnToHeading((int) (robotHeading + tagBearing), "Turning to tag");
            return true;
        }
        else {
            return false;
        }
    }


    public void driveStraight(double speed, double inches, String message) {
        encoderDrive(speed, inches, inches, inches, inches, 30, message);
    }
    public void driveStraight(double speed, double inches, double timeoutS, String message) {
        encoderDrive(speed, inches, inches, inches, inches, timeoutS, message);
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
                             double timeoutS,
                             String message) {
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

            frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // reset the timeout time and start motion.
            runtime.reset();
            frontLeftDrive.setPower(Math.abs(speed));
            frontRightDrive.setPower(Math.abs(speed));
            backRightDrive.setPower(Math.abs(speed));
            backLeftDrive.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (frontLeftDrive.isBusy() || frontRightDrive.isBusy() || backLeftDrive.isBusy() || backRightDrive.isBusy())) {
                // Display it for the driver.
                telemetry.addLine(message);
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
    public void turnToHeading(int target_heading, String message) {
        double current_heading = imu.getRobotYawPitchRollAngles().getYaw();

        double angle_error = target_heading - current_heading;

        double adjustment_turn_power = angle_error * TURN_SPEED;
        adjustment_turn_power = Range.clip(adjustment_turn_power, -TURN_SPEED, TURN_SPEED);

        frontLeftDrive.setPower(-adjustment_turn_power);
        backLeftDrive.setPower(-adjustment_turn_power);
        frontRightDrive.setPower(adjustment_turn_power);
        backRightDrive.setPower(adjustment_turn_power);
        while(Math.abs(angle_error) > TURN_ANGLE_TOLERANCE) {
            telemetry.addLine(message);
            current_heading = imu.getRobotYawPitchRollAngles().getYaw();
            angle_error = target_heading - current_heading;
            telemetry.addLine("IMU heading: " + imu.getRobotYawPitchRollAngles().getYaw() + ", Target heading: " + target_heading);
            telemetry.update();
        }
        adjustment_turn_power = angle_error * TURN_SPEED;
        adjustment_turn_power = Range.clip(adjustment_turn_power, -TURN_SPEED, TURN_SPEED);

        frontLeftDrive.setPower(-adjustment_turn_power);
        backLeftDrive.setPower(-adjustment_turn_power);
        frontRightDrive.setPower(adjustment_turn_power);
        backRightDrive.setPower(adjustment_turn_power);
        while(Math.abs(angle_error) > TURN_ANGLE_TOLERANCE) {
            telemetry.addLine(message);
            current_heading = imu.getRobotYawPitchRollAngles().getYaw();
            angle_error = target_heading - current_heading;
            telemetry.addLine("IMU heading: " + imu.getRobotYawPitchRollAngles().getYaw() + ", Target heading: " + target_heading);
            telemetry.update();
        }
        adjustment_turn_power = angle_error * TURN_SPEED;
        adjustment_turn_power = Range.clip(adjustment_turn_power, -TURN_SPEED, TURN_SPEED);

        frontLeftDrive.setPower(-adjustment_turn_power);
        backLeftDrive.setPower(-adjustment_turn_power);
        frontRightDrive.setPower(adjustment_turn_power);
        backRightDrive.setPower(adjustment_turn_power);
        while(Math.abs(angle_error) > TURN_ANGLE_TOLERANCE) {
            telemetry.addLine(message);
            current_heading = imu.getRobotYawPitchRollAngles().getYaw();
            angle_error = target_heading - current_heading;
            telemetry.addLine("IMU heading: " + imu.getRobotYawPitchRollAngles().getYaw() + ", Target heading: " + target_heading);
            telemetry.update();
        }
            // Stop motors or transition to next action
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
            // ... and so on for other motors
    }
}
