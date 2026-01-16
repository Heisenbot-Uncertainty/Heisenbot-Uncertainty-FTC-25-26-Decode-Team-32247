/*
  Copyright 2026 FIRST Tech Challenge Team 32247 FTC
 
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
 
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
 
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
 */
package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import android.graphics.Color;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import java.util.List;

@TeleOp(name = "TeleOp Main", group = "TeleOp")
public class TeleOpMain extends LinearOpMode {
    
    private org.firstinspires.ftc.teamcode.robot.Robot robot;
        double wheelPowerTiny, wheelPowerLarge,
            cannonPowerClose, cannonPowerFar,
            loadMinPos, loadMaxPos,
            purpleRedPercentage, purpleGreenPercentage, purpleBluePercentage,
            greenRedPercentage, greenGreenPercentage, greenBluePercentage,
            blueFarRange, blueFarX, blueFarY, blueFarYaw,
            redFarRange, redFarX, redFarY, redFarYaw,
            errorThreshold, aprilTagErrorThreshold, aprilTagYawErrorThreshold;

        int targetID = 24;

        boolean manualMode = false, 
            debugMode = false, 
            toggle = false, 
            abort = false;
    
    // For auto aim loop.
    public boolean goalTagDetected() {
    List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
    for (AprilTagDetection detection : detections) {
        if (detection.id == targetID && detection.ftcPose != null) {
            return true;
        }
    }
    return false;
}

    AprilTagProcessor aprilTagProcessor;
    VisionPortal visionPortal;
    ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        robot = new org.firstinspires.ftc.teamcode.robot.Robot(hardwareMap);

        wheelPowerTiny = Constants.WHEEL_POWER_TINY;
        wheelPowerLarge = Constants.WHEEL_POWER_LARGE;
        cannonPowerClose = Constants.CANNON_POWER_CLOSE;
        cannonPowerFar = Constants.CANNON_POWER_FAR;
        loadMinPos = Constants.LOAD_SERVO_MIN_POS;
        loadMaxPos = Constants.LOAD_SERVO_MAX_POS;
        purpleRedPercentage = Constants.PURPLE_RED_PERCENTAGE;
        purpleGreenPercentage = Constants.PURPLE_GREEN_PERCENTAGE;
        purpleBluePercentage = Constants.PURPLE_BLUE_PERCENTAGE;
        greenRedPercentage = Constants.GREEN_RED_PERCENTAGE;
        greenGreenPercentage = Constants.GREEN_GREEN_PERCENTAGE;
        greenBluePercentage = Constants.GREEN_BLUE_PERCENTAGE;
        errorThreshold = Constants.ERROR_THRESHOLD;
        aprilTagErrorThreshold = Constants.APRIL_TAG_ERROR_THRESHOLD;
        aprilTagYawErrorThreshold = Constants.APRIL_TAG_YAW_ERROR_THRESHOLD;

        blueFarRange = Constants.BLUE_FAR_RANGE;
        blueFarX = Constants.BLUE_FAR_X;
        blueFarY = Constants.BLUE_FAR_Y;
        blueFarYaw = Constants.BLUE_FAR_YAW;
        
        redFarRange = Constants.RED_FAR_RANGE;
        redFarX = Constants.RED_FAR_X;
        redFarY = Constants.RED_FAR_Y;
        redFarYaw = Constants.RED_FAR_YAW;
        
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        
        initializeVisionPortal();

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();
            // Dont start cannon idle and load servo until after teleop initialize phase.
            robot.cannon.reset();
            robot.cannon.idle();
            telemetry.addData("Target", "RED");
        while (opModeIsActive()) {
             robot.cannon.idle();
            wheelMath();
            intakeTrigger();
            callMeIshmael();
            findThatWhale();
            colorSensorMath();
            targetToggle();
            debugToggle();
            if (debugMode == true) {
                displayVisionPortalData();
            }
            telemetry.update();
        }
    }

    public void initializeVisionPortal() {
        VisionPortal.Builder visionPortalBuilder = new VisionPortal.Builder();
        AprilTagLibrary.Builder tagLibraryBuilder = new AprilTagLibrary.Builder();
        // All tags used this year.
        tagLibraryBuilder.addTag(new AprilTagMetadata(20, "bluGoal", 0.166, DistanceUnit.METER));
        tagLibraryBuilder.addTag(new AprilTagMetadata(21, "GPP", 0.166, DistanceUnit.METER));
        tagLibraryBuilder.addTag(new AprilTagMetadata(22, "PGP", 0.166, DistanceUnit.METER));
        tagLibraryBuilder.addTag(new AprilTagMetadata(23, "PPG", 0.166, DistanceUnit.METER));
        tagLibraryBuilder.addTag(new AprilTagMetadata(24, "redGoal", 0.166, DistanceUnit.METER));
        AprilTagLibrary decodeTagLibrary = tagLibraryBuilder.build();

        aprilTagProcessor = new AprilTagProcessor.Builder()
            .setTagLibrary(decodeTagLibrary)
            .build();

        visionPortalBuilder
            .setCamera(hardwareMap.get(WebcamName.class, "turretCam"))
            .addProcessor(aprilTagProcessor);

        visionPortal = visionPortalBuilder.build();
    }

    public void displayVisionPortalData() {
        List<AprilTagDetection> aprilTagDetections = aprilTagProcessor.getDetections();
        // This only displays if debug mode is on.
        for (AprilTagDetection detection : aprilTagDetections) {
            telemetry.addData("ID", detection.id);
                if (detection.id == targetID && detection.ftcPose != null) {
                } else if (detection.ftcPose != null) {
                    telemetry.addData("Range", detection.ftcPose.range);
                    telemetry.addData("X", detection.ftcPose.x);
                    telemetry.addData("Y", detection.ftcPose.y);
                    telemetry.addData("Yaw", detection.ftcPose.yaw);
                } else if (detection.ftcPose == null) {
                    // Fallback if tag id is unknown to not break code.
                    telemetry.addLine("Null Data (and that makes me a sad panda˙◠˙)");
                }
        }
    }
    // What sends power/velocity to the cannon.
    public void shootDemBalls(String distance) {
        if (distance == "close") {
            robot.cannon.shootClose();
        } else if (distance == "far") {
            robot.cannon.shootFar();
        }
        // Push ball into cannon wheel
        robot.cannon.harpoonersFire();
        sleep(1000);
        // Reset load servo and wheel rpm
        robot.cannon.reset();
        robot.cannon.idle(); 
    }

    public void wheelMath() {
        // Small movements for dpad controls.
        if (gamepad1.dpad_up) {
                robot.drive.setPower(wheelPowerTiny, wheelPowerTiny, wheelPowerTiny, wheelPowerTiny);
            } else if (gamepad1.dpad_down) {
                robot.drive.setPower(-wheelPowerTiny,-wheelPowerTiny,-wheelPowerTiny,-wheelPowerTiny);
            } else if (gamepad1.dpad_left) {
                robot.drive.setPower(wheelPowerTiny,-wheelPowerTiny,-wheelPowerTiny,wheelPowerTiny);
            } else if (gamepad1.dpad_right) {
                robot.drive.setPower(-wheelPowerTiny,wheelPowerTiny,wheelPowerTiny,-wheelPowerTiny);
            } else {
                // Main, Large movements for joysticks.
                robot.drive.setPower(
                (wheelPowerLarge*(-(gamepad1.right_stick_y - (-gamepad1.right_stick_x)))),
                (wheelPowerLarge*(-(gamepad1.left_stick_y + (-gamepad1.left_stick_x)))),
                (wheelPowerLarge*(-(gamepad1.right_stick_y + (-gamepad1.right_stick_x)))),
                (wheelPowerLarge*(-(gamepad1.left_stick_y - (-gamepad1.left_stick_x))))
                );
        }
    }
    // Controls for intaking and outtaking balls.
    public void intakeTrigger() {
        if (gamepad1.right_trigger > .1) {
            robot.intake.intake();
        } else if (gamepad1.right_bumper) {
            robot.intake.outtake();
        } else {
            robot.intake.stop();
        }
    }
    // Line up for auto aim loop.
    public void callMeIshmael() {
        if (gamepad2.dpad_up) {
            if (targetID == 20) {
                robot.drive.moveTank(2, .5, 2);
                robot.drive.moveStrafe(-5, .5, 2);
                robot.drive.moveTurn(-26, .5, 2);
            }
            if (targetID == 24) {
                robot.drive.moveTank(2, .5, 2);
                robot.drive.moveStrafe(5, .5, 2);
                robot.drive.moveTurn(26, .5, 2);
            }
        }
    }
    // Auto aim loop.
    public void findThatWhale() {
        if (!gamepad2.rightBumperWasPressed()) return;
        // Doing "if (gamepad2.rightBumperWasPressed())" breaks this function (Reason unknown).
            double targetX;
            double targetY;
            double targetRange;
            double targetYaw;

        if (targetID == 24) {
            targetX = redFarX;
            targetY = redFarY;
            targetRange = redFarRange;
            targetYaw = redFarYaw;
        } else if (targetID == 20) {
            targetX = blueFarX;
            targetY = blueFarY;
            targetRange = blueFarRange;
            targetYaw = blueFarYaw;
        } else {
            return;
        }

        AprilTagDetection detection;
        // Abandon all hope, ye who enter here.
        while (opModeIsActive()) {
            if (gamepad2.a) {
                abort = true;
                break;
            }
            detection = null;
            for (AprilTagDetection d : aprilTagProcessor.getDetections()) {
                if (d.id == targetID && d.ftcPose != null) {
                    detection = d;
                    break;
                }
            }

        if (detection == null) {
            robot.drive.stop();
            idle();
            continue;
        }

        double yawError = detection.ftcPose.yaw - targetYaw;

        if (Math.abs(yawError) <= aprilTagYawErrorThreshold) {
            robot.drive.stop();
            break;
        }

        double turn = yawError * 0.01;
        turn = Math.max(-0.2, Math.min(0.2, turn));

        robot.drive.setPower(
            turn,
           -turn,
            turn,
           -turn
        );
        idle();
        if (gamepad2.a) {
            break;
        }
    }

    while (opModeIsActive()) {
        if (gamepad2.a) {
            abort = true;
            break;
            }
        detection = null;
        for (AprilTagDetection d : aprilTagProcessor.getDetections()) {
            if (d.id == targetID && d.ftcPose != null) {
                detection = d;
                break;
            }
        }

        if (detection == null) {
            robot.drive.stop();
            idle();
            continue;
        }

        double xError = detection.ftcPose.x - targetX;

        if (Math.abs(xError) <= aprilTagErrorThreshold) {
            robot.drive.stop();
            break;
        }

        double strafe = -xError * 0.15;
        strafe = Math.max(-0.25, Math.min(0.25, strafe));

        robot.drive.setPower(
            strafe,
           -strafe,
           -strafe,
            strafe
        );
        idle();
    }

    while (opModeIsActive()) {
        if (gamepad2.a) {
            abort = true;
            break;
            }
        detection = null;
        for (AprilTagDetection d : aprilTagProcessor.getDetections()) {
            if (d.id == targetID && d.ftcPose != null) {
                detection = d;
                break;
            }
        }

        if (detection == null) {
            robot.drive.stop();
            idle();
            continue;
        }

        double rangeError = detection.ftcPose.range - targetRange;

        if (Math.abs(rangeError) <= aprilTagErrorThreshold) {
            robot.drive.stop();
            break;
        }

        double forward = rangeError * 0.15;
        forward = Math.max(-0.25, Math.min(0.25, forward));

        robot.drive.setPower(
            forward,
            forward,
            forward,
            forward
        );
        idle();
    }
    robot.drive.stop();
    if (abort == false) {
    shootDemBalls("far");
    } else {
        abort = false;
    }
}

    public void colorSensorMath() {
        double total = colorSensor.red() + colorSensor.green() + colorSensor.blue();
        // Get RGB in percent.
        double r = colorSensor.red() * 100.0 / total;
        double g = colorSensor.green() * 100.0 / total;
        double b = colorSensor.blue() * 100.0 / total;
        double purpleError =
            Math.abs(r - purpleRedPercentage) +
            Math.abs(g - purpleGreenPercentage) +
            Math.abs(b - purpleBluePercentage);
        double greenError =
            Math.abs(r - greenRedPercentage) +
            Math.abs(g - greenGreenPercentage) +
            Math.abs(b - greenBluePercentage);
            // Find if color is purple, green, or neither using RGB percent with an error threshold.
        if (purpleError < greenError && purpleError < errorThreshold) {
            telemetry.addData("Color", "PURPLE");
        } else if (greenError < purpleError && greenError < errorThreshold) {
            telemetry.addData("Color", "GREEN");
        } else {
            telemetry.addData("Color", "NULL");
        }
    }
    // Change what goal to look for during auto aim.
    public void targetToggle() {
        if (gamepad2.y && targetID == 20) {
            targetID = 24;
            toggle = true;
        } else if (gamepad2.y && targetID == 24) {
            sleep(500);
            if (toggle == false) {
            targetID = 20;
            }
        }
            if (targetID == 20) {
                telemetry.addData("Target", "BLU");
            } else if (targetID == 24) {
                telemetry.addData("Target", "RED");
            }
    }
    // Stuff for the nerds.
    public void debugToggle() {
        if ((gamepad1.x || gamepad2.x) && debugMode == false) {
            debugMode = true;
            toggle = true;
        } else if ((gamepad1.x || gamepad2.x) && debugMode == true) {
            sleep(500);
            if (toggle == false) {
                debugMode = false;  
            }
        }
            telemetry.addData("DebugMode", debugMode);
    }
}