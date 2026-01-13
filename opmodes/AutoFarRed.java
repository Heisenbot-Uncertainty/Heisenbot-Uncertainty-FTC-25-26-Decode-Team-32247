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
import org.firstinspires.ftc.teamcode.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import org.firstinspires.ftc.teamcode.util.Constants;
import java.util.List;



@Autonomous
public class AutoFarRed extends LinearOpMode {

    private Robot robot;

    private DcMotor leftRear, rightRear, leftFront, rightFront;
    private DcMotor intakeMotor, cannonMotor;
    private Servo loadingServo;
    
    double wheelPowerTiny;
    double wheelPowerLarge;
    double intakePower;
    double cannonPowerClose;
    double cannonPowerFar;
    double loadMinPos;
    double loadMaxPos;
    double purpleRedPercentage;
    double purpleGreenPercentage;
    double purpleBluePercentage;
    double greenRedPercentage;
    double greenGreenPercentage;
    double greenBluePercentage;
    double blueFarRange;
    double blueFarX;
    double blueFarY;
    double blueFarYaw;
    double redFarRange;
    double redFarX;
    double redFarY;
    double redFarYaw;
    double errorThreshold;
    double aprilTagErrorThreshold;
    double aprilTagYawErrorThreshold;
    
    boolean manualMode = false;
    boolean debugMode = false;
    boolean toggle;
    boolean abort = false;
    int targetID = 24;

    int goalColor = 24;

    AprilTagProcessor aprilTagProcessor;
    VisionPortal visionPortal;
    ColorSensor colorSensor;

    @Override
    public void runOpMode() {

        robot = new org.firstinspires.ftc.teamcode.robot.Robot(hardwareMap);

        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        cannonMotor = hardwareMap.get(DcMotor.class, "cannonMotor");
        loadingServo = hardwareMap.get(Servo.class, "loadingServo");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cannonMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cannonMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRear.setTargetPosition(0);
        rightRear.setTargetPosition(0);
        leftFront.setTargetPosition(0);
        rightFront.setTargetPosition(0);

        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        cannonMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

wheelPowerTiny = Constants.WHEEL_POWER_TINY;
        wheelPowerLarge = Constants.WHEEL_POWER_LARGE;
        intakePower = Constants.INTAKE_POWER;
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



        initializeVisionPortal();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            sleep(5000);
            robot.drive.moveTank(3, .5, 3);
            robot.drive.moveTurn(22, .5, 3);
            robot.drive.moveStrafe(2, .5, 3);
            shootDemBalls("far");
            sleep(5000);
            shootDemBalls("far");
        }
    }

    public void shootDemBalls(String distance) {

        if (distance == "close") {
            robot.cannon.shootClose();
        } else if (distance == "far") {
            robot.cannon.shootFar();
        }

        sleep(4000);

        robot.cannon.harpoonersFire();
        sleep(5000);

        robot.cannon.reset();
        robot.cannon.stop();
    }

    public void initializeVisionPortal() {

        VisionPortal.Builder visionPortalBuilder = new VisionPortal.Builder();
        AprilTagLibrary.Builder tagLibraryBuilder = new AprilTagLibrary.Builder();

        tagLibraryBuilder.addTag(new AprilTagMetadata(20, "bluGoal", 0.166, DistanceUnit.METER));
        tagLibraryBuilder.addTag(new AprilTagMetadata(21, "GPP",     0.166, DistanceUnit.METER));
        tagLibraryBuilder.addTag(new AprilTagMetadata(22, "PGP",     0.166, DistanceUnit.METER));
        tagLibraryBuilder.addTag(new AprilTagMetadata(23, "PPG",     0.166, DistanceUnit.METER));
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
    
    public void findThatWhale() {
        if (!gamepad2.rightBumperWasPressed()) return;
        // For some reason doing "if (gamepad2.rightBumperWasPressed())" breaks this function
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
        if (purpleError < greenError && purpleError < errorThreshold) {
            telemetry.addData("Color", "PURPLE");
        } else if (greenError < purpleError && greenError < errorThreshold) {
            telemetry.addData("Color", "GREEN");
        } else {
            telemetry.addData("Color", "NULL");
        }
    }

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
    
    public void debugToggle() {
        if ((gamepad1.a || gamepad2.a) && debugMode == false) {
            debugMode = true;
            toggle = true;
        } else if ((gamepad1.a || gamepad2.a) && debugMode == true) {
            sleep(500);
            if (toggle == false) {
                debugMode = false;  
            }
        }
    }
}
