package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Constants;

public class DriveSubSystem {

    private DcMotor leftRear, rightRear, leftFront, rightFront;

    int encodersPerDecimeter;
    int encodersPerDegree;
    int encoders, turnEncoders;

    public DriveSubSystem(HardwareMap hardwareMap) {
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        encodersPerDecimeter = Constants.ENCODERS_PER_DECIMETER;
        encodersPerDegree = Constants.ENCODERS_PER_DEGREE;
    }

    public void setPower(double leftRearPower, double rightRearPower, double leftFrontPower, double rightFrontPower) {
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
    }

    public void moveTank(int distance, double power, int maxTime){
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        encoders = Math.round((encodersPerDecimeter * distance));

        leftRear.setPower(power);
        rightRear.setPower(power);
        leftFront.setPower(power);
        rightFront.setPower(power);
        
        leftRear.setTargetPosition(encoders);
        rightRear.setTargetPosition(encoders);
        leftFront.setTargetPosition(encoders);
        rightFront.setTargetPosition(encoders);
        
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        endMovement(maxTime);
    }
    
    public void moveStrafe(int distance, double power, int maxTime){
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        encoders = Math.round((encodersPerDecimeter * distance));

        leftRear.setPower(power);
        rightRear.setPower(power);
        leftFront.setPower(power);
        rightFront.setPower(power);
        
        leftRear.setTargetPosition(encoders);
        rightRear.setTargetPosition(-encoders);
        leftFront.setTargetPosition(-encoders);
        rightFront.setTargetPosition(encoders);
        
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        endMovement(maxTime);
    }
    
    public void moveTurn(int degrees, double power, int maxTime){
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        turnEncoders = Math.round((encodersPerDegree * degrees));

        leftRear.setPower(power);
        rightRear.setPower(power);
        leftFront.setPower(power);
        rightFront.setPower(power);
        
        leftRear.setTargetPosition(turnEncoders);
        rightRear.setTargetPosition(-turnEncoders);
        leftFront.setTargetPosition(turnEncoders);
        rightFront.setTargetPosition(-turnEncoders);
        
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        endMovement(maxTime);
    }
    
    public void endMovement(int maxTime){
    ElapsedTime timer = new ElapsedTime();
    timer.reset();

    while (true) {
        boolean atTarget =
            Math.abs(leftRear.getCurrentPosition()  - leftRear.getTargetPosition())  <= 20 &&
            Math.abs(rightRear.getCurrentPosition() - rightRear.getTargetPosition()) <= 20 &&
            Math.abs(leftFront.getCurrentPosition() - leftFront.getTargetPosition()) <= 20 &&
            Math.abs(rightFront.getCurrentPosition()- rightFront.getTargetPosition())<= 20;

        if (atTarget || timer.seconds() >= maxTime) {
            leftRear.setPower(0);
            rightRear.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);

            leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftRear.setTargetPosition(0);
            rightRear.setTargetPosition(0);
            leftFront.setTargetPosition(0);
            rightFront.setTargetPosition(0);

            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            break;
        }
        }
    }
    
    public void stop() {
        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }
}