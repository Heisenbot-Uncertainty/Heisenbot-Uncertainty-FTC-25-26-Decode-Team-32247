package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Constants;

public class CannonSubSystem {

    private DcMotorEx cannonMotor;
    private Servo loadingServo;
    
    double cannonPowerClose;
    double cannonPowerFar;
    double cannonRPMClose;
    double cannonRPMFar;
    double loadMinPos;
    double loadMaxPos;

    public CannonSubSystem(HardwareMap hardwareMap) {
        cannonMotor = hardwareMap.get(DcMotorEx.class, "cannonMotor");
        loadingServo = hardwareMap.get(Servo.class, "loadingServo");
        
        cannonPowerClose = Constants.CANNON_POWER_CLOSE;
        cannonPowerFar = Constants.CANNON_POWER_FAR;
        cannonRPMClose = Constants.CANNON_RPM_CLOSE;
        cannonRPMFar = Constants.CANNON_RPM_FAR;
        loadMinPos = Constants.LOAD_SERVO_MIN_POS;
        loadMaxPos = Constants.LOAD_SERVO_MAX_POS;
    }
    
    public void shootClose() {
        cannonMotor.setPower(cannonPowerClose);
        while (true) {
            if (Math.abs(cannonMotor.getVelocity()) >= cannonRPMClose) {
                break;
            }
        } 
    }
    
    public void shootFar() {
        cannonMotor.setPower(cannonPowerFar);
        while (true) {
            if (Math.abs(cannonMotor.getVelocity()) >= cannonRPMFar) {
                stop();
                break;
            }
        } 
    }
    
    public void harpoonersFire() {
        loadingServo.setPosition(loadMaxPos);
    }
    
    public void reset() {
        loadingServo.setPosition(loadMinPos);
    }
    
    public void stop() {
        cannonMotor.setPower(0);
    }
}
