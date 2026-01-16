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
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Constants;

public class CannonSubSystem {

    private DcMotorEx cannonMotor;
    private Servo loadingServo;
    
    double cannonRPMClose, cannonRPMFar, loadMinPos, loadMaxPos;

    public CannonSubSystem(HardwareMap hardwareMap) {
        cannonMotor = hardwareMap.get(DcMotorEx.class, "cannonMotor");
        loadingServo = hardwareMap.get(Servo.class, "loadingServo");
    
        cannonRPMClose = Constants.CANNON_RPM_CLOSE;
        cannonRPMFar = Constants.CANNON_RPM_FAR;
        loadMinPos = Constants.LOAD_SERVO_MIN_POS;
        loadMaxPos = Constants.LOAD_SERVO_MAX_POS;
        
        cannonMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                cannonMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    
    public void shootClose() {
        //cannonMotor.setPower(1);
           cannonMotor.setVelocity((500 * 112) / 60);
        while (true) {
            // Make sure to not continue the firing process until motor is to speed.
            if (Math.abs(cannonMotor.getVelocity() - cannonRPMClose) <= 20) {
                break;
            }
        } 
    }
    
    public void shootFar() {
      //  cannonMotor.setPower(1);
          cannonMotor.setVelocity((500 * 112) / 60);
        while (true) {
            // Make sure to not continue the firing process until motor is to speed.
            if (Math.abs(cannonMotor.getVelocity() - cannonRPMFar) <= 20) {
                break;
            }
        }
    }
    
    public void harpoonersFire() {
        // Push ball into cannon wheel.
        loadingServo.setPosition(loadMaxPos);
    }
    
    public void reset() {
        // Put load servo back for another ball
        loadingServo.setPosition(loadMinPos);
    }
    
    public void idle() {
     cannonMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the cannon motor to idle to make speed up for shooting take less time.
      //  cannonMotor.setPower(1);
       cannonMotor.setVelocity((50 * 112) / 60);
    }
}