package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubSystem {

    private DcMotor intakeMotor;

    public IntakeSubSystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
    }

    public void setPower(double intakePower) {
        intakeMotor.setPower(intakePower);
    }
    public void stop() {
        intakeMotor.setPower(0);
    }
}