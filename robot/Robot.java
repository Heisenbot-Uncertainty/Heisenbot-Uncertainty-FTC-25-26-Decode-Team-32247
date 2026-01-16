package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystems.*;

public class Robot {

    public final DriveSubSystem drive;
    public final IntakeSubSystem intake;
    public final CannonSubSystem cannon;
    
    public Robot(HardwareMap hardwareMap) {
        drive = new DriveSubSystem(hardwareMap);
        intake = new IntakeSubSystem(hardwareMap);
        cannon = new CannonSubSystem(hardwareMap);
    }
}