package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Subsystems.Arm1Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Arm2Subsystem;

public class intekekolmode extends ParallelCommandGroup {
    public intekekolmode(Arm1Subsystem Arm1, Arm2Subsystem Arm2) {
        addCommands(
                new MoveArmsCommand(Arm1), // Slider hareket eder
                new Arm2intake(Arm2)     // Bilek aynı anda hareket eder

        );

    }

}
