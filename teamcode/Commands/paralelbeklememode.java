package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Subsystems.Arm1Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Arm2Subsystem;

public class paralelbeklememode extends ParallelCommandGroup {
    public paralelbeklememode(Arm1Subsystem Arm1, Arm2Subsystem Arm2) {
        addCommands(
                new arm1intakemode(Arm1), //
                new Arm2intake(Arm2)     //

        );

    }

}
