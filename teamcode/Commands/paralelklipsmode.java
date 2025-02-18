package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Subsystems.Arm1Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Arm2Subsystem;

public class paralelklipsmode extends ParallelCommandGroup {
    public paralelklipsmode(Arm1Subsystem Arm1, Arm2Subsystem Arm2) {
        addCommands(
                new Arm1klipsmode(Arm1), //
                new Arm2klips(Arm2)     //

        );

    }

}
