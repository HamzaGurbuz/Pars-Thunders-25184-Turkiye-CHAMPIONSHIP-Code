package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Subsystems.Arm1Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Arm2Subsystem;

public class paraletakmalklipsmode extends ParallelCommandGroup {
    public paraletakmalklipsmode(Arm1Subsystem Arm1, Arm2Subsystem Arm2) {
        addCommands(
                new Arm1takmaklipsmode(Arm1), //
                new Arm2klips(Arm2)     //

        );

    }

}
