package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Subsystems.Arm1Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Arm2Subsystem;

public class paralelresetmode extends ParallelCommandGroup {
    public paralelresetmode(Arm1Subsystem Arm1, Arm2Subsystem Arm2) {
        addCommands(
                new ARM1RESETCommand(Arm1), //
                new ARM2RESETCommand(Arm2)     //

        );

    }

}
