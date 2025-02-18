package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm1Subsystem;


public class ARM1RESETCommand extends CommandBase {

    private Arm1Subsystem arm1Subsystem;

    public ARM1RESETCommand(Arm1Subsystem arm1Subsystem) {
        this.arm1Subsystem = arm1Subsystem;

        addRequirements(arm1Subsystem);


    }

    @Override
    public void execute() {

       arm1Subsystem.arm1Print();
        arm1Subsystem.Arm1Reset();
        //arm1Subsystem.gerikol();
    }

    @Override
    public void end(boolean interrupted) {
        //arm1Subsystem.Arm1stop();
        // Stop the motor when the command ends
    //    arm1Subsystem.Arm1stop();

    }


}
