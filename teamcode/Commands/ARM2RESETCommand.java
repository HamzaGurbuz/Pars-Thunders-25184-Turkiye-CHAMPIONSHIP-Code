package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm2Subsystem;


public class ARM2RESETCommand extends CommandBase {

    private Arm2Subsystem arm2Subsystem;

    public ARM2RESETCommand(Arm2Subsystem arm2Subsystem) {
        this.arm2Subsystem = arm2Subsystem;

        addRequirements(arm2Subsystem);


    }

    @Override
    public void execute() {

       arm2Subsystem.arm2Print();
        arm2Subsystem.Arm2Reset();
        //arm1Subsystem.gerikol();
        //arm2Subsystem.resetPosition();
    }

    @Override
    public void end(boolean interrupted) {
        //arm1Subsystem.Arm1stop();
        // Stop the motor when the command ends
    //    arm1Subsystem.Arm1stop();

    }


}
