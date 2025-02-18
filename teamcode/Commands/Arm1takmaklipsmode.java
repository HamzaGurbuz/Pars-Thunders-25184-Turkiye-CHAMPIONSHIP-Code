package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm1Subsystem;


public class Arm1takmaklipsmode extends CommandBase {

    private Arm1Subsystem arm1Subsystem;

    public Arm1takmaklipsmode(Arm1Subsystem armSubsystem) {
        this.arm1Subsystem = armSubsystem;

        addRequirements(armSubsystem);


    }

    @Override
    public void execute() {
      //arm1Subsystem.moveToPosition(-500);
       // arm1Subsystem.denemekol();
        arm1Subsystem.aci6takmaklips();
        arm1Subsystem.arm1Print();
    }

    @Override
    public void end(boolean interrupted) {
        arm1Subsystem.Arm1stop();
        // Stop the motor when the command ends

    }


}
