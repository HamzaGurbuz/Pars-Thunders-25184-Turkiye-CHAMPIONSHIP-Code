package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm1Subsystem;


public class arm1intakemode extends CommandBase {

    private Arm1Subsystem arm1Subsystem;

    public arm1intakemode(Arm1Subsystem armSubsystem) {
        this.arm1Subsystem = armSubsystem;

        addRequirements(armSubsystem);


    }

    @Override
    public void execute() {
      //arm1Subsystem.moveToPosition(-500);
       // arm1Subsystem.denemekol();
        arm1Subsystem.aci2intake();
        arm1Subsystem.arm1Print();
    }

    @Override
    public void end(boolean interrupted) {
        arm1Subsystem.Arm1stop();
        // Stop the motor when the command ends

    }


}
