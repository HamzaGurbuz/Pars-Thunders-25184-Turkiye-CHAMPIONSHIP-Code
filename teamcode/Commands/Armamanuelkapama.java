package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm1Subsystem;


public class Armamanuelkapama extends CommandBase {

    private Arm1Subsystem arm1Subsystem;

    public Armamanuelkapama(Arm1Subsystem armSubsystem) {
        this.arm1Subsystem = armSubsystem;

        addRequirements(armSubsystem);


    }

    @Override
    public void execute() {
        // Set the arm motor to move upwards
        // Hedef pozisyonu 1000 olarak belirle
       //arm1Subsystem.moveToPosition(500);
       // arm1Subsystem.aci1();
        arm1Subsystem.arm1Print();
        //arm1Subsystem.Arm1Reset();
      arm1Subsystem.gerikol();
    }

    @Override
    public void end(boolean interrupted) {
        //arm1Subsystem.Arm1stop();
        // Stop the motor when the command ends
     arm1Subsystem.Arm1stop();

    }


}
