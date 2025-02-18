package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm2Subsystem;


public class Arm2havuz extends CommandBase {

    private Arm2Subsystem arm2Subsystem;

    public Arm2havuz(Arm2Subsystem arm2Subsystem) {
        this.arm2Subsystem = arm2Subsystem;

        addRequirements(arm2Subsystem);


    }

    @Override
    public void execute() {

       arm2Subsystem.aci3intake();
       // arm2Subsystem.arm2Print();
    }

    @Override
    public void end(boolean interrupted) {

        //arm2Subsystem.aci4();
        arm2Subsystem.arm2Print();
        arm2Subsystem.Arm2stop();
        // Stop the motor when the command ends

    }


}
