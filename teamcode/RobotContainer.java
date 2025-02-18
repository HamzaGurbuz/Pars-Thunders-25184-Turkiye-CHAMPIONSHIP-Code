package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Commands.intaketukurmode;
import org.firstinspires.ftc.teamcode.Commands.paralelbasketmode;
import org.firstinspires.ftc.teamcode.Commands.paralelbeklememode;
import org.firstinspires.ftc.teamcode.Commands.paralelhavuzmode;
import org.firstinspires.ftc.teamcode.Commands.paralelklipsmode;
import org.firstinspires.ftc.teamcode.Commands.paralelresetmode;
import org.firstinspires.ftc.teamcode.Commands.paraletakmalklipsmode;
import org.firstinspires.ftc.teamcode.Subsystems.Arm1Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Arm2Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class RobotContainer {
    private final DriveSubsystem driveSubsystem;
    private final GamepadEx operatorGamepad;
    private final Arm1Subsystem arm1Subsystem;
    private final Arm2Subsystem arm2Subsystem;




    private final DriveCommand driveCommand;
    //private final MoveArmsCommand moveArmsCommand;

    public RobotContainer(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2){
        driveSubsystem = new DriveSubsystem(hardwareMap);
        operatorGamepad = new GamepadEx(gamepad2);
        arm1Subsystem = new Arm1Subsystem(hardwareMap);
        arm2Subsystem = new Arm2Subsystem(hardwareMap);



        driveCommand = new DriveCommand(driveSubsystem, gamepad1);
        //moveArmsCommand = new MoveArmsCommand(arm1Subsystem);


        driveSubsystem.setDefaultCommand(driveCommand);
        //arm1Subsystem.setDefaultCommand(moveArmsCommand);
        configureBindings();



    }


    public void configureBindings() {


        arm1Subsystem.arm1Print();
        arm2Subsystem.arm2Print();


        new GamepadButton(operatorGamepad, GamepadKeys.Button.START)
                .whenHeld(new paralelresetmode(arm1Subsystem,arm2Subsystem));

        new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
                .whenHeld(new paralelbeklememode(arm1Subsystem,arm2Subsystem));


        new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
                .whenHeld(new paralelhavuzmode(arm1Subsystem,arm2Subsystem));


        new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
                .whenHeld(new paralelbasketmode(arm1Subsystem,arm2Subsystem));


        new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenHeld(new paralelklipsmode(arm1Subsystem,arm2Subsystem));

        new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP)
                .whenHeld(new paraletakmalklipsmode(arm1Subsystem,arm2Subsystem));


        new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(new intaketukurmode(arm2Subsystem));

    }

    public void run() {
        CommandScheduler.getInstance().run();


    }
}
