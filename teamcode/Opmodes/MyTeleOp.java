package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotContainer;


@TeleOp(name = "Main TeleOp")
public class MyTeleOp extends LinearOpMode {
    private  RobotContainer robotContainer;


    @Override

    public void runOpMode() {

        robotContainer = new RobotContainer(hardwareMap, gamepad1, gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            robotContainer.run();

            telemetry.setMsTransmissionInterval(11);


        }}}
