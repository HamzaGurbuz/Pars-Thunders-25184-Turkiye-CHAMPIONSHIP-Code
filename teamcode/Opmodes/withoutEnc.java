package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Arm1Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Arm2Subsystem;
import org.firstinspires.ftc.teamcode.drive.WattWhell;

@Autonomous(name = "4 Basket Auto")
public class withoutEnc extends LinearOpMode {

    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private FtcDashboard dashboard;
    public WattWhell armwatt1, armwatt2;

    private Arm1Subsystem arm1Subsystem;
    private Arm2Subsystem arm2Subsystem;

    private DcMotor arm1Motor;
    private DcMotor arm2Motor;

    private int leftFrontPos = 0;
    private int rightFrontPos = 0;
    private int leftBackPos = 0;
    private int rightBackPos = 0;

    @Override
    public void runOpMode() {
        setupTelemetry();
        initializeHardware();

        TelemetryPacket packet = new TelemetryPacket();
        double arm1pos = armwatt1.getPosition();
        double arm2pos = armwatt2.getPosition();

        arm1pos = arm1pos *0;
        arm2pos = arm2pos *0;

        //arm1Subsystem.Arm1Reset();
        //arm1Subsystem.Arm1Reset();
        packet.put("Watt1", arm1pos);
        packet.put("Watt2", arm2pos);

        telemetry.update();
        dashboard.sendTelemetryPacket(packet);

        arm1Subsystem = new Arm1Subsystem(hardwareMap);
        arm2Subsystem = new Arm2Subsystem(hardwareMap);
        resetDriveEncoders();





        waitForStart();


        arm1pos = arm1pos *0;
        arm2pos = arm2pos *0;
        sleep(500);

        executeAutonomousRoutine();
    }


    public void setupTelemetry() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(10);
    }


    public void initializeHardware() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        arm1Motor = hardwareMap.get(DcMotor.class, "Arm1Motor");
        arm2Motor = hardwareMap.get(DcMotor.class, "Arm2Motor");

        armwatt1 = hardwareMap.get(WattWhell.class, "Arm1Watt");
        armwatt2 = hardwareMap.get(WattWhell.class, "Arm2Watt");

        configureMotorDirections();
        configureMotorBehavior();

        dashboard = FtcDashboard.getInstance();
    }

    public void configureMotorDirections() {
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
    }

    public void configureMotorBehavior() {
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void resetDriveEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void executeAutonomousRoutine() {



        //resetArms();
        drive(300, 300, 300, 300, 0.5);
        sleep(500);

        drive(-580, 580, 580, -580, 0.7);
        drive(380, 380, -380, -380, 0.6);
        sleep(500);

        moveBasket(-1300, -210);
        sleep(1850);

        arm2Subsystem.intaketukur();
        sleep(500);

        ///////////////////////////////////////////////////////

        /// Step 2: Take The Second Sample To Spike Band

        moveBasketBack(0, 0);
        sleep(1100);


        moveBasketBack(0, -1600);
        sleep(1400);

        arm2Subsystem.startContinuousServo(1);

        drive(-60, -60, -60, -60, 0.4);
        sleep(100);
        drive(-280, -280, 280, 280, 0.2);

        arm2Subsystem.startContinuousServo(1);
        drive(30, 30, 30, 30, 0.2);
        sleep(1000);

        /////////////////////////////////////////////////////////

        /// Step 3: Drop Second Sample






        /////////////////////////////////////////////////////////
    }

    public void resetArms() {
        arm1Subsystem.Arm1Reset();
        arm2Subsystem.Arm2Reset();
    }

    public void stopArms() {
        arm1Subsystem.Arm1stop();
        arm2Subsystem.Arm2stop();
    }

    public void drive(int leftFrontTarget, int leftBackTarget, int rightFrontTarget, int rightBackTarget, double speed) {
        leftFrontPos += leftFrontTarget;
        leftBackPos += leftBackTarget;
        rightFrontPos += rightFrontTarget;
        rightBackPos += rightBackTarget;

        leftFront.setTargetPosition(leftFrontPos);
        leftBack.setTargetPosition(leftBackPos);
        rightFront.setTargetPosition(rightFrontPos);
        rightBack.setTargetPosition(rightBackPos);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);
    }

    public void moveBasket(int arm1pos, int arm2pos) {
        arm1Motor.setTargetPosition(arm1pos);
        arm1Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1Motor.setPower(0.7);

        arm2Motor.setTargetPosition(arm2pos);
        arm2Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2Motor.setPower(0.7);
    }

    public void moveBasketBack(int arm1pos, int arm2pos) {
        arm1Motor.setTargetPosition(arm1pos);
        arm1Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1Motor.setPower(0.4);

        arm2Motor.setTargetPosition(arm2pos);
        arm2Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2Motor.setPower(0.4);
    }
}
