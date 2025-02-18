package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.WattWhell;

public class Arm2Subsystem extends SubsystemBase {

    private final DcMotor arm2Motor;
    private final WattWhell arm2Watt;
  //  private  final Servo intakeservo;
    private final Servo continuousServo; // 360 derece servo

    private final FtcDashboard dashboard; // FTC Dashboard için

    public double currentPosition = 0; // Sürekli okuma için değişken

    public double maxSpeed = 0.7;  // Daha düşük maksimum motor gücü
    public double maxAccel = 0.1; // Daha düşük maksimum ivme
    public double targetPosition; // Hedef pozisyon (encoder ticks)
    public double currentSpeed = 0; // Şu anki hız
    public double timeStep = 0.050; // Her döngüde geçen süre (20ms) 0.035
    public final double positionTolerance = 50; // Hedef pozisyona olan tolerans (ticks)

    // PID kontrol parametreleri
    public double kP = 0.0009; //0.0010
    public double kI = 0.00003;
    public double kD = 0.00015;

    public double integral = 0;
    public double lastError = 0;
    public final double integralLimit = 50; // İntegral terim sınır değeri
    public final double dampingFactor = 0.99; // Türev sönümleme faktörü



    public Arm2Subsystem(HardwareMap hardwareMap) {

        arm2Motor = hardwareMap.get(DcMotor.class, "Arm2Motor");
        arm2Watt = hardwareMap.get(WattWhell.class, "Arm2Watt");
       // continuousServo = hardwareMap.get(continuousServo.class, "intakeservo");
        continuousServo = hardwareMap.get(Servo.class, "ContinuousServo"); // 360 derece servo

        arm2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // FTC Dashboard başlat
        dashboard = FtcDashboard.getInstance();

        // Başlangıç pozisyonunu oku
        currentPosition = arm2Watt.getPosition();
    }

    @Override
    public void periodic() {
        // Sürekli pozisyon güncellemesi
        currentPosition = arm2Watt.getPosition();

        // FTC Dashboard için bir TelemetryPacket oluştur
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Arm2 WattWhell Position", currentPosition);

        // Packet'i FTC Dashboard'a gönder
        dashboard.sendTelemetryPacket(packet);
    }

    public void arm2Print() {
        // Telemetry'yi güncel tutmak için sadece dashboard üzerinden bilgi gönderiliyor
    }



    public void Arm2stop() {
        arm2Motor.setPower(0);
        stopContinuousServo();

    }
    public  void servostop(){

    }

    public void Arm2Reset() {
        arm2Watt.resetPosition();
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
        update();
    }

    public void update() {
        double currentPos = arm2Watt.getPosition();
        double distanceToGo = targetPosition - currentPos;

        // Eğer hedefe çok yakınsa motoru durdur
        if (Math.abs(distanceToGo) < positionTolerance) {
            arm2Motor.setPower(0);
            currentSpeed = 0;
            integral = 0;
            lastError = 0;
            return;
        }

        // PID kontrol hesaplamaları
        double error = distanceToGo;

        // İntegral birikmesini sınırla (Anti-Windup)
        integral += error * timeStep;
        integral = Math.max(-integralLimit, Math.min(integralLimit, integral));

        // Türev hesaplaması ve sönümleme
        double derivative = (error - lastError) / timeStep;
        derivative *= dampingFactor;

        // PID kontrol çıkışı
        double pidOutput = (kP * error) + (kI * integral) + (kD * derivative);
        lastError = error;

        // PID çıkışını sınırla
        pidOutput = Math.max(-maxSpeed, Math.min(maxSpeed, pidOutput));

        // Motor gücünü uygula (PID çıktısı)
        arm2Motor.setPower(pidOutput*-1);
    }



    public void aci2bekleme() {
        setTargetPosition(2600);
    }

    public void aci3intake() {
        setTargetPosition(4100);
        startContinuousServo(1);

        }

    public void  servointake(){

    }
    public void aci4basket() {
        setTargetPosition(100);
    }

    public void aci4basketOtonom() {
        setTargetPosition(150);
    }

    public void aci5klips() {
        setTargetPosition(750);
    }

    public void aci8goto0(){
        setTargetPosition(100);

    }

    //int  klipsasma = 1;
    public  void  aci7asmamodu(){
        setTargetPosition(400);
    }

    public void intaketukur(){
        continuousServo.setPosition(-1);
    }


    public void denemekol() {
        arm2Motor.setPower(-1);
    }

    public void gerikol() {
        arm2Motor.setPower(1);
    }

    public void resetPosition() {
        arm2Watt.resetPosition();
    }

    public void startContinuousServo(double speed) {
        // Hızı ayarla (1: Saat yönü, -1: Saatin ters yönü)
        continuousServo.setPosition(0.5 + (speed / 2));
    }

    // 360 derece servo durdurma komutu
    public void stopContinuousServo() {
        continuousServo.setPosition(0.5); // Servo durur
    }




}
