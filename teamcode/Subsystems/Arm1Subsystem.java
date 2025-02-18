package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.WattWhell;

public class Arm1Subsystem extends SubsystemBase {

    private final DcMotor arm1Motor;
    private final WattWhell arm1Watt;
    private final FtcDashboard dashboard; // FTC Dashboard için

    private double currentPosition = 0; // Sürekli okuma için değişken

  //  private double currentPosition = 0; // Sürekli okuma için değişken

    public double maxSpeed = 1;  // Daha düşük maksimum motor gücü
    public double maxAccel = 0.1; // Daha düşük maksimum ivme
    public double targetPosition; // Hedef pozisyon (encoder ticks)
    public double currentSpeed = 0; // Şu anki hız
    public double timeStep = 0.040; // Her döngüde geçen süre (20ms) 0.010
    public final double positionTolerance = 50; // Hedef pozisyona olan tolerans (ticks)

    // PID kontrol parametreleri
    public double kP = 0.0021;
    public double kI = 0.00003;
    public double kD = 0.00016;

    public double integral = 0;
    public double lastError = 0;
    public final double integralLimit = 50; // İntegral terim sınır değeri
    public final double dampingFactor = 0.99; // Türev sönümleme faktörü


    public Arm1Subsystem(HardwareMap hardwareMap) {
        arm1Motor = hardwareMap.get(DcMotor.class, "Arm1Motor");
        arm1Watt = hardwareMap.get(WattWhell.class, "Arm1Watt");

        arm1Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // FTC Dashboard başlat
        dashboard = FtcDashboard.getInstance();

        // Başlangıç pozisyonunu oku
        currentPosition = arm1Watt.getPosition();
    }

    @Override
    public void periodic() {
        // Sürekli pozisyon güncellemesi
        currentPosition = arm1Watt.getPosition();

        // FTC Dashboard için bir TelemetryPacket oluştur
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Arm1 WattWhell Position", currentPosition);

        // Packet'i FTC Dashboard'a gönder
        dashboard.sendTelemetryPacket(packet);
    }

    public void arm1Print() {
        // Telemetry'yi güncel tutmak için sadece dashboard üzerinden bilgi gönderiliyor
    }

    public void Arm1stop() {
        arm1Motor.setPower(0);
    }

    public void Arm1Reset() {

        arm1Watt.resetPosition();
    }


    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
        update();
    }

    public void update() {
        double currentPos = arm1Watt.getPosition();
        double distanceToGo = targetPosition - currentPos;

        // Eğer hedefe çok yakınsa motoru durdur
        if (Math.abs(distanceToGo) < positionTolerance) {
            arm1Motor.setPower(0);
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
        pidOutput = Math.max(-maxSpeed , Math.min(maxSpeed , pidOutput)*0.5);

        // Motor gücünü uygula (PID çıktısı)
        arm1Motor.setPower(pidOutput);
    }



    public void aci1() {
        setTargetPosition(-800);
    }


 public  void aci2intake(){
        setTargetPosition(-600);
 }


    public void aci4basket(){
     setTargetPosition(-2500);

 }

    public void aci5klips(){
        setTargetPosition(-100);

    }

    public void aci6takmaklips(){
        setTargetPosition(-2000);

    }

    public void aci8goto(){
        setTargetPosition(100);

    }


    public void denemekol() {
        arm1Motor.setPower(-1);
    }

    public void gerikol() {
        arm1Motor.setPower(1);
    }
}
