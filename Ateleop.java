package Felipe;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.Func;
import java.util.Locale;

@TeleOp

public class Ateleop extends LinearOpMode {
private static final boolean USE_WEBCAM = true;
private AprilTagProcessor aprilTag;
private VisionPortal visionPortal;

//motores
DcMotorEx DireitaFrente;
DcMotorEx EsquerdaFrente;
DcMotorEx DireitaTras;
DcMotorEx EsquerdaTras;
DcMotorEx Braço;
DcMotorEx BraçoEsquerda;

DistanceSensor SensorDistancia;
Servo Aviao;
BNO055IMU imu;
Orientation angles;

//variáveis
double velocidade;
double mecanum;
double girar;
boolean redução;
boolean matheus;
boolean garra;
double x;
double y;


    public void runOpMode() {
        
        //união objtos java x reais
        EsquerdaTras = hardwareMap.get(DcMotorEx.class,"EsquerdaTras");
        DireitaTras = hardwareMap.get(DcMotorEx.class,"DireitaTras");
        DireitaFrente = hardwareMap.get(DcMotorEx.class,"DireitaFrente");
        EsquerdaFrente = hardwareMap.get(DcMotorEx.class,"EsquerdaFrente");
        SensorDistancia = hardwareMap.get(DistanceSensor.class, "SensorDistancia");
        Braço = hardwareMap.get(DcMotorEx.class,"Braço");
        BraçoEsquerda = hardwareMap.get(DcMotorEx.class,"BraçoEsquerda");
        Aviao = hardwareMap.get(Servo.class,"Aviao");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        telemetry.addData("Aguardando", "Start");
        telemetry.update();
        Aviao.setDirection(Servo.Direction.REVERSE);
        BraçoEsquerda.setDirection(DcMotorEx.Direction.REVERSE);
        waitForStart();
        Aviao.setPosition(0.3);
        //loop principal
        while(opModeIsActive()){
            
///////////// MECANUM /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
           if(gamepad1.left_stick_y > 0 && 13.5 >  SensorDistancia.getDistance(DistanceUnit.CM)){
               continue;
           }
            if(gamepad1.left_bumper == true && matheus == false){
                matheus = true;
            }
            
            if(gamepad1.left_bumper == true && matheus == true){
                matheus = false;
            }
            
           if(matheus == true && SensorDistancia.getDistance(DistanceUnit.CM) > 12) {
            matheus = false;
           DireitaFrente.setPower(0.5);
           EsquerdaFrente.setPower(0.5);
           DireitaTras.setPower(0.5);
           EsquerdaTras.setPower(0.5);
           }
            
            velocidade = -gamepad1.left_stick_y;
            mecanum = gamepad1.left_stick_x;
            girar = gamepad1.right_stick_x;
            
            x = 0.7;
            y = 0.4;
            
            if(redução == true){
                DireitaFrente.setPower((velocidade - mecanum - girar)*y);
                EsquerdaFrente.setPower((velocidade - mecanum + girar)*-y);
                DireitaTras.setPower((velocidade + mecanum - girar)*y);
                EsquerdaTras.setPower((velocidade + mecanum + girar)*y);
            }
            else{
                DireitaFrente.setPower((velocidade - mecanum - girar)*x);
                EsquerdaFrente.setPower((velocidade - mecanum + girar)*-x);
                DireitaTras.setPower((velocidade + mecanum - girar)*x);
                EsquerdaTras.setPower((velocidade + mecanum + girar)*x);   
            }
            
/////////////Redução ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            if (redução == true && gamepad1.y == true){
                redução = false;
                sleep (200);
            }
            
             if (redução == false && gamepad1.y == true){
                redução = true;
                sleep (200);
            }
//////////// BRAÇO /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //braço estava muito rapido
            double b = 0.3;
            if(gamepad2.b == true && Braço.getCurrentPosition() < -600 ){
                continue;
            }
            
            if(gamepad2.a == true && Braço.getCurrentPosition() > 500 ){
                continue;
            }
            
            if (gamepad2.a == true){////positivo é para baixo
            Braço.setPower(b);
            BraçoEsquerda.setPower(b);
            }
            
            if (gamepad2.b == true){
            Braço.setPower(-b);
            BraçoEsquerda.setPower(-b);
            } 
            //karol pediu para diminuir a velo no final
            else if (gamepad2.b && Braço.getCurrentPosition() <  -300){
                Braço.setPower(-b*0.5);
            }
            
            if(gamepad2.b == false && gamepad2.a == false){
                Braço.setPower(-0.015);
                BraçoEsquerda.setPower(-0.015);
            }
           
            ////////////////////////////////////
           //pq tem um avião aqui?????????  / ele faz parte da garra na minha cabeça /
            if (gamepad2.y == true){
            Aviao.setPosition (0.5);
            }
            //////////////////////////////////
            if (gamepad2.x == true && garra == false){
                garra = true;
            }
             if (gamepad2.x == true && garra == true){
                garra = false;
            }
            if (garra == true && Braço.getCurrentPosition() < -520){
                garra = false;
                sleep(200);
            }
            if (garra == true && Braço.getCurrentPosition() >-500){
                Braço.setPower(-b);
              
            }
            
////////////// TELEMETRIA ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////            
            telemetry.addData("Y", gamepad1.right_stick_y);
            telemetry.addData("X", gamepad1.left_stick_x);
            telemetry.addData("power", gamepad1.right_trigger);
            telemetry.addData("redução", redução);
            telemetry.addData("Braço", Braço.getCurrentPosition());
            telemetry.addData("BraçoEsquerda", BraçoEsquerda.getCurrentPosition());
            telemetry.addData("Encoder", DireitaFrente.getCurrentPosition());
            telemetry.addData("range", String.format("%.01f cm", SensorDistancia.getDistance(DistanceUnit.CM)));
            telemetry.update();
        }
    }
    
    
   void composeTelemetry() {
    
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
                {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
                }
            });

        telemetry.addLine()
            .addData("status", new Func<String>() {
                @Override public String value() {
                    return imu.getSystemStatus().toShortString();
                    }
                })
            .addData("calib", new Func<String>() {
                @Override public String value() {
                    return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
            .addData("heading", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                });
                
   }
   
   String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
