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
double x;
double y;

boolean redução;
boolean matheus;
boolean garra;


    public void runOpMode() {
        
        //união objtos java x reais
        EsquerdaTras = hardwareMap.get(DcMotorEx.class,"EsquerdaTras");
        DireitaTras = hardwareMap.get(DcMotorEx.class,"DireitaTras");
        DireitaFrente = hardwareMap.get(DcMotorEx.class,"DireitaFrente");
        EsquerdaFrente = hardwareMap.get(DcMotorEx.class,"EsquerdaFrente");
        SensorDistancia = hardwareMap.get(DistanceSensor.class, "SensorDistancia");
        /*
        ***
        Usar nas fases de teste porque este valor deve ser continuado após o autonomo
        ***
        */
        Braço = hardwareMap.get(DcMotorEx.class,"Braço");
        Braço.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoder
        Braço.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        BraçoEsquerda = hardwareMap.get(DcMotorEx.class,"BraçoEsquerda");
        BraçoEsquerda.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoder
        BraçoEsquerda.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //RUN_TO_POSITION
         
        Aviao = hardwareMap.get(Servo.class,"Aviao");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        telemetry.addData("Aguardando", "Start");
        telemetry.addData("X", gamepad1.left_stick_x);
        telemetry.update();
        Aviao.setDirection(Servo.Direction.REVERSE);
        BraçoEsquerda.setDirection(DcMotorEx.Direction.REVERSE);
        EsquerdaFrente.setDirection(DcMotorEx.Direction.REVERSE);
        Aviao.setPosition(0.2);
        
        
        waitForStart();  ////////start
        //loop principal
        
        while(opModeIsActive()){
            if(gamepad1.left_trigger > 0.5){ // Freia o robô
               power4Rodas(-0.05); continue;
            }
///////////// MECANUM /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            if(gamepad1.left_stick_y > 0 && 13.5 >  SensorDistancia.getDistance(DistanceUnit.CM)){
               continue; 
            }
            if(gamepad1.left_stick_y > 0 && gamepad1.left_stick_x > -0.35 && gamepad1.left_stick_x < 0.35 ){
                power4Rodas(velocidade);
            }
            if ((gamepad1.left_stick_y < 0.05 && gamepad1.left_stick_y > -0.05) && (gamepad1.left_stick_x < 0.05 && gamepad1.left_stick_x > -0.05)){
                power4Rodas(0.015);
            }

            if(gamepad1.left_bumper == true && matheus == false) matheus = true;           
            
            if(gamepad1.left_bumper == true && matheus == true) matheus = false;
           
            velocidade = -gamepad1.left_stick_y;
            mecanum = gamepad1.left_stick_x;
            girar = gamepad1.right_stick_x;
            
            x = 1;
            y = 0.5;
            
            if(redução == true){
                powerMecanum(y);
                
            }else{
                powerMecanum(x);
            }
            
            if(SensorDistancia.getDistance(DistanceUnit.CM) < 50){
                powerMecanum(0.2);
            }
            if(SensorDistancia.getDistance(DistanceUnit.CM) < 5){
                powerMecanum(-0.05);
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
            double a = 0.3;
            double b = 0.4;
            
            if (gamepad2.a == true && Braço.getCurrentPosition() >= 0){//não deixa forçar no solo
                Braço.setPower(0.02);
                BraçoEsquerda.setPower(0.02);
                continue;
            }
            if (gamepad2.a == true && Braço.getCurrentPosition() >-200){//não deixa forçar no solo
                power4Rodas(-0.25);
            }
            if (Braço.getCurrentPosition() >-200){ //reduziu enquanto está mais baixo
                 redução = true;
            }
             if (Braço.getCurrentPosition() <-200 && Braço.getCurrentPosition() >-300 ){ //reduziu enquanto está mais baixo
                 redução = false;
             }
            if (gamepad2.a == true && Braço.getCurrentPosition() < -300){////positivo é para baixo 
                Braço.setPower(a);
                BraçoEsquerda.setPower(a);
            }
            if(gamepad2.a == true && Braço.getCurrentPosition() >= -300){
                Braço.setPower(-0.1);
                BraçoEsquerda.setPower(-0.1); 
                Braço.setPower(a*0.5);//50% da pot 
                BraçoEsquerda.setPower(a*0.5);//50% da pot
                if(Braço.getCurrentPosition() < -100) power4Rodas(-0.1); // ré apenas entre -400 e -100
            }
            
            if(gamepad2.right_trigger > 0.5){ // Deixa garra em -330
                if (Braço.getCurrentPosition() > -330 ){
                    Braço.setPower(-0.2);
                    BraçoEsquerda.setPower(-0.2);   
                }else{
                    Braço.setPower(0.2);
                    BraçoEsquerda.setPower(0.2); 
                }
                if (Braço.getCurrentPosition() < -330 && Braço.getCurrentPosition() > -400){
                    Braço.setPower(-0.02);
                    BraçoEsquerda.setPower(-0.02);
                }
            }
            
            if (gamepad2.b == true && Braço.getCurrentPosition() <= -1080){//não deixa forçar a possição da entrega
                Braço.setPower(0.015);
                BraçoEsquerda.setPower(0.015);
                continue;
            }
            if (gamepad2.b == true && Braço.getCurrentPosition() > -700 ){
                Braço.setPower(-b);
                BraçoEsquerda.setPower(-b);
            }
            else{
                if (gamepad2.b == true && Braço.getCurrentPosition() <= -700){
                    Braço.setPower(-b*0.5);
                    BraçoEsquerda.setPower(-b*0.5);
                }
            }
            
            if(gamepad2.b == false && gamepad2.a == false && gamepad2.right_trigger < 0.5){ //trava o motor na posição
                if (Braço.getCurrentPosition() > -900){
                    Braço.setPower(-0.015);
                    BraçoEsquerda.setPower(-0.015); 
                }else{
                    Braço.setPower(0.015);
                    BraçoEsquerda.setPower(0.015); 
                }
            }
           
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
           if (gamepad2.y == true){
                Aviao.setPosition (0.5);
            }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            if (gamepad2.x == true){ //abaixa a garra e reseta o encoder 
                Braço.setPower(0.4);
                BraçoEsquerda.setPower(0.4);
                power4Rodas(-0.2);
                Braço.setPower(0.1);
                BraçoEsquerda.setPower(0.1);
                Braço.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoder
                Braço.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////   
   //Seta potencias iguais(p) as 4 rodas do robo
    void power4Rodas(double p){
        DireitaFrente.setPower(p);
        EsquerdaFrente.setPower(p);
        DireitaTras.setPower(p);
        EsquerdaTras.setPower(p);
    }
    
    //Seta as rodas para reberem power em movimento mecanum vezes um limitador(l)
    void powerMecanum(double l){
        DireitaFrente.setPower((velocidade - mecanum - girar)*l);
        EsquerdaFrente.setPower((velocidade - mecanum + girar)*l);
        DireitaTras.setPower((velocidade + mecanum - girar)*l);
        EsquerdaTras.setPower((velocidade + mecanum + girar)*l);
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
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
