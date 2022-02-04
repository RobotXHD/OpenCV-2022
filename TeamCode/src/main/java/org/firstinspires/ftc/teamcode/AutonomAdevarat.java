package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Autonomous
public class AutonomAdevarat extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    DcMotorEx motorFR, motorFL, motorBR, motorBL;
    private TFObjectDetector tfod;
    int varrez,ok=0,pduck=0;
    private static final String VUFORIA_KEY="ATYhHtX/////AAABmYiQro9x40+lpw67He6N9ct0CV8iFEOYHpbJ8b4xWTZwlfuNQu2h1+PQ03af1oOQ+Wf5jInqtlVVUYnO4fT45P++kQk+pqOW6dYQu+X7hM0zl3kDZ1v1iwIMDm4t0JHmG7CRgSKRjHOx5QNdkuLN692ENXQwpdHfKWnrAX4Q354hV7bHkHZN5EBxrioAVF7cGdz22wU48q5C14kzQFuBXwzv9GLxLtwcLWzuJqOLTVl+jOGBgecbSVMfUGqv99hVoxwANgWBAJBYnBFNO2dOlCbR/vfWD1tngy/geO8ZGyzarUr/wvQp2q7kI1MCBJ6QN0/Seti2/ABk3fe/9ASG7pmJJOkenXQ3LtBJD0AB4epB";
    public double lastTime;
    OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia;
    private static final String cub = "Cube";
    String rezultat = "None";
    static final double COUNTSPERR = 383.6;
    static final double GEARREDUCTION = 1;
    static final double DIAMROT = 9.6;
    static final double COUNTS_PER_CM = (COUNTSPERR*GEARREDUCTION) / (DIAMROT*3.1415);
    private DcMotorEx DJL;
    private DcMotorEx arm;
    private Servo loader;
    private Servo grabber_left;
    private Servo grabber_right;

    private final double crThreshHigh = 150;
    private final double crThreshLow = 120;
    private final double cbThreshHigh = 255;
    private final double cbThreshLow = 255;
    int currentmotorBL;
    int currentmotorBR;
    int currentmotorFL;
    int currentmotorFR;
    double Lpos = 0.7;
    String rezultat1 = "None";
    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR");
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL");
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR");
        arm     = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        DJL     = (DcMotorEx) hardwareMap.dcMotor.get("DJL");
        grabber_left   = hardwareMap.servo.get("grabber_left");
        grabber_right  = hardwareMap.servo.get("grabber_right");
        loader         = hardwareMap.servo.get("loader");

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DJL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        loader.setPosition(1.0);
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0 / 9.0);
        }
        Detectam.start();

        if(!isStarted())

            waitForStart();

        if(rezultat == "None" && rezultat1 == "None") varrez=1;
        else if(rezultat == "Duck" && rezultat1 == "Marker")  varrez=2;
        else if(rezultat == "Marker" && rezultat1 == "Duck")  varrez=3;
        telemetry.addData("var",varrez );
        telemetry.update();

        if(varrez==1)
        {
            Translatare(-140, 30, 0.6);
            //Translatare(-45, 30, 0.3);
            lastTime = System.currentTimeMillis();
            while(lastTime + 200 > System.currentTimeMillis()){
            }

            DJL.setPower(0.4);
            lastTime = System.currentTimeMillis();
            while(lastTime + 2500 > System.currentTimeMillis()){
            }
            DJL.setPower(0.0);

            Translatare(265, 20, 0.7);
            Rotire(440, 0.8);

            arm.setTargetPosition(450);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);
            while(arm.isBusy());
            arm.setPower(0);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lastTime = System.currentTimeMillis();
            while(lastTime + 100 > System.currentTimeMillis()){
            }

            loader.setPosition(0.0);


            arm.setTargetPosition(0);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);
            while(arm.isBusy());
            arm.setPower(0);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Rotire(-650, 0.9);
            Translatare(100, 0, 0.5);
            Translatare(0, 250, 0.5);

            loader.setPosition(1.0);
        }

        if(varrez == 2)
        {

            Translatare(-140, 30, 0.6);
            //Translatare(-45, 30, 0.3);
            lastTime = System.currentTimeMillis();
            while(lastTime + 200 > System.currentTimeMillis()){
            }

            DJL.setPower(0.4);
            lastTime = System.currentTimeMillis();
            while(lastTime + 3000 > System.currentTimeMillis()){
            }
            DJL.setPower(0.0);

            Translatare(265, -10, 0.7);
            Rotire(440, 0.8);

            arm.setTargetPosition(515);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.8);
            while(arm.isBusy());
            arm.setPower(0);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lastTime = System.currentTimeMillis();
            while(lastTime + 500 > System.currentTimeMillis()){
            }

            Translatare(0, -10, 0.3);
            lastTime = System.currentTimeMillis();
            while(lastTime + 500 > System.currentTimeMillis()){}

            loader.setPosition(0.0);

            Translatare(0, 10, 0.3);


            arm.setTargetPosition(0);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);
            while(arm.isBusy());
            arm.setPower(0);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Rotire(-650, 0.9);
            Translatare(80, 0, 0.5);
            Translatare(0, 250, 0.5);

            loader.setPosition(1.0);

        }
        if(varrez == 3)
        {

            //pozitionare shoot 3
            Translatare(10, 57, 0.3);

            //sleep(50);
            Rotire(5, 0.3);
            Rotire(4, 0.3);

            //shoot 3 - 1
            //shuter.setVelocity(-2390);
            lastTime = System.currentTimeMillis();
            while(lastTime + 500 > System.currentTimeMillis()){

            }
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            //shuter.setVelocity(-2420);
            lastTime = System.currentTimeMillis();
            while(lastTime + 500 > System.currentTimeMillis()){

            }//300

            //shoot 3 - 2
            //shuter.setVelocity(-2380);
            lastTime = System.currentTimeMillis();
            while(lastTime + 10 > System.currentTimeMillis()){

            }
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            //shuter.setVelocity(-2390);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }

            //shoot 3 - 3
            //shuter.setVelocity(-2420);
            lastTime = System.currentTimeMillis();
            while(lastTime + 10 > System.currentTimeMillis()){

            }
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            //shuter.setVelocity(-2390);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }

            //shoot 3 - 4
            //shuter.setVelocity(-2400);
            lastTime = System.currentTimeMillis();
            while(lastTime + 10 > System.currentTimeMillis()){

            }
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            //shuter.setVelocity(-2390);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }

            //shuter.setVelocity(-2180);
            //Cnoc inele
            //intake.setPower(1.0);
            Translatare(-5, 0, 0.3);
            //Translatare(-5, -5, 0.5);
            Translatare(0, 35, 0.6);
            lastTime = System.currentTimeMillis();
            while(lastTime + 100 > System.currentTimeMillis()) {
            }
            Translatare(0, 25, 0.2);
            lastTime = System.currentTimeMillis();
            while(lastTime + 50 > System.currentTimeMillis()){
            }
            //intake.setPower(1.0);



            Translatare(0, -42, 0.35);// X = -18
            lastTime = System.currentTimeMillis();
            while(lastTime + 50 > System.currentTimeMillis()){

            }
            //sleep(50);

            Translatare(-10, 47, 0.35);// X = 40
            lastTime = System.currentTimeMillis();
            while(lastTime + 50 > System.currentTimeMillis()){

            } //sleep


            Rotire(7, 0.3);

            //shoot 3 - 4
            //shuter.setVelocity(-2215);
            lastTime = System.currentTimeMillis();
            while(lastTime + 10 > System.currentTimeMillis()){

            }
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            //shuter.setVelocity(-2205);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }

            Translatare(0, 48, 0.3);
            lastTime = System.currentTimeMillis();
            while(lastTime + 50 > System.currentTimeMillis()){

            }

            //shuter.setVelocity(-2100);
            lastTime = System.currentTimeMillis();
            while(lastTime + 10 > System.currentTimeMillis()){

            }
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            //shuter.setVelocity(-2080);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }

            //shuter.setVelocity(-2100);
            lastTime = System.currentTimeMillis();
            while(lastTime + 10 > System.currentTimeMillis()){

            }
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            //shuter.setVelocity(-2080);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }


            lastTime = System.currentTimeMillis();
            while(lastTime + 10 > System.currentTimeMillis()){

            }
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            //shuter.setVelocity(-2085);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }

            //shuter.setVelocity(-2105);
            lastTime = System.currentTimeMillis();
            while(lastTime + 10 > System.currentTimeMillis()){

            }
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            //shuter.setVelocity(-2085);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }

            //shuter.setVelocity(-2105);
            lastTime = System.currentTimeMillis();
            while(lastTime + 10 > System.currentTimeMillis()){

            }
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            //shuter.setVelocity(-2085);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }



            //grabber_right.setPosition(0.9);


            Translatare(70, 140, 0.3);
            lastTime = System.currentTimeMillis();
            while(lastTime + 50 > System.currentTimeMillis()){

            }

            //intake.setPower(0);

            arm.setTargetPosition(-400);//-507 original
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1.0);
            while(arm.isBusy())
                lastTime = System.currentTimeMillis();
            while(lastTime + 5 > System.currentTimeMillis()){

            }

            arm.setPower(0);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lastTime = System.currentTimeMillis();
            while(lastTime + 50 > System.currentTimeMillis()){

            }


            lastTime = System.currentTimeMillis();
            while(lastTime + 50 > System.currentTimeMillis()){

            }


            Translatare(-70, -110, 0.5);
            lastTime = System.currentTimeMillis();
            while(lastTime + 50 > System.currentTimeMillis()){

            }

            arm.setTargetPosition(-100);//-321; -400
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1.0);
            while(arm.isBusy());
            arm.setPower(0);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public Thread Detectam = new Thread(new Runnable()
    {
        @Override
        public void run()
        {
            if(isStarted())
            {
                while (isStarted())
                {
                    if (tfod != null)
                    {
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null)
                        {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            if (updatedRecognitions.size()==0)
                                rezultat = "None";
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions)
                            {
                                telemetry.addData(String.format("label (%d)",i),recognition.getLabel());
                                rezultat = recognition.getLabel();
                                telemetry.addData(String.format(" left,top (%d)",i),"%.03f,%.03f", recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format(" right,bottom (%d)",i),"%.03f, %.03f", recognition.getRight(), recognition.getBottom());
                            }
                            telemetry.addData("var",rezultat );
                            telemetry.addData("pozitiecub",pduck);
                            telemetry.update();
                        }
                    }
                }
            }
            if(tfod != null)
            {
                tfod.shutdown();
            }
        }
    });
    public void Translatare(int deltaX, int deltaY, double speed)
    {
        boolean Done = false;
        int errorpos ;
        int Maxerror = 20;
        int targetBL, targetBR, targetFL, targetFR;
        double cpcm = COUNTS_PER_CM * 0.707 ;

        currentmotorBL = motorBL.getCurrentPosition();
        currentmotorBR = motorBR.getCurrentPosition();
        currentmotorFL = motorFL.getCurrentPosition();
        currentmotorFR = motorFR.getCurrentPosition();

        targetBR = currentmotorBR + (int) (( deltaY + deltaX) * cpcm);
        targetBL = currentmotorBL + (int) ((-deltaY + deltaX) * cpcm);
        targetFR = currentmotorFR + (int) (( deltaY - deltaX) * cpcm);
        targetFL = currentmotorFL + (int) ((-deltaY - deltaX) * cpcm);


         /*
         motorBR.setTargetPosition(currentmotorBR + (int) (( deltaY + deltaX) * cpcm));
         motorBL.setTargetPosition(currentmotorBL + (int) ((-deltaY + deltaX) * cpcm));
         motorFR.setTargetPosition(currentmotorFR + (int) (( deltaY - deltaX) * cpcm));
         motorFL.setTargetPosition(currentmotorFL + (int) ((-deltaY - deltaX) * cpcm));
         */
        motorBL.setTargetPosition(targetBL);
        motorBR.setTargetPosition(targetBR);
        motorFL.setTargetPosition(targetFL);
        motorFR.setTargetPosition(targetFR);

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBL.setPower(speed);
        motorBR.setPower(speed);
        motorFL.setPower(speed);
        motorFR.setPower(speed);

        Done = false;
        while(!Done && opModeIsActive()){
            Done = true;
            errorpos = Math.abs(targetBL - motorBL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetBR - motorBR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFL - motorFL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFR - motorFR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;
        }

        //while(motorFR.isBusy() || motorFL.isBusy() || motorBR.isBusy() || motorBL.isBusy() && opModeIsActive());

        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);

        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void Rotire (int deltaA, double speed)
    {
        boolean Done = false;
        int errorpos ;
        int Maxerror = 15;
        int targetBL, targetBR, targetFL, targetFR;
        double cpdeg = 17.5 * 3.141 / 180 * COUNTS_PER_CM;

        currentmotorBL = motorBL.getCurrentPosition();
        currentmotorBR = motorBR.getCurrentPosition();
        currentmotorFL = motorFL.getCurrentPosition();
        currentmotorFR = motorFR.getCurrentPosition();

        targetBL = currentmotorBL + (int) (deltaA * cpdeg);
        targetBR = currentmotorBR + (int) (deltaA * cpdeg);
        targetFL = currentmotorFL + (int) (deltaA * cpdeg);
        targetFR = currentmotorFR + (int) (deltaA * cpdeg);

        motorBL.setTargetPosition(targetBL);
        motorBR.setTargetPosition(targetBR);
        motorFL.setTargetPosition(targetFL);
        motorFR.setTargetPosition(targetFR);

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBL.setPower(speed);
        motorBR.setPower(speed);
        motorFL.setPower(speed);
        motorFR.setPower(speed);

        Done = false;
        while(!Done && opModeIsActive()){
            Done = true;
            errorpos = Math.abs(targetBL - motorBL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetBR - motorBR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFL - motorFL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFR - motorFR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;
        }
    }
    private void initVuforia()
    {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.setFrameQueueCapacity(3);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    /*              |
                    |
                    |
                    |
                    |
       _____________|_______________
                    |
                    |
                    |
                    |
                    |
                    |
                    |
                    |
                    |
                    |
                    |

     */
}
