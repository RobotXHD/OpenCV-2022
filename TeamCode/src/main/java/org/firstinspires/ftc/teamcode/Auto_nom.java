package org.firstinspires.ftc.teamcode;

import android.graphics.Rect;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.opencv.core.Scalar;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous
public class Auto_nom extends LinearOpMode {
    DcMotorEx motorFR, motorFL, motorBR, motorBL;
    private TFObjectDetector tfod;
    int varrez,ok=0,pduck=0;
    public double lastTime;

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

    private double crThreshHigh = 150;
    private double crThreshLow = 120;
    private double cbThreshHigh = 255;
    private double cbThreshLow = 255;
    int currentmotorBL;
    int currentmotorBR;
    int currentmotorFL;
    int currentmotorFR;
    double Lpos = 0.7;
    private OpenCvCamera webcam;
    private ContourPipeline pipeline;

    private final int minRectangleArea = 2000;
    private final double leftBarcodeRangeBoundary = 0.3; //i.e 30% of the way across the frame from the left
    private final double rightBarcodeRangeBoundary = 0.6; //i.e 60% of the way across the frame from the left

    private double lowerRuntime = 0;
    private double upperRuntime = 0;

    // Pink Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 150.0, 120.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR");
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL");
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR");
        arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        DJL = (DcMotorEx) hardwareMap.dcMotor.get("DJL");
        grabber_left = hardwareMap.servo.get("grabber_left");
        grabber_right = hardwareMap.servo.get("grabber_right");
        loader = hardwareMap.servo.get("loader");

        pipeline = new ContourPipeline(0.2, 0.2, 0.2, 0.2);

        pipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        pipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);

        webcam.setPipeline(pipeline);

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
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        Culoare.start();
        if(!isStarted())
            waitForStart();
        if(isStopRequested()) return;
        if(varrez==1){
            Translatare(-140, 30, 0.6);
            //Translatare(-45, 30, 0.3);
            wait(200);

            DJL.setPower(0.4);
            lastTime = System.currentTimeMillis();
            wait(2500);
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
    public void testing(ContourPipeline pipeline){
        if(lowerRuntime + 0.05 < getRuntime()){
            crThreshLow += -gamepad1.left_stick_y;
            cbThreshLow += gamepad1.left_stick_x;
            lowerRuntime = getRuntime();
        }
        if(upperRuntime + 0.05 < getRuntime()){
            crThreshHigh += -gamepad1.right_stick_y;
            cbThreshHigh += gamepad1.right_stick_x;
            upperRuntime = getRuntime();
        }

        crThreshLow = inValues(crThreshLow, 0, 255);
        crThreshHigh = inValues(crThreshHigh, 0, 255);
        cbThreshLow = inValues(cbThreshLow, 0, 255);
        cbThreshHigh = inValues(cbThreshHigh, 0, 255);

        pipeline.configureScalarLower(0.0, crThreshLow, cbThreshLow);
        pipeline.configureScalarUpper(255.0, crThreshHigh, cbThreshHigh);

        telemetry.addData("lowerCr ", crThreshLow);
        telemetry.addData("lowerCb ", cbThreshLow);
        telemetry.addData("UpperCr ", crThreshHigh);
        telemetry.addData("UpperCb ", cbThreshHigh);
    }
    public double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }
    public Thread Culoare = new Thread(new Runnable()
    {
        @Override
        public void run()
        {
            if(isStarted())
            {
                while (isStarted())
                {
                    if(pipeline.error){
                        telemetry.addData("Exception: ", pipeline.debug.getStackTrace());
                    }
                    // Only use this line of the code when you want to find the lower and upper values, using Ftc Dashboard (https://acmerobotics.github.io/ftc-dashboard/gettingstarted)
                    // testing(pipeline);

                    // Watch our YouTube Tutorial for the better explanation

                    double rectangleArea = pipeline.getRectArea();

                    //Print out the area of the rectangle that is found.
                    telemetry.addData("Rectangle Area", rectangleArea);

                    //Check to see if the rectangle has a large enough area to be a marker.
                    if(rectangleArea > minRectangleArea){
                        //Then check the location of the rectangle to see which barcode it is in.
                        if(pipeline.getRectMidpointX() > rightBarcodeRangeBoundary * pipeline.getRectWidth()){
                            telemetry.addData("Barcode Position", "Right");
                            varrez=3;
                        }
                        else if(pipeline.getRectMidpointX() < leftBarcodeRangeBoundary * pipeline.getRectWidth()){
                            telemetry.addData("Barcode Position", "Left");
                            varrez=1;
                        }
                        else {
                            telemetry.addData("Barcode Position", "Center");
                            varrez=2;
                        }
                    }

                    telemetry.update();
                }
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
    public void wait(int t){
        lastTime=System.currentTimeMillis();
        while(lastTime+t<System.currentTimeMillis()){

        }
    }
}
/*              |
                |
                |
                |
                |
________________|________________
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
                |

 */
