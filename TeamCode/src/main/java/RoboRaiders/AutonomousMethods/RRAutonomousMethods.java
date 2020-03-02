package RoboRaiders.AutonomousMethods;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import RoboRaiders.Autonomous.RoboRaidersPipeline;
import RoboRaiders.Autonomous.RoboRaidersPipelineWebcam;
import RoboRaiders.Logger.Logger;
import RoboRaiders.Robot.Robot;

//import org.openftc.easyopencv.OpenCvWebcam;

public abstract class RRAutonomousMethods extends LinearOpMode {

    public double motor_power;
    public double degreesToTurn;
    public double currentHeading;
    public double finalHeading;
    public double integratedZAxis;
    public double iza_lastHeading = 0.0;
    public double iza_deltaHeading;
    public float iza_newHeading;
    public Orientation iza_angles;
    private RoboRaidersPID rrPID = new RoboRaidersPID(0.012,0,0.029);
    private RoboRaidersPID rrPID180 = new RoboRaidersPID(0.008, 0, 0);

    private static final double STRAFING_SCALE = 0.1;

    private int stoneLocation = 999;


    public void liftMotorRTPDriveWithStone(Robot robot) {
        robot.resetLiftEncoder();
        robot.runLiftWithEncoderRTP();
        robot.setLiftMotorTargetPosition(18); //18 encoders is equal to 1/8 inch up
        robot.setLiftMotorPower(.3);
    }

    public void liftMotorRTPDriveWithoutStone(Robot robot, double liftPower, int position) {
        robot.resetLiftEncoder();
        robot.runLiftWithEncoderRTP();
        robot.setLiftMotorTargetPosition(-18); //-18 encoders is equal to 1/8 inch down
        robot.setLiftMotorPower(.3);
    }

    public void encodersMoveRTP(Robot robot, double distance, double power, String direction){
        robot.resetEncoders();
        robot.runWithEncodersSTP();

        final double v = robot.driveTrainCalculateCounts(distance);
        double COUNTS = v;

        if (direction.equals("forward")) {
            robot.setDTMotorTargetPosition((int)COUNTS);
            robot.setDriveMotorPower(power, power, power, power);

            while ((double)robot.getSortedEncoderCount() < COUNTS && opModeIsActive()){
                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                telemetry.update();
            }
            robot.setDriveMotorPower(0, 0, 0, 0);
        }

        if (direction.equals("backward")) {
            robot.setDTMotorTargetPosition(-(int)COUNTS);
            robot.setDriveMotorPower(-power, -power, -power, -power);

            while (-(double)robot.getSortedEncoderCount() > -COUNTS && opModeIsActive()){
                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                telemetry.update();
            }
            robot.setDriveMotorPower(0, 0, 0, 0);
        }
        robot.resetEncoders();
        robot.runWithoutEncoders();
    }

    public void encodersMove(Robot robot, double distance, double power, String direction) { //sets the parameters

        robot.resetEncoders(); //resets encoders
        robot.runWithEncoders(); //sets the mode back to run with encoder

        final double v = robot.driveTrainCalculateCounts(distance);
        double COUNTS = v; //COUNTS is now equal to the value calculated

        if (direction.equals("forward")) { //if the desired direction is forward

            robot.setDriveMotorPower(power, power, power, power); //start driving forward

            while ((double)robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still less than the desired count and the opMode has not been stopped

                //telemetry.addData("COUNTS", COUNTS);
                //telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                //telemetry.update();
            }

            robot.setDriveMotorPower(0, 0, 0, 0); //stop the robot
        } else if (direction.equals("backward")) { //if the desired direction is backward

            robot.setDriveMotorPower(-power, -power, -power, -power); //start driving backward

            while ((double)robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still greater than the desired count and the opMode has not been stopped

                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                telemetry.update();
            }
            robot.setDriveMotorPower(0,0,0,0);
        }
    }

    public void encodersMoveStrafe(Robot robot, double distance, double power, String direction) {
        robot.resetEncoders();
        robot.runWithEncoders();

        final double v = robot.driveTrainCalculateCounts(distance);
        double COUNTS = v; //COUNTS is now equal to the value calculated

        if (direction.equals("right")) { //if the desired direction is right

            robot.setDriveMotorPower(power, -power, -power, power); //start strafing right

            while (robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still less than the desired count and the opMode has not been stopped

                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                telemetry.update();
            }

            robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stop the robot
        } else if (direction.equals("left")) { //if the desired direction is left

            robot.setDriveMotorPower(-power, power, power, -power); //start strafing left

            while (robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still greater than the desired count and the opMode has not been stopped

                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                telemetry.update();
            }

            robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stop the robot
        }

        // smksmksmk need to reset the encoders
        // smksmksmk need to reset the encoders
        // smksmksmk need to reset the encoders
        // smksmksmk need to reset the encoders

        robot.runWithoutEncoders(); //sets the mode back to run without encoder
    }

    public void runIntake(Robot robot, double power) {
        robot.setIntakePower(power);
    }

    /*public void intakeArmAuto (Robot robot, double position) {
      robot.intakeArmAuto(position);
    }*/

    public void stoneSampleServo (Robot robot) {
       // intakeArmAuto(robot, 0.0);
        robotSleep(1000);
        encodersMove(robot, 15, .3, "backward");
        robotSleep(1000);
        encodersMove(robot, 2, .3, "forward");
        robotSleep(1000);
        //runIntake(robot, -.5);
    }


    public void collectStone(Robot robot){
        double startIntakeTime = System.currentTimeMillis();
        runIntake(robot, -.6);
        encodersMove(robot, 28, .15, "forward");
        while (opModeIsActive() && System.currentTimeMillis() - startIntakeTime < 4000){
        }
        runIntake(robot, 0);
    }

    public void robotSleep(int timeToSleep) {
        try {
            Thread.sleep(timeToSleep);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void imuTurn(Robot robot, float degreesToTurn, double power, String direction) { //gets hardware from
        //Robot and defines degrees as a
        //float, power as a double, and direction as a string
        //robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //telemetry.addLine().addData("degreesToTurn",String.valueOf(degreesToTurn));
        currentHeading = robot.getIntegratedZAxis();
        //finalHeading = currentHeading + degreesToTurn;
        //telemetry.update();

        // robot.getHeading(); returns the current heading of the IMU

        if (direction.equals("left")) { //if the desired direction is left
            finalHeading = currentHeading + degreesToTurn;
            robot.setDriveMotorPower(-power, power, -power, power); //the robot will turn right
            while(opModeIsActive() && robot.getIntegratedZAxis() < finalHeading) {
                //robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //currentHeading = robot.getIntegratedZAxis();
                telemetry.addLine().addData("getHeading",String.valueOf(currentHeading));
                telemetry.addLine().addData("finalheading",String.valueOf(finalHeading));
                telemetry.addLine().addData("IntZ",String.valueOf(robot.integratedZAxis));
                telemetry.update();

            }
        }
        else { //if the desired direction is right
            finalHeading = currentHeading - degreesToTurn;
            robot.setDriveMotorPower(power, -power, power, -power); //the robot will turn right
            while(opModeIsActive() && robot.getIntegratedZAxis() > finalHeading) {
                //robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //currentHeading = robot.getIntegratedZAxis();
                telemetry.addLine().addData("getHeading",String.valueOf(currentHeading));
                telemetry.addLine().addData("finalheading",String.valueOf(finalHeading));
                telemetry.addLine().addData("IntZ",String.valueOf(robot.integratedZAxis));
                telemetry.update();
            }
        }
        robot.setDriveMotorPower(0,0,0,0);
    }

    /**
     * turning with PID
     * @param robot
     * @param rrPID
     * @param degreesToTurn
     * @param direction
     */
    public void imuTurnPID(RoboRaiders.AutonomousMethods.RoboRaidersPID rrPID, Robot robot, float degreesToTurn, String direction) {

        double power = 0.0;
        int loopcount = 0;
        int maxLoopCount = 0;


        // Normally the motor powers for the left side are set to reverse (this allows the motors
        // to turn in the same direction.  For turning however, set the motors on the left side of
        // the robot to turn reverse (prior was forward), which will turn the left motors in the opposite direction
        // of the right motors, thus turning the robot.
        robot.motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.motorBackLeft.setDirection(DcMotor.Direction.REVERSE);


        Logger L = new Logger("imuTurnPID");
        rrPID.initialize();
        telemetry.addLine().addData("in", "imuTurnPID");

        currentHeading = 0.0;
        currentHeading = robot.getIntegratedZAxis();


        L.Debug("Start");
        L.Debug("currentHeading: ", currentHeading);
        L.Debug("degreesToTurn", degreesToTurn);

        // robot.getHeading(); returns the current heading of the IMU

        // When turning and reading the IMU...clockwise angles generally decrease, that is they
        // get smaller.  Whereas, counter clockwise angles generally increase, that is they
        // get larger.  Thus turning right requires the number of degrees to turn to be decremented
        // from the current heading.  For turning left, the number of degrees to turn is incremented
        // to the current heading.

        if (degreesToTurn <= 90){maxLoopCount = 20;}
        else {maxLoopCount = 50;}

        if (direction.equals("right")) { //if the desired direction is right
            finalHeading = currentHeading - degreesToTurn;
            telemetry.addLine().addData("currentHeading", currentHeading);
            telemetry.addLine().addData("finalHeading", finalHeading);

            L.Debug("Turning Right");
            L.Debug("finalHeading: ",finalHeading);

            // power = rrPID.CalculatePIDPowers(finalHeading,currentHeading);
            telemetry.addLine().addData("power", power);

            L.Debug("Calculated PID Power (power): ",power);

            //  robot.setDriveMotorPower(power, power, power, power); //the robot will turn right


            // The robot will turn within plus or minus 3.5 degrees and needs to complete the turn
            // in 20 iterations or less.  During testing it was found that the robot would still be
            // applying small amounts of power in an attempt to get the robot to turn the last
            // few degrees, however, the small amounts of power wasn't quite enough to over come the
            // effects of friction on the robot

            while((opModeIsActive() && (loopcount < maxLoopCount &&
                    !(currentHeading < finalHeading + 2.5 && currentHeading > finalHeading - 2.5)))){
                //&& Math.abs(power) > 0.1) {
                currentHeading = robot.getIntegratedZAxis();
                power = rrPID.CalculatePIDPowers(finalHeading,currentHeading) * 0.75;

                loopcount++;

                L.Debug("In While Loop");
                L.Debug("finalHeading: ",finalHeading);
                L.Debug("currentHeading: ",currentHeading);
                L.Debug("Remaining Degrees: ",finalHeading - currentHeading);
                L.Debug("Calculated PID Power (power): ",power);
                L.Debug("loopcount", loopcount);


                robot.setDriveMotorPower(power, power, power, power);

                //robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //currentHeading = robot.getIntegratedZAxis();
                telemetry.addLine().addData("right", "right");
                telemetry.addLine().addData("power", String.valueOf(power));
                telemetry.addLine().addData("IntZ",String.valueOf(robot.getIntegratedZAxis()));
                telemetry.addLine().addData("finalHeading",String.valueOf(finalHeading));
                telemetry.addLine().addData("difference",String.valueOf(finalHeading - robot.getIntegratedZAxis()));
                telemetry.update();

            }
            robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stops robot
            L.Debug("Out of While Loop");
            L.Debug("finalHeading: ",finalHeading);
            L.Debug("currentHeading: ",robot.getIntegratedZAxis());
            L.Debug("Remaining Degrees: ",finalHeading - robot.getIntegratedZAxis());



        }
        else { //if the desired direction is left
            finalHeading = currentHeading + degreesToTurn;

            L.Debug("Turning Left");
            L.Debug("finalHeading: ",finalHeading);
            robot.motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
            robot.motorBackRight.setDirection(DcMotor.Direction.REVERSE);


            // The robot will turn within plus or minus 3.5 degrees and needs to complete the turn
            // in 20 iterations or less.  During testing it was found that the robot would still be
            // applying small amounts of power in an attempt to get the robot to turn the last
            // few degrees, however, the small amounts of power wasn't quite enough to over come the
            // effects of friction on the robot

            while((opModeIsActive() && (loopcount < maxLoopCount &&
                    !(currentHeading > finalHeading - 2.5 && currentHeading < finalHeading + 2.5)))){
                //&& Math.abs(power) > 0.1) {
                currentHeading = robot.getIntegratedZAxis();
                power = rrPID.CalculatePIDPowers(finalHeading,currentHeading) * 0.75;

                loopcount++;

                L.Debug("In While Loop");
                L.Debug("finalHeading: ",finalHeading);
                L.Debug("currentHeading: ",currentHeading);
                L.Debug("Remaining Degrees: ",finalHeading - currentHeading);
                L.Debug("Calculated PID Power (power): ",power);
                L.Debug("loopcount", loopcount);


                robot.setDriveMotorPower(power, power, power, power);


                //robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //currentHeading = robot.getIntegratedZAxis();
                telemetry.addLine().addData("left", "left");
                //telemetry.addLine().addData("getHeading",String.valueOf(currentHeading));
                telemetry.addLine().addData("IntZ",String.valueOf(robot.integratedZAxis));
                telemetry.addLine().addData("finalHeading",String.valueOf(finalHeading));
                telemetry.addLine().addData("difference",String.valueOf(finalHeading - robot.getIntegratedZAxis()));
                telemetry.update();
            }
        }


        robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stops robot


        // Reverse the motor direction on the left motors so that all motors spin in the same
        // direction
        robot.motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.motorBackLeft.setDirection(DcMotor.Direction.FORWARD);


        L.Debug("End");
    }

    public int stoneDetection(){
//        OpenCvCamera phone_camera;
        OpenCvInternalCamera phone_camera;
        RoboRaidersPipeline stone_pipeline;
        int pattern = 999;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

//        phone_camera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phone_camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phone_camera.openCameraDevice();
        stone_pipeline = new RoboRaidersPipeline(pattern);
        phone_camera.setPipeline(stone_pipeline);

        phone_camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        while (opModeIsActive() && stone_pipeline.getPattern() == 999) {
            telemetry.addData("FRAME", phone_camera.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phone_camera.getFps()));
            telemetry.addData("TFT MS", phone_camera.getTotalFrameTimeMs());
            telemetry.addData("PT MS", phone_camera.getPipelineTimeMs());
            telemetry.addData("OT MS", phone_camera.getOverheadTimeMs());
            telemetry.addData("MAX FPS", phone_camera.getCurrentPipelineMaxFps());
            telemetry.addData("PATTERN", stone_pipeline.getPattern());
            telemetry.update();
        }
        phone_camera.stopStreaming();
        telemetry.addData("PATTERN", stone_pipeline.getPattern());
        telemetry.update();
        return stone_pipeline.getPattern();

    }



    public int stoneDetectionWebcam(float[] leftRec, float[] rightRec){

        OpenCvCamera webcam;
        RoboRaidersPipelineWebcam stone_pipeline;
        int pattern = 999;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDevice();

        stone_pipeline = new RoboRaidersPipelineWebcam(pattern, leftRec, rightRec);

        webcam.setPipeline(stone_pipeline);

        webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);

        while (opModeIsActive() && stone_pipeline.getPattern() == 999) {
            telemetry.addData("FRAME", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("TFT MS", webcam.getTotalFrameTimeMs());
            telemetry.addData("PT MS", webcam.getPipelineTimeMs());
            telemetry.addData("OT MS", webcam.getOverheadTimeMs());
            telemetry.addData("MAX FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("PATTERN", stone_pipeline.getPattern());
            telemetry.update();
        }
        webcam.stopStreaming();
        telemetry.addData("PATTERN", stone_pipeline.getPattern());
        telemetry.addData("left brightness", stone_pipeline.getLeft_br());
        telemetry.addData("right brightness", stone_pipeline.getRight_br());
        telemetry.update();
        return stone_pipeline.getPattern();

    }

    public void stoneSamplingRed(Robot robot){ //NOTE THE PATTERNS MAY OR MAY NOT BE SCREWED UP!!!
        int stoneLocation = stoneDetection();

        switch (stoneLocation){
            case 1: //stone is on leftmost (not if the frame)
                leftStoneRed(robot);
                break;
            case 3: //stone is on the left (middle)
                rightStoneRed(robot);
                break;
            case 2: //stone is on the right
                middleStoneRed(robot);
                break;
            case 999:
                middleStoneRed(robot);
                break;
        }
    }



    public void stoneSamplingWebcamRed(Robot robot){ //NOTE THE PATTERNS MAY OR MAY NOT BE SCREWED UP!!!

        float leftRec[]  = {9f, 5f, 15f, 15f};
        float rightRec[] = {9f, 15f, 15f, 25f};
        int stoneLocation = stoneDetectionWebcam(leftRec, rightRec);

        // Gets first sky stone in the quarry.  The first skystone is furtherest away from the
        // field perimeter
        switch (stoneLocation){
            case 1: //stone is on leftmost (not if the frame)
                leftStoneRed(robot);
                imuTurnPID(rrPID180, robot, 176, "left");
                runIntake(robot, 1.0);
                encodersMoveRTP(robot, 15, .8, "backward");
                break;
            case 3: //stone is on the left (middle)
                rightStoneRed(robot);
                imuTurnPID(rrPID180, robot, 176, "left");
                runIntake(robot, 1.0);
                encodersMoveRTP(robot, 15, .8, "backward");
                break;
            case 2: //stone is on the right
                middleStoneRed(robot);
                imuTurnPID(rrPID180, robot, 176, "left");
                runIntake(robot, 1.0);
                encodersMoveRTP(robot, 15, .8, "backward");
                break;
            case 999:
                middleStoneRed(robot);
                imuTurnPID(rrPID180, robot, 176, "left");
                runIntake(robot, 1.0);
                encodersMoveRTP(robot, 15, .8, "backward");
                break;
        }
        switch (stoneLocation){
            case 1: //stone is on leftmost (not if the frame)
                secondLeftSkyStoneRed(robot);
                imuTurnPID(rrPID180, robot, 176, "left");
                runIntake(robot, 1.0);
                encodersMoveRTP(robot, 15, .8, "backward");
                break;
            case 3: //stone is on the left (middle)
                secondRightSkyStoneRed(robot);
                imuTurnPID(rrPID180, robot, 180, "left");
                runIntake(robot, 1.0);
                encodersMoveRTP(robot, 15, .8, "backward");
                break;
            case 2: //stone is on the right
                secondMiddleSkyStoneRed(robot);
                imuTurnPID(rrPID180, robot, 176, "left");
                runIntake(robot, 1.0);
                encodersMoveRTP(robot, 15, .8, "backward");
                break;
            case 999:
                secondMiddleSkyStoneRed(robot);
                imuTurnPID(rrPID180, robot, 176, "left");
                runIntake(robot, 1.0);
                encodersMoveRTP(robot, 15, .8, "backward");
                break;
        }
//
        runIntake(robot, 0.0);
        encodersMoveStrafe(robot, 10, .8, "left");

        /**
         * at this point the robot has deposited the first stone on the foundation!
         * reset the robot's lift and swing arm and have it strafe back into position
         * for the return trip
         */


    }

    public void setStoneLocation(int stoneLocation){
        this.stoneLocation = stoneLocation;
    }

    public int getStoneLocation(){
        return this.stoneLocation;
    }

    public void ejectStone(Robot robot){
//        imuTurnPID(rrPID, robot, 90, "left");
//        //robot.setCaptureServoUp();
//        encodersMoveRTP(robot, 24, .8, "forward");
//        runIntake(robot, 1.0);
//        double startTouchTime = System.currentTimeMillis();
//        while (robot.getStoneDistance() <= 1.1 && System.currentTimeMillis()-startTouchTime < 1500){}
//        encodersMoveRTP(robot, 24, .4, "backward");
//        runIntake(robot, 0.0);
//        imuTurnPID(rrPID, robot, 85, "right");

        imuTurnPID(rrPID180, robot, 178, "left");
        runIntake(robot, 1.0);
//        double startTouchTime = System.currentTimeMillis();
//        while (robot.getStoneDistance() <= 1.1 && System.currentTimeMillis()-startTouchTime < 1500){}
//        runIntake(robot, 0.0);

    }

    public void stoneSamplingWebcamBlue(Robot robot){ //NOTE THE PATTERNS MAY OR MAY NOT BE SCREWED UP!!!

        float leftRec[]  = {10f, 3f, 15f, 11f};
        float rightRec[] = {10f, 13f, 15f, 22f};
        int stoneLocation = stoneDetectionWebcam(leftRec, rightRec);
        setStoneLocation(stoneLocation);


        // Gets first sky stone in the quarry.  The first skystone is furtherest away from the
        // field perimeter
        switch (stoneLocation){
            case 1: //stone is on leftmost (not if the frame)
                leftStoneBlue(robot);
                imuTurnPID(rrPID180, robot, 176, "left");
                runIntake(robot, 1.0);
                break;
            case 3: //stone is on the left (middle)
                rightStoneBlue(robot);
                imuTurnPID(rrPID180, robot, 176, "left");
                runIntake(robot, 1.0);
                break;
            case 2: //stone is on the right
                middleStoneBlue(robot);
                imuTurnPID(rrPID180, robot, 172, "left");
                runIntake(robot, 1.0);
                break;
            case 999:
                middleStoneBlue(robot);
                imuTurnPID(rrPID180, robot, 172, "left");
                runIntake(robot, 1.0);
                break;
        }

       // ejectStone(robot);

        /**
         * at this point the robot has deposited the first stone on the foundation!
         * reset the robot's lift and swing arm and have it strafe back into position
         * for the return trip
         */





        // Could add an option here that says, if we don't want to do the second skystone
        // then just go and park
        // Get second sky stone in quarry.  The second skystone is closest to the field
        // perimeter
        switch (stoneLocation){
            case 1: //stone is on leftmost (not if the frame)
                secondLeftSkyStoneBlue(robot);
                imuTurnPID(rrPID180, robot, 176, "left");
                runIntake(robot, 1.0);
                encodersMoveRTP(robot, 15, .8, "backward");
                break;
            case 3: //stone is on the left (middle)
                secondRightSkyStoneBlue(robot);
                imuTurnPID(rrPID180, robot, 180, "left");
                runIntake(robot, 1.0);
                encodersMoveRTP(robot, 15, .8, "backward");
                break;
            case 2: //stone is on the right
                secondMiddleSkyStoneBlue(robot);
                imuTurnPID(rrPID180, robot, 176, "left");
                runIntake(robot, 1.0);
                encodersMoveRTP(robot, 15, .8, "backward");
                break;
            case 999:
               secondMiddleSkyStoneBlue(robot);
                imuTurnPID(rrPID180, robot, 176, "left");
                runIntake(robot, 1.0);
                encodersMoveRTP(robot, 15, .8, "backward");
                break;
        }

        runIntake(robot, 0.0);
        encodersMoveStrafe(robot, 10, .8, "right");
    }

//    public void getSecondStoneBlue(Robot robot){
//        int stoneLocation = getStoneLocation();
//        telemetry.addLine().addData("stoneLocation",stoneLocation);
//        telemetry.update();
//        switch (stoneLocation){
//            case 1:
//                telemetry.addLine().addData("stone location",stoneLocation);
//                left2ndSkyStoneBlue(robot);
//                break;
//            case 2:
//                break;
//            case 3:
//                break;
//            case 999:
//                break;
//
//        }
//    }


    public void secondMiddleSkyStoneBlue(Robot robot){
        encodersMoveRTP(robot, 58, .8, "backward");
        runIntake(robot, 0.0);
        imuTurnPID(rrPID180, robot, 177, "left");
        encodersMoveStrafe(robot, 15, 0.7, "left");//D = 19, P = .5
        runIntake(robot, -1.0);
        encodersMoveRTP(robot, 10, 0.3, "forward");
        double startTouchTime = System.currentTimeMillis();
        while (robot.getStoneDistance() <= 1.1 && System.currentTimeMillis()-startTouchTime < 1500) {} //keep going as long as it is under 1.5 seconds and the distance sensor is under 1 cm
        runIntake(robot, 0.0);
        //robot.setCaptureServoDown();
        //robotSleep(500);
        //liftMotorRTPDriveWithStone(robot);
        encodersMoveStrafe(robot, 13, .8, "right");
        encodersMoveRTP(robot, 62, .8, "backward");
    }

    public void secondMiddleSkyStoneRed(Robot robot){
        encodersMoveRTP(robot, 58, .8, "backward");
        runIntake(robot, 0.0);
        imuTurnPID(rrPID180, robot, 177, "right");
        encodersMoveStrafe(robot, 15, 0.7, "right");//D = 19, P = .5
        runIntake(robot, -1.0);
        encodersMoveRTP(robot, 10, 0.3, "forward");
        double startTouchTime = System.currentTimeMillis();
        while (robot.getStoneDistance() <= 1.1 && System.currentTimeMillis()-startTouchTime < 1500) {} //keep going as long as it is under 1.5 seconds and the distance sensor is under 1 cm
        runIntake(robot, 0.0);
        //robot.setCaptureServoDown();
        //robotSleep(500);
        //liftMotorRTPDriveWithStone(robot);
        encodersMoveStrafe(robot, 13, .8, "left");
        encodersMoveRTP(robot, 62, .8, "backward");
    }

    public void secondLeftSkyStoneRed (Robot robot) {
        encodersMoveRTP(robot,70, .8,"backward");
        runIntake(robot, 0.0);
        imuTurnPID(rrPID180, robot, 178, "right");
        encodersMoveStrafe(robot, 13, .7, "right");
        runIntake(robot, -1.0);
        encodersMoveRTP(robot,9, 0.3, "forward");
        double startTouchTime = System.currentTimeMillis();
        while (robot.getStoneDistance() <= 1.1 && System.currentTimeMillis()-startTouchTime < 1500) {} //keep going as long as it is under 1.5 seconds and the distance sensor is under 1 cm
        runIntake(robot, 0.0);
        encodersMoveStrafe(robot, 15, .7,"left");
        encodersMoveRTP(robot, 65, .8, "backward");
    }

    public void secondRightSkyStoneRed(Robot robot){
        encodersMoveRTP(robot, 26, .8, "backward");
        runIntake(robot, 0.0);
        imuTurnPID(rrPID180, robot, 176, "left");
        encodersMoveStrafe(robot, 16, .5, "right");
        runIntake(robot, -1.0);
        encodersMoveRTP(robot, 10, .8, "forward");
        double startTouchTime = System.currentTimeMillis();
        while (robot.getStoneDistance() <= 1.1 && System.currentTimeMillis()-startTouchTime < 1500){}
        runIntake(robot, 0.0);
        //robot.setCaptureServoDown();
        //robotSleep(500);
        //liftMotorRTPDriveWithStone(robot);
        encodersMoveStrafe(robot, 16, .5, "left");
        encodersMoveRTP(robot, 56, .8, "backward");
    }

    public void secondRightSkyStoneBlue (Robot robot) {
        encodersMoveRTP(robot,70, .8,"backward");
        runIntake(robot, 0.0);
        imuTurnPID(rrPID180, robot, 178, "left");
        encodersMoveStrafe(robot, 13, .7, "left");
        runIntake(robot, -1.0);
        encodersMoveRTP(robot,9, 0.3, "forward");
        double startTouchTime = System.currentTimeMillis();
        while (robot.getStoneDistance() <= 1.1 && System.currentTimeMillis()-startTouchTime < 1500) {} //keep going as long as it is under 1.5 seconds and the distance sensor is under 1 cm
        runIntake(robot, 0.0);
        encodersMoveStrafe(robot, 15, .7,"right");
        encodersMoveRTP(robot, 65, .8, "backward");
    }

    public void secondLeftSkyStoneBlue(Robot robot){
        encodersMoveRTP(robot, 52, .8, "backward");
        runIntake(robot, 0.0);
        imuTurnPID(rrPID180, robot, 176, "left");
        encodersMoveStrafe(robot, 14, .5, "left");
        runIntake(robot, -1.0);
        encodersMoveRTP(robot, 10, .8, "forward");
        double startTouchTime = System.currentTimeMillis();
        while (robot.getStoneDistance() <= 1.1 && System.currentTimeMillis()-startTouchTime < 1500){}
        runIntake(robot, 0.0);
        //robot.setCaptureServoDown();
        //robotSleep(500);
        //liftMotorRTPDriveWithStone(robot);
        encodersMoveStrafe(robot, 15, .5, "right");
        encodersMoveRTP(robot, 60, .8, "backward");
    }

    public void rightStoneBlue(Robot robot){
        encodersMoveRTP(robot, 19, .8, "forward");
        imuTurnPID(rrPID, robot, 86,  "right");
        encodersMoveRTP(robot, 7, .6, "backward");
        encodersMoveStrafe(robot, 19,.6, "left");
        runIntake(robot, -1.0);
        encodersMoveRTP(robot, 9, .8, "forward");
        double startTouchTime = System.currentTimeMillis();
        while (robot.getStoneDistance() <= 1.1 && System.currentTimeMillis()-startTouchTime < 1500){}
        runIntake(robot, 0.0);
        //robot.setCaptureServoDown();
        //robotSleep(500);
        //liftMotorRTPDriveWithStone(robot);
        encodersMoveStrafe(robot, 12.5, .8, "right");
        encodersMoveRTP(robot, 50, .8, "backward");
//        encodersMoveStrafe(robot, 10, .5, "right");
//        stoneOnFoundation(robot);
//        resetStoneMechanism(robot);
//        encodersMoveStrafe(robot, 5, 0.5, "left");
    }

    public void leftStoneBlue(Robot robot){
        encodersMoveRTP(robot, 20.5, .8, "forward");
        encodersMoveStrafe(robot, 25, .5, "left");
        imuTurnPID(rrPID,robot, 45,  "right");
        //robotSleep(100);
        runIntake(robot, -1.0);
        encodersMoveRTP(robot, 20, .3, "forward");
        double startTouchTime = System.currentTimeMillis();
        while (robot.getStoneDistance() <= 1.1 && System.currentTimeMillis()-startTouchTime < 5000){}
        runIntake(robot, 0.0);
        //robot.setCaptureServoDown();
        //robotSleep(500);
//        liftMotorRTPDriveWithStone(robot);
        encodersMoveRTP(robot, 13, .8, "backward");
        imuTurnPID(rrPID,robot, 57,  "right");
        //encodersMoveStrafe(robot, 20, .5, "right");
        encodersMoveRTP(robot, 28, .8, "backward");
//        stoneOnFoundation(robot);
//        resetStoneMechanism(robot);
//        encodersMoveStrafe(robot, 5, 0.5, "left");
    }

    public void middleStoneBlue(Robot robot){
        encodersMoveRTP(robot, 18, .8, "forward");
        imuTurnPID(rrPID, robot, 85,  "right");
        encodersMoveRTP(robot, 13.5, .6, "backward");
        encodersMoveStrafe(robot, 19, .7, "left");//D = 19, P = .5
        runIntake(robot, -1.0);
        encodersMoveRTP(robot, 10, 0.5, "forward");
        double startTouchTime = System.currentTimeMillis();
        while (robot.getStoneDistance() <= 1.1 && System.currentTimeMillis()-startTouchTime < 1500) {} //keep going as long as it is under 1.5 seconds and the distance sensor is under 1 cm
        runIntake(robot, 0.0);
        //robot.setCaptureServoDown();
        //robotSleep(500);
        //liftMotorRTPDriveWithStone(robot);
        encodersMoveStrafe(robot, 12.5, .6, "right");
        encodersMoveRTP(robot, 42, .8, "backward");
//        encodersMoveStrafe(robot, 10, .5, "right");
//        stoneOnFoundation(robot);
//        resetStoneMechanism(robot);
//        encodersMoveStrafe(robot, 5, 0.5, "left");
    }

    public void leftStoneRed(Robot robot){
        encodersMoveRTP(robot, 18, .8, "forward");
        imuTurnPID(rrPID, robot, 83, "left");
        encodersMoveRTP(robot, 10, .6, "backward");
        encodersMoveStrafe(robot, 19, .5, "right");
        runIntake(robot, -1.0);
        encodersMoveRTP(robot, 10, .2, "forward");
        double startTouchTime = System.currentTimeMillis();
        while (robot.getStoneDistance() <= 1.1 && System.currentTimeMillis()-startTouchTime < 1500){}
        runIntake(robot, 0.0);
        robot.setCaptureServoDown();
        robotSleep(500);
        liftMotorRTPDriveWithStone(robot);
        encodersMoveStrafe(robot, 13, .8, "left");
        encodersMoveRTP(robot, 50, .8, "backward");
//        encodersMoveStrafe(robot, 10, .5, "left");
//        stoneOnFoundation(robot);
//        resetStoneMechanism(robot);
//        encodersMoveStrafe(robot, 5, 0.5, "right");
    }

    public void middleStoneRed(Robot robot){
        encodersMoveRTP(robot, 18, .8, "forward");
        imuTurnPID(rrPID, robot, 88,  "left");
        encodersMoveRTP(robot, 15.5, .6, "backward");
        encodersMoveStrafe(robot, 20, .5, "right");
        runIntake(robot, -1.0);
        encodersMoveRTP(robot, 9, .4, "forward");
        double startTouchTime = System.currentTimeMillis();
        while (robot.getStoneDistance() <= 1.1 && System.currentTimeMillis()-startTouchTime < 1500) {} //keep going as long as it is under 1.5 seconds and the distance sensor is under 1 cm
        runIntake(robot, 0.0);
        //robot.setCaptureServoDown();
        //robotSleep(500);
        //liftMotorRTPDriveWithStone(robot);
        encodersMoveStrafe(robot, 14, .8, "left");
        encodersMoveRTP(robot, 47, .8, "backward");
        imuTurnPID(rrPID180, robot, 181, "left");
        runIntake(robot, 1.0);
//      encodersMoveStrafe(robot, 10, .5, "left");
//      stoneOnFoundation(robot);
//      resetStoneMechanism(robot);
//      encodersMoveStrafe(robot, 5, 0.5, "right");
    }

    public void rightStoneRed(Robot robot){
        encodersMoveRTP(robot, 18, .8, "forward");
        encodersMoveStrafe(robot, 25, .5, "right");
        imuTurnPID(rrPID, robot, 40,  "left");
        runIntake(robot, -1.0);
        encodersMoveRTP(robot, 24, .2, "forward");
        double startTouchTime = System.currentTimeMillis();
        while (robot.getStoneDistance() <= 1.1 && System.currentTimeMillis()-startTouchTime < 3500){}
        runIntake(robot, 0.0);
        encodersMoveRTP(robot, 18, .8, "backward");
        imuTurnPID(rrPID, robot, 55, "left");
        //encodersMoveStrafe(robot, 2, .5, "left");
        encodersMoveRTP(robot, 20, .8, "backward");
//        stoneOnFoundation(robot);
//        resetStoneMechanism(robot);
//        encodersMoveStrafe(robot, 5, 0.5, "right");

    }

    public void middle2ndSkyStoneRed(Robot robot){   //REQUIRES TESTING!!!!!!!!!!!
        encodersMoveRTP(robot, 50, .8, "backward");
        runIntake(robot, 0.0);
        imuTurnPID(rrPID180, robot, 180, "left");
        encodersMoveStrafe(robot, 20, 0.7, "left");//D = 19, P = .5
        runIntake(robot, -1.0);
        encodersMoveRTP(robot, 10, 0.3, "forward");
        double startTouchTime = System.currentTimeMillis();
        while (robot.getStoneDistance() <= 1.1 && System.currentTimeMillis()-startTouchTime < 1500) {} //keep going as long as it is under 1.5 seconds and the distance sensor is under 1 cm
        runIntake(robot, 0.0);
        //robot.setCaptureServoDown();
        //robotSleep(500);
        //liftMotorRTPDriveWithStone(robot);
        encodersMoveStrafe(robot, 11.5, .8, "right");
        encodersMoveRTP(robot, 60, .8, "backward");
    }

    public void left2ndSkyStoneRed(Robot robot){
        encodersMoveRTP(robot, 58, .8, "backward");
        runIntake(robot, 0.0);
        imuTurnPID(rrPID180, robot, 180, "left");
        encodersMoveStrafe(robot, 17, .5, "right");
        runIntake(robot, -1.0);
        encodersMoveRTP(robot, 10, .8, "forward");
        double startTouchTime = System.currentTimeMillis();
        while (robot.getStoneDistance() <= 1.1 && System.currentTimeMillis()-startTouchTime < 1500){}
        runIntake(robot, 0.0);
        //robot.setCaptureServoDown();
        //robotSleep(500);
        //liftMotorRTPDriveWithStone(robot);
        encodersMoveStrafe(robot, 17, .5, "left");
        encodersMoveRTP(robot, 60, .8, "backward");
    }

    public void right2ndSkyStoneRed (Robot robot){
        encodersMoveRTP(robot,67 , .8,"backward");
        runIntake(robot, 0.0);
        imuTurnPID(rrPID180, robot, 180, "left");
        encodersMoveStrafe(robot, 21, .7, "right");
        runIntake(robot, -1.0);
        encodersMoveRTP(robot,9, 0.3, "forward");
        double startTouchTime = System.currentTimeMillis();
        while (robot.getStoneDistance() <= 1.1 && System.currentTimeMillis()-startTouchTime < 1500) {} //keep going as long as it is under 1.5 seconds and the distance sensor is under 1 cm
        runIntake(robot, 0.0);
        encodersMoveStrafe(robot, 15, .7,"left");
        encodersMoveRTP(robot, 65, .8, "backward");
    }

    public void stoneOnFoundation(Robot robot){
        int liftCount = (int)robot.liftCalculateCounts(18);
        robot.setLiftMotorTargetPosition(liftCount);
        robot.setLiftMotorPower(0.8);
        while (opModeIsActive() && robot.getCurrentLiftPosition() < liftCount - 50){}
        robot.setLiftMotorPower(0.0);
        robot.setStoneSwingServoOut();
        robotSleep(1000);
        robot.setCaptureServoUp();
        robotSleep(1000);
    }

    public void resetStoneMechanism(Robot robot){
        robot.setStoneSwingServoIn();
        robotSleep(1000);
        int liftPositionDown = (int)robot.getCurrentLiftPosition() - (int)robot.liftCalculateCounts(18);
        telemetry.addLine().addData("current position",robot.getCurrentLiftPosition());
        robot.setLiftMotorTargetPosition(liftPositionDown);
        robot.setLiftMotorPower(0.8);
        double liftDownTime = System.currentTimeMillis();
        while (opModeIsActive() && robot.getCurrentLiftPosition() > liftPositionDown - 55 && (System.currentTimeMillis() - liftDownTime) < 1500){
            telemetry.addLine().addData("encoderCount", robot.getCurrentLiftPosition());
            telemetry.addLine().addData("liftPosition", liftPositionDown);
            telemetry.update();
        }
        robot.setLiftMotorPower(0.0);
        robot.resetLiftEncoder();
        robot.runLiftWithEncoderRTP();

    }

    public void blueFoundation(Robot robot){
        robot.setCapstoneElbowDown();
        encodersMoveRTP(robot, 10, .8, "backward"); //robot moves backwards 30 inches
        encodersMoveStrafe(robot, 10, .5, "right"); //robot strafes to the middle of foundation
        encodersMoveRTP(robot, 21, .5, "backward");
        robot.setFoundationGrabberGrabbed(); //foundation servos go down
        robotSleep(2000);
        encodersMoveRTP(robot, 25, .8, "forward"); //robot moves forwards 15 inches
        imuTurnPID(rrPID, robot, 70,  "left"); //robot turns 90 degrees right
        //robotSleep(1000);
        robot.setFoundationGrabberUnGrabbed(); //foundation servos come up
        robotSleep(2000);
        encodersMoveRTP(robot, 30, .8, "backward"); //robot moves to wall
        encodersMove(robot, 5, .8, "forward");
        encodersMoveStrafe(robot, 8, .5, "right");
        encodersMoveRTP(robot, 30, .8, "forward"); //robot parks under SkyBridge
        encodersMoveStrafe(robot, 10, .8, "right");
    }

    public void redFoundation(Robot robot) {
        encodersMoveRTP(robot, 10, .8, "backward"); //robot moves backwards 30 inches
        encodersMoveStrafe(robot, 10, .5, "left"); //robot strafes to the middle of foundation
        encodersMoveRTP(robot, 21, .5, "backward");
        robot.setFoundationGrabberGrabbed(); //foundation servos go down
        robotSleep(2000);
        encodersMoveRTP(robot, 25, .8, "forward"); //robot moves forwards 15 inches
        imuTurnPID(rrPID, robot, 70,  "right"); //robot turns 90 degrees right
        //robotSleep(1000);
        robot.setFoundationGrabberUnGrabbed(); //foundation servos come up
        robotSleep(2000);
        encodersMoveRTP(robot, 30, .8, "backward"); //robot moves to wall
        encodersMove(robot, 5, .8, "forward");
        encodersMoveStrafe(robot, 8, .5, "left");
        encodersMoveRTP(robot, 30, .8, "forward"); //robot parks under SkyBridge
        encodersMoveStrafe(robot, 10, .8, "left");
    }

    public void parkSkyBridgeBlue(Robot robot){
        encodersMoveRTP(robot, 30, .5, "forward");
        encodersMoveStrafe(robot, 25, .5, "right");
    }

    public void parkSkyBridgeRed(Robot robot){
        encodersMoveRTP(robot, 10, .5, "backward");
        encodersMoveStrafe(robot, 5, .5, "left");
    }

    public void deliverStone(Robot robot){
        imuTurnPID(rrPID,robot, 160,  "right");
        robot.setCaptureServoUp();
        runIntake(robot, 1.0);
        encodersMoveRTP(robot, 10, .8, "backward");
        runIntake(robot, 0.0);
    }

    public void loadSideFoundationRed(Robot robot){
        encodersMoveRTP(robot, 25, .8, "backward");
        imuTurn(robot, 70, .6, "left");
        encodersMoveRTP(robot, 12.5, .5, "backward");
        robot.setFoundationGrabberGrabbed(); //foundation servos go down
        robotSleep(2000);
        encodersMoveRTP(robot, 25, .8, "forward"); //robot moves forwards 15 inches
        imuTurn(robot, 70, .6, "right"); //robot turns 90 degrees right
        //robotSleep(1000);
        robot.setFoundationGrabberUnGrabbed(); //foundation servos come up
        robotSleep(2000);
        encodersMoveRTP(robot, 30, .8, "backward"); //robot moves to wall
        stoneOnFoundation(robot);
        resetStoneMechanism(robot);
        encodersMove(robot, 5, .8, "forward");
        encodersMoveStrafe(robot, 5, .5, "right");
        encodersMoveRTP(robot, 30, .8, "forward");

    }

    public void loadSideFoundationBlue(Robot robot){

        encodersMoveRTP(robot, 25, .8, "backward");
        imuTurnPID(rrPID, robot, 70, "right");
        encodersMoveRTP(robot, 12.5, .5, "backward");
        robot.setFoundationGrabberGrabbed(); //foundation servos go down
        robotSleep(2000);
        encodersMoveRTP(robot, 25, .8, "forward"); //robot moves forwards 15 inches
        imuTurnPID(rrPID, robot, 70, "left"); //robot turns 90 degrees right
        //robotSleep(1000);
        robot.setFoundationGrabberUnGrabbed(); //foundation servos come up
        robotSleep(2000);
        encodersMoveRTP(robot, 30, .8, "backward"); //robot moves to wall
        stoneOnFoundation(robot);
        resetStoneMechanism(robot);
        encodersMove(robot, 5, .8, "forward");
        encodersMoveStrafe(robot, 5, .5, "left");
        encodersMoveRTP(robot, 30, .8, "forward"); //robot parks under SkyBridge

    }

    /**
     * This is going to readjust the powers of the motors on the robot in order to keep it from going astray
     * @param robot
     * @param power
     * @param intendedHeading
     */
    public double[] strafingStraightPowerAdjustment (Robot robot, double power, double intendedHeading) {
        double[] motorPowers = new double[4]; //We are creating a list that goes: leftfront, rightfront, leftback, rightback

        double currentheading = robot.getIntegratedZAxis();
        double deltaheading = currentheading - intendedHeading;

        motorPowers[0] = power - (STRAFING_SCALE * Math.sin(Math.toRadians(deltaheading)));
        motorPowers[1] = power - (STRAFING_SCALE * Math.sin(Math.toRadians(deltaheading)));
        motorPowers[2] = power + (STRAFING_SCALE * Math.sin(Math.toRadians(deltaheading)));
        motorPowers[3] = power + (STRAFING_SCALE * Math.sin(Math.toRadians(deltaheading)));

        return motorPowers;
    }


    /**
     * Tuner method to tune the PID variables for driving straight
     * @param robotPID
     * @param robot
     * @param wantedDistance
     * @param direction
     */
    public void encoderDrivePIDTuner(RoboRaidersPID robotPID, Robot robot, double wantedDistance, double direction) {


        robot.resetEncoders();
        robot.runWithEncoders();
        robotPID.initialize();   // re-initialized the pid variables that we care about

        double EncoderCount = Math.abs(robot.driveTrainCalculateCounts(wantedDistance));
        double currentEncoderCount = robot.getSortedEncoderCount();
        if (direction == 0.0) {
            while (opModeIsActive() && (currentEncoderCount <= EncoderCount || currentEncoderCount >= EncoderCount ))
            {
                motor_power = robotPID.CalculatePIDPowers(EncoderCount, robot.getSortedEncoderCount());
                robot.setDriveMotorPower(motor_power, motor_power, motor_power, motor_power);
                currentEncoderCount = robot.getSortedEncoderCount();
            }

        }
        else if (direction == 1.0) {
            while (opModeIsActive() && (currentEncoderCount <= EncoderCount || currentEncoderCount >= EncoderCount ))
            {
                motor_power = robotPID.CalculatePIDPowers(EncoderCount, robot.getSortedEncoderCount());
                robot.setDriveMotorPower(-motor_power, -motor_power, -motor_power, -motor_power);
                currentEncoderCount = robot.getSortedEncoderCount();
            }

        }
    }


}


