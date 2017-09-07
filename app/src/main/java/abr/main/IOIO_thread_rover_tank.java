package abr.main;

import ioio.lib.api.AnalogInput;
import ioio.lib.api.DigitalOutput;
import ioio.lib.api.PulseInput;
import ioio.lib.api.PwmOutput;
import ioio.lib.api.exception.ConnectionLostException;



public class IOIO_thread_rover_tank extends IOIO_thread {
    private PwmOutput pwm_left1, pwm_left2, pwm_right1, pwm_right2;
    private DigitalOutput dir_left1, dir_left2, dir_right1, dir_right2;
    private AnalogInput irLeft, irCenter, irRight;
    private float speed_left, speed_right;
    private boolean direction_left, direction_right;
    private PulseInput encoder_left_A, encoder_right_A;
    public int counter_forward_left, counter_forward_right;
    public int counter_left, counter_right;
    public int counter_turn_left, counter_turn_right;
    private float irLeftReading, irCenterReading, irRightReading;

    public static final double ProximityThreshold = 0.7; //IR sensor threshold for collisions

    @Override
    public void setup() throws ConnectionLostException {
        try {
            pwm_left1 = ioio_.openPwmOutput(3, 490); //motor channel 1: front left
            pwm_left2 = ioio_.openPwmOutput(5, 490); //motor channel 2: back left
            pwm_right1 = ioio_.openPwmOutput(7, 490); //motor channel 3: front right
            pwm_right2 = ioio_.openPwmOutput(10, 490); //motor channel 4: back right

            dir_left1 = ioio_.openDigitalOutput(2, true);    //motor channel 1: front left
            dir_left2 = ioio_.openDigitalOutput(4, true);    //motor channel 2: back left
            dir_right1 = ioio_.openDigitalOutput(6, true); //motor channel 3: front right
            dir_right2 = ioio_.openDigitalOutput(8, true); //motor channel 4: back right

            encoder_left_A = ioio_.openPulseInput(11, PulseInput.PulseMode.POSITIVE);
            encoder_right_A = ioio_.openPulseInput(12, PulseInput.PulseMode.POSITIVE);

            irLeft = ioio_.openAnalogInput(42);
            irCenter = ioio_.openAnalogInput(43);
            irRight = ioio_.openAnalogInput(44);

            counter_left=0;
            counter_right=0;
            counter_forward_left=0;
            counter_forward_right=0;
            counter_turn_left=0;
            counter_turn_right=0;

            direction_left = false;
            direction_right = false;
            speed_left = 0;
            speed_right = 0;

            (new Thread() {
                public void run() {
                    while(true) {
                        try {
                            //counter_left = (int) encoder_left_A.getFrequency();
                            encoder_left_A.getDurationBuffered();
                            counter_left++;
                            counter_forward_left++;
                            counter_turn_left++;
                        } catch (Exception e){

                        }
                    }
                }
            }).start();

            (new Thread() {
                public void run() {
                    while(true) {
                        try {
                            //counter_left = (int) encoder_left_A.getFrequency();
                            encoder_right_A.getDurationBuffered();
                            counter_right++;
                            counter_forward_right++;
                            counter_turn_right++;
                        } catch (Exception e){

                        }
                    }
                }
            }).start();
        } catch (ConnectionLostException e) {
            throw e;
        }
    }

    @Override
    public void loop() throws ConnectionLostException {
        ioio_.beginBatch();

        try {
            pwm_left1.setDutyCycle(speed_left);
            pwm_left2.setDutyCycle(speed_left);
            pwm_right1.setDutyCycle(speed_right);
            pwm_right2.setDutyCycle(speed_right);

            dir_left1.write(direction_left);
            dir_left2.write(!direction_left);
            dir_right1.write(direction_right);
            dir_right2.write(!direction_right);

            irLeftReading = irLeft.getVoltage();
            irCenterReading = irCenter.getVoltage();
            irRightReading = irRight.getVoltage();

            // Log.d("robo", "in loop");


            Thread.sleep(10);
        } catch (InterruptedException e) {
            ioio_.disconnect();
        } finally {
            ioio_.endBatch();
        }
    }

    public void avoid() {
        float ir_left = getIrLeftReading();
        float ir_center = getIrCenterReading();
        float ir_right = getIrRightReading();

        if (ir_left > ProximityThreshold && ir_right > ProximityThreshold) {
            move(1400); //move backward

            if (ir_left > ir_right) {
                turn(1400); //turn cw
            } else {
                turn(1600); //turn ccw
            }
        }
        else if (ir_left > ProximityThreshold || ir_center > ProximityThreshold) {
            move(1500); // stop
            turn(1600); //turn cw
        }
        else if (ir_right > ProximityThreshold) {
            move(1500); // stop
            turn(1400); //turn ccw
        }
    }

    public synchronized void move(float leftSpeed, float rightSpeed, boolean leftForward, boolean rightForward) {
        speed_left = leftSpeed;
        speed_right = rightSpeed;
        direction_left = leftForward;
        direction_right = rightForward;
    }

    public synchronized void move(int value) { //.2 --> .15
        if (value > 1500) {
            speed_left = (float) 0.10;
            speed_right = (float) 0.10;
            direction_left = true;
            direction_right = true;
            counter_turn_left=0;
            counter_turn_right=0;
        } else if (value < 1500) {
            speed_left = (float) 0.10;
            speed_right = (float) 0.10;
            direction_left = false;
            direction_right = false;
            counter_turn_left=0;
            counter_turn_right=0;
        } else {
            speed_left = 0;
            speed_right = 0;
        }
    }

    public synchronized void turn(int value) { //.3 --> .25
        if (value > 1500) {
            speed_left = (float) 0.25;
            speed_right = (float) 0.25;
            direction_left = true;
            direction_right = false;
            counter_forward_left=0;
            counter_forward_right=0;
        } else if (value < 1500) {
            speed_left = (float) 0.25;
            speed_right = (float) 0.25;
            direction_left = false;
            direction_right = true;
            counter_forward_left=0;
            counter_forward_right=0;
        } else {
            speed_left = 0;
            speed_right = 0;
        }


    }

    //avoid crash warning
    public boolean getCrashWarning1() {
        float ir_left = getIrLeftReading();
        float ir_center = getIrCenterReading();
        float ir_right = getIrRightReading();

        if (ir_left > ProximityThreshold || ir_center > ProximityThreshold || ir_right > ProximityThreshold) {
            return true;
        } else {
            return false;
        }
    }

    //u-turn crash warning
    public boolean getCrashWarning2() {
        float ir_left = getIrLeftReading();
        float ir_center = getIrCenterReading();
        float ir_right = getIrRightReading();

        if ((ir_left > ProximityThreshold && ir_center > ProximityThreshold) || (ir_center > ProximityThreshold && ir_right > ProximityThreshold)) {
            return true;
        } else {
            return false;
        }
    }

    public float getIrLeftReading() {
        // Log.d("robo", "getIR");
        return irLeftReading;
    }

    public float getIrCenterReading() {
        return irCenterReading;
    }

    public float getIrRightReading() {
        return irRightReading;
    }
}