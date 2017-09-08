package abr.predator;

import android.util.Log;

import ioio.lib.api.AnalogInput;
import ioio.lib.api.DigitalOutput;
import ioio.lib.api.PulseInput;
import ioio.lib.api.PwmOutput;
import ioio.lib.api.exception.ConnectionLostException;
import ioio.lib.util.BaseIOIOLooper;

public class IOIO_thread extends BaseIOIOLooper
{
	private PwmOutput pwm_left1, pwm_left2, pwm_right1,pwm_right2;
	private DigitalOutput dir_left1, dir_left2, dir_right1, dir_right2;
	private PulseInput encoder_leftA;
	private AnalogInput ir1, ir2, ir3;
	float ir1_reading, ir2_reading, ir3_reading;
	public boolean encoder_leftA_val;
	public int counter_left;
	float speed_left, speed_right;
	int move_val=1500,turn_val=1500;
	boolean direction_left, direction_right;

	@Override
	public void setup() throws ConnectionLostException
	{
		try
		{
			pwm_left1 = ioio_.openPwmOutput(3, 490); //motor channel 1: front left
			pwm_left2 = ioio_.openPwmOutput(5, 490); //motor channel 2: back left
			pwm_right1 = ioio_.openPwmOutput(7, 490); //motor channel 3: front right
			pwm_right2 = ioio_.openPwmOutput(10, 490); //motor channel 4: back right

			ir1 = ioio_.openAnalogInput(42);
			ir2 = ioio_.openAnalogInput(43);
			ir3 = ioio_.openAnalogInput(44);

			ir1_reading = 0.0f;
			ir2_reading = 0.0f;
			ir3_reading = 0.0f;

			dir_left1 = ioio_.openDigitalOutput(2, true);	//motor channel 1: front left
			dir_left2 = ioio_.openDigitalOutput(4, true);	//motor channel 2: back left
			dir_right1 = ioio_.openDigitalOutput(6, true); //motor channel 3: front right
			dir_right2 = ioio_.openDigitalOutput(8, true); //motor channel 4: back right

			encoder_leftA = ioio_.openPulseInput(11, PulseInput.PulseMode.POSITIVE);
			counter_left=0;

			direction_left = false;
			direction_right = false;
			speed_left = 0;
			speed_right = 0;

			(new Thread() {
				public void run() {
					while(true) {
						try {
							encoder_leftA.getDurationBuffered();
							counter_left++;
						} catch (Exception e){

						}
					}
				}
			}).start();
		}
		catch (ConnectionLostException e){throw e;}
	}

	@Override
	public void loop() throws ConnectionLostException
	{
		ioio_.beginBatch();

		try
		{
			int left_motor = 1500+(int)Math.round(.5*((move_val-1500)+(turn_val-1500)));
			int right_motor = 1500+(int)Math.round(.5*((move_val-1500)-(turn_val-1500)));
			Log.i("haha","lm:"+left_motor);
			Log.i("haha","rm:"+right_motor);
			if(left_motor > 1500){
				direction_left = true;
			} else {
				direction_left = false;
			}
			speed_left = Math.abs((float)(left_motor - 1500)/500);
			if(right_motor > 1500){
				direction_right = true;
			} else {
				direction_right = false;
			}
			speed_right = Math.abs((float)(right_motor - 1500)/500);

			ir1_reading = ir1.getVoltage();
			ir2_reading = ir2.getVoltage();
			ir3_reading = ir3.getVoltage();

			//This steering method will maximize the speed of the robot.
            /*
            if(move_val != 1500){
                direction_left = ((move_val - 1500) > 0);
                direction_right = ((move_val - 1500) > 0);
                speed_left = .18f; //18
                speed_right = .18f; //18
                if(turn_val > 1500){
                    speed_right -= .1;
                } else {
                    speed_left -= .1;
                }
            } else if(turn_val !=1500) {
                speed_left = 0.6f;
                speed_right = 0.6f;
                direction_left = ((turn_val - 1500) > 0);
                direction_right = ((turn_val - 1500) < 0);
            } else {
                speed_left = 0.0f;
                speed_right = 0.0f;
            }
            */

			pwm_left1.setDutyCycle(speed_left);
			pwm_left2.setDutyCycle(speed_left);
			pwm_right1.setDutyCycle(speed_right);
			pwm_right2.setDutyCycle(speed_right);

			dir_left1.write(direction_left);
			dir_left2.write(!direction_left);
			dir_right1.write(direction_right);
			dir_right2.write(!direction_right);

			Thread.sleep(10);
		}
		catch (InterruptedException e){ ioio_.disconnect();}
		finally{ ioio_.endBatch();}
	}

	public synchronized void move(int value)
	{
		move_val = value;
	}

	public synchronized void turn(int value)
	{
		turn_val = value;
	}

	public float get_ir1_reading() {
		return 100*((1f/15.7f*(-ir1_reading))+0.22f);
	}
	public float get_ir2_reading() {
		return 100*((1f/15.7f*(-ir2_reading))+0.22f);
	}
	public float get_ir3_reading() { return 100*((1f/15.7f*(-ir3_reading))+0.22f); }
}