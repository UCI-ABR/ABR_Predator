package abr.predator;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.graphics.Bitmap;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.media.AudioManager;
import android.media.ToneGenerator;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.TextView;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Calendar;
import java.util.List;

import ioio.lib.util.IOIOLooper;
import ioio.lib.util.IOIOLooperProvider;
import ioio.lib.util.android.IOIOAndroidApplicationHelper;

public class Main_activity extends Activity implements IOIOLooperProvider, CvCameraViewListener2 // implements IOIOLooperProvider: from IOIOActivity
{
	private final IOIOAndroidApplicationHelper helper_ = new IOIOAndroidApplicationHelper(this, this); // from IOIOActivity

	boolean redRobot = true; //account for different gear ratios
	int forwardSpeed;
	int turningSpeed;
	int obstacleTurningSpeed;

	// ioio variables
	IOIO_thread_rover_tank m_ioio_thread;

	//blob detection variables
	private CameraBridgeViewBase mOpenCvCameraView;
	private Mat mRgba;
	private Scalar mBlobColorRgba;
	private ColorBlobDetector mDetectorYellow, mDetectorGreen;
	private Mat mSpectrum;
	private Scalar CONTOUR_COLOR;
	float[] greenRadius;
	float[] yellowRadius;
	Point yellowCenter;
	double centerDifference;
	//app state variables
	private boolean autoMode;

	//variables for logging
	private Sensor mGyroscope;
	private Sensor mGravityS;
	float[] mGravityV;
	float[] mGyro;

	float distance = 0;

	//variables for compass
	private SensorManager mSensorManager;
	private Sensor mCompass, mAccelerometer;
	float[] mGravity;
	float[] mGeomagnetic;
	public float heading = 0;
	public float bearing;

	//ui variables
	TextView sonar1Text;
	TextView sonar2Text;
	TextView sonar3Text;
	TextView distanceText;
	TextView bearingText;
	TextView headingText;

	//sockets for message passing
	Boolean isClient = true;
	ServerSocket serverSocket;
	Socket socket;
	Socket clientSocket;
	DataInputStream dataInputStream;
	DataOutputStream dataOutputStream;
	int receive;
	Boolean outsideNest = true;

	//occupancy grid variables
	int numSweeps = 0;
	int[][] occupancyGrid;
	Location[][] gridLocations;
	Location centerLocation;
	Location topLeft;
	Location bottomRight;
	int gridSize = 2;
	int sendCounter = 0;
	int destRow = 0;
	int destCol = 0;

	//timers
	int pauseCounter = 0;
	int backCounter = 0;
	Long startTime;
	Long currTime;
	int sleepCounter = 0;
	int turnAroundCounter = 0;
	int moveCounter = 0;

	//pan/tilt
	int panVal = 1500;
	int tiltVal = 1500;
	boolean panningRight = false;
	boolean tiltingUp = false;
	int panInc;
	int tiltInc;

	// called to use OpenCV libraries contained within the app as opposed to a separate download
	static {
		if (!OpenCVLoader.initDebug()) {
			// Handle initialization error
		}
	}

	// called whenever the activity is created
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);

		requestWindowFeature(Window.FEATURE_NO_TITLE);
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		setContentView(R.layout.activity_main_activity);

		helper_.create(); // from IOIOActivity

		//set up opencv camera
		mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.color_blob_detection_activity_surface_view);
		mOpenCvCameraView.setCvCameraViewListener(this);
		mOpenCvCameraView.enableView();

		//initialize textviews
		sonar1Text = (TextView) findViewById(R.id.sonar1);
		sonar2Text = (TextView) findViewById(R.id.sonar2);
		sonar3Text = (TextView) findViewById(R.id.sonar3);
		distanceText = (TextView) findViewById(R.id.distanceText);
		bearingText = (TextView) findViewById(R.id.bearingText);
		headingText = (TextView) findViewById(R.id.headingText);

		//add functionality to autoMode button
		Button buttonAuto = (Button) findViewById(R.id.btnAuto);
		buttonAuto.setOnClickListener(new OnClickListener() {
			public void onClick(View v) {
				if (!autoMode) {
					v.setBackgroundResource(R.drawable.button_auto_on);
					autoMode = true;
					startTime = System.currentTimeMillis();
				} else {
					v.setBackgroundResource(R.drawable.button_auto_off);
					autoMode = false;
				}
			}
		});

		//set starting autoMode button color
		if (autoMode) {
			buttonAuto.setBackgroundResource(R.drawable.button_auto_on);
		} else {
			buttonAuto.setBackgroundResource(R.drawable.button_auto_off);
		}

		//set up sockets for communication with other robots

		if (isClient) {
			try {
				Object[] objects = (new FileClientAsyncTask()).execute().get();
				socket = (Socket) objects[0];
				dataOutputStream = (DataOutputStream) objects[1];
				dataInputStream = (DataInputStream) objects[2];
			} catch (Exception e) {
				Log.e("rescue robotics", e.getMessage());
			}
		} else {
			try {
				Object[] objects = (new FileServerAsyncTask()).execute().get();
				serverSocket = (ServerSocket) objects[0];
				clientSocket = (Socket) objects[1];
				dataInputStream = (DataInputStream) objects[2];
				dataOutputStream = (DataOutputStream) objects[3];
			} catch (Exception e) {
				Log.e("rescue robotics", e.getMessage());
			}
		}


		//set speeds. adjust accordingly for your robot
		if (redRobot) {
			forwardSpeed = 180;
			turningSpeed = 100;
			obstacleTurningSpeed = 100;
		} else {
			forwardSpeed = 110;
			turningSpeed = 80;
			obstacleTurningSpeed = 90;
		}
	}


	//Called whenever activity resumes from pause
	@Override
	public void onResume() {
		super.onResume();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.enableView();
        /*
        if (mGoogleApiClient.isConnected()) {
           startLocationUpdates();
       }
       */
	}

	//Called when activity pauses
	@Override
	public void onPause() {
		super.onPause();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
	}

	//Called when activity restarts. onCreate() will then be called
	@Override
	public void onRestart() {
		super.onRestart();
		Log.i("activity cycle", "main activity restarting");
	}

	//Called when camera view starts. change bucket color here
	public void onCameraViewStarted(int width, int height) {
		mRgba = new Mat(height, width, CvType.CV_8UC4);
		mDetectorYellow = new ColorBlobDetector();
		mDetectorGreen = new ColorBlobDetector();
		mSpectrum = new Mat();
		mBlobColorRgba = new Scalar(255);
		CONTOUR_COLOR = new Scalar(255, 0, 0, 255);

		//To set color, find HSV values of desired color and convert each value to 1-255 scale
		//mDetector.setHsvColor(new Scalar(7, 196, 144)); // red
		mDetectorYellow.setHsvColor(new Scalar(6.5, 254, 175));        //yellow
		mDetectorGreen.setHsvColor(new Scalar(88, 255, 120));  //green
	}

	//Called when camera view stops
	public void onCameraViewStopped() {
		mRgba.release();
	}

	//Called at every camera frame. Main controls of the robot movements are in this function
	public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
      /*
      if(sendCounter > 100){
         sendGrid();
         sendCounter = 0;
      }
      getGrid();
      sendCounter++;
      */

		mRgba = inputFrame.rgba();
		mDetectorYellow.process(mRgba);
		mDetectorGreen.process(mRgba);

		List<MatOfPoint> contours1 = mDetectorYellow.getContours();
		List<MatOfPoint> contours2 = mDetectorGreen.getContours();
		// Log.e("rescue robotics", "Contours count: " + contours.size());
		Imgproc.drawContours(mRgba, contours1, -1, CONTOUR_COLOR);
		Imgproc.drawContours(mRgba, contours2, -1, CONTOUR_COLOR);

		Mat colorLabel = mRgba.submat(4, 68, 4, 68);
		colorLabel.setTo(mBlobColorRgba);

		Mat spectrumLabel = mRgba.submat(4, 4 + mSpectrum.rows(), 70,
				70 + mSpectrum.cols());
		mSpectrum.copyTo(spectrumLabel);

		/*****************************************************************************************************/
      /*                            PREDATOR SEARCH CODE                               */
		/*****************************************************************************************************/
		receive = getInt();

		if (receive == 2) {
			Log.i("receivedInt", "" + receive);
			ToneGenerator toneG = new ToneGenerator(AudioManager.STREAM_ALARM, 100);
			toneG.startTone(ToneGenerator.TONE_CDMA_PIP, 100);
			//m_ioio_thread.move(1500); // stop
			autoMode = true;
		}


		if (autoMode) {    //search if autoMode is on
			++sleepCounter;

			greenRadius = mDetectorGreen.getRadius();
			yellowRadius = mDetectorYellow.getRadius();
			Log.i("GreenRadius", Float.toString(greenRadius[0]));
			Log.i("YellowRadius", Float.toString(yellowRadius[0]));

			if (receive == 1) { // when the food is found by the agent or when it's starved
				ToneGenerator toneG = new ToneGenerator(AudioManager.STREAM_ALARM, 100);
				toneG.startTone(ToneGenerator.TONE_CDMA_PIP, 300);
				Log.i("receivedInt", "" + receive);

				m_ioio_thread.move(1500); // stop
				autoMode = false;

            /*try {
               socket.close();
            } catch (IOException e) {
               Log.e("tried socket close", e.getMessage());
            }
            */

			} else if (receive == 3) {
				ToneGenerator toneG = new ToneGenerator(AudioManager.STREAM_ALARM, 100);
				toneG.startTone(ToneGenerator.TONE_CDMA_PIP, 100);
				outsideNest = false;
				turnAroundCounter = 30;
				moveCounter = 30;
				Log.i("receivedInt", "" + receive);

			} else if (receive == 4) {
				ToneGenerator toneG = new ToneGenerator(AudioManager.STREAM_ALARM, 100);
				toneG.startTone(ToneGenerator.TONE_CDMA_PIP, 100);
				outsideNest = true;
				turnAroundCounter = 0;
				moveCounter = 0;
				Log.i("receivedInt", "" + receive);
			}

			/*if (!outsideNest) {
				turnAroundCounter = 30;
				backwardCounter = 30;
				//++turnAroundCounter;

				if (turnAroundCounter < 30) {
					m_ioio_thread.move(1500); //stop
					m_ioio_thread.turn(1600);
					Log.i("turn around","turning");
				} else {
					m_ioio_thread.turn(1500);
					m_ioio_thread.move(1600); //stop
					Log.i("turn around","moving");
				}
			}*/
			yellowCenter = mDetectorYellow.getCenter();
			Log.i("yellowCenter.x", String.format("%.2f", yellowCenter.x));
			if (sleepCounter % 60 > 40) {
				m_ioio_thread.move(1500); //move forward
			}
			else if(turnAroundCounter > 0){
				turnAroundCounter--;
				m_ioio_thread.move(1500); //stop
				m_ioio_thread.turn(1550);
				Log.i("turnaround","turning");
				if (yellowRadius[0] >=30 && outsideNest){
					turnAroundCounter = 0;
					moveCounter = 0;
				}
			} else if (moveCounter > 0){
				moveCounter--;
				if(m_ioio_thread.getCrashWarning1()) {
					moveCounter = 0;
				} else {
					m_ioio_thread.turn(1500);
					m_ioio_thread.move(1600);
				}
				Log.i("turnaround","moving");
				if (yellowRadius[0] >=30 && outsideNest){
					turnAroundCounter = 0;
					moveCounter = 0;
				}
			}
			else {
				//search for prey
				if (yellowRadius[0] <= 30) {
					//m_ioio_thread.move(1500); //stop
					//m_ioio_thread.turn(1400); //turn ccw
					turnAroundCounter = 30 + (int)(Math.random()*30);
					moveCounter = 60 + (int)(Math.random()*30);
				}
				//found object, but not prey
				else if (m_ioio_thread.getCrashWarning1() && yellowRadius[0] <= 30) {
					m_ioio_thread.avoid(); //avoid obstacle
					Log.i("HI 1", "Im stopped");
				}
				//found prey's home; turn around
				else if (greenRadius[0] > 700) {
					m_ioio_thread.move(1500); //stop
					m_ioio_thread.turn(1600); //turn cw
					Log.i("HI 2", "Im stopped");
				}
				else if(outsideNest) {
					//found prey on left; follow
					if (yellowCenter.x < 500 && (yellowRadius[0] < 320 && yellowRadius[0] > 30)) {
						m_ioio_thread.move(1500); //stop
						m_ioio_thread.turn(1400); //turn ccw
					}
					//found prey on right; follow
					else if (yellowCenter.x > 1100 && (yellowRadius[0] < 320 && yellowRadius[0] > 30)) {
						m_ioio_thread.move(1500); //stop
						m_ioio_thread.turn(1600); //turn cw
					}
					//caught prey; stop
					else if (yellowRadius[0] >= 320) {
						m_ioio_thread.move(1500); //stop
						Log.i("HI 3", "Im stopped");

						sendInt(1);
						ToneGenerator toneG = new ToneGenerator(AudioManager.STREAM_ALARM, 100);
						toneG.startTone(ToneGenerator.TONE_CDMA_CALL_SIGNAL_ISDN_PING_RING, 400);
						autoMode = false;
					}
					else {
						m_ioio_thread.turn(1500); //turn cw
						m_ioio_thread.move(1600); //stop
					}
				}
			}
		} else {
			Log.i("HI 5", "Im stopped");
			m_ioio_thread.move(1500); // stop

		}

		/*****************************************************************************************************/

		return mRgba;
	}

	//send an integer using output stream from socket
	public void sendInt(int intToSend) {
		if (dataOutputStream != null)
			try {
				dataOutputStream.writeInt(intToSend);
				Log.i("rescue robotics", "grid sent");
				Log.i("Predator", "Munch Munch");
				//socket.close();
			} catch (IOException e) {
				Log.e("rescue robotics", e.getMessage());
			}
	}

	//receive an integer using input stream from socket
	public int getInt() {
		try {
			if (dataInputStream != null && dataInputStream.available() >= 1) {
				Log.i("getInt", "received int");
				return dataInputStream.readInt();
			} else if (dataInputStream != null) {
				Log.i("getInt", "not null,dataInputStream.available=" + dataInputStream.available());
			} else {
				Log.i("getInt", "null,dataInputStream.available=" + dataInputStream.available());
			}
		} catch (IOException e) {
			Log.e("rescue robotics", e.getMessage());
		}
		return 0;
	}

	//send occupancy grid information using output stream from socket
	public void sendGrid() {
		if (dataOutputStream != null)
			try {
				dataOutputStream.writeInt(numSweeps);
				for (int i = 0; i < gridSize; i++) {
					for (int j = 0; j < gridSize; j++) {
						dataOutputStream.writeInt(occupancyGrid[i][j]);
					}
				}
				Log.i("rescue robotics", "grid sent");
			} catch (IOException e) {
				Log.e("rescue robotics", e.getMessage());
			}
	}

	//receive and update occupancy grid data using input stream from socket
	public void getGrid() {
		try {
			if (dataInputStream != null && dataInputStream.available() >= 4 * 17) {
				int receivedNumSweeps = dataInputStream.readInt();
				for (int i = 0; i < gridSize; i++) {
					for (int j = 0; j < gridSize; j++) {
						int receivedVal = dataInputStream.readInt();
						if ((receivedNumSweeps > numSweeps) || receivedVal > occupancyGrid[i][j])
							occupancyGrid[i][j] = receivedVal;
						Log.e("rescue robotics", "i:" + i + ",j:" + j + ",val:" + receivedVal);
					}
				}
				if (receivedNumSweeps > numSweeps)
					numSweeps = receivedNumSweeps;
			}
		} catch (IOException e) {
			Log.e("rescue robotics", e.getMessage());
		}
	}

	//construct the 2D matrix of Locations corresponding to the centers of grid squares
	//works only in northern hemisphere and not if your field crosses the prime meridian
	public static Location[][] calculateGridLocations(Location topLeft, Location bottomRight, int gridSize) {
		Location[][] result = new Location[gridSize][gridSize];
		for (int i = 0; i < gridSize; i++) {
			for (int j = 0; j < gridSize; j++) {
				//calculate dot around origin
				double c = Math.sqrt(Math.pow(topLeft.getLongitude() - bottomRight.getLongitude(), 2) + Math.pow(topLeft.getLatitude() - bottomRight.getLatitude(), 2));
				double x = c / Math.sqrt(2);
				double lon = -x / 2 + x / gridSize / 2 + i * x / gridSize;
				double lat = x / 2 - x / gridSize / 2 - j * x / gridSize;
				//calculate theta
				double midpt_lon = (topLeft.getLongitude() + bottomRight.getLongitude()) / 2;
				double midpt_lat = (topLeft.getLatitude() + bottomRight.getLatitude()) / 2;
				double topLeftOrig_lon = topLeft.getLongitude() - midpt_lon;
				double topLeftOrig_lat = topLeft.getLatitude() - midpt_lat;
				double topLeftOrig_theta = Math.atan2(topLeftOrig_lat, topLeftOrig_lon);
				double theta = topLeftOrig_theta - 3 * Math.PI / 4;
				//rotate dot around origin
				lon = lon * Math.cos(theta) - lat * Math.sin(theta);
				lat = lon * Math.sin(theta) + lat * Math.cos(theta);
				//translate dot to original offset location
				lon = lon + midpt_lon;
				lat = lat + midpt_lat;
				Location resultLoc = new Location("");
				resultLoc.setLongitude(lon);
				resultLoc.setLatitude(lat);
				result[j][i] = resultLoc;
				Log.i("rescue robotics", "lon:" + lon + ",lat:" + lat);
			}
		}
		return result;
	}

	//revert any degree measurement back to the -179 to 180 degree scale
	public float fixWraparound(float deg) {
		if (deg <= 180.0 && deg > -179.99)
			return deg;
		else if (deg > 180)
			return deg - 360;
		else
			return deg + 360;

	}

	//determine whether 2 directions are roughly pointing in the same direction, correcting for angle wraparound
	public boolean sameDir(float dir1, float dir2){
		float dir = bearing%360;
		float headingMod = heading%360;
		//return (Math.abs((double) (headingMod - dir)) < 22.5 || Math.abs((double) (headingMod - dir)) > 337.5);
		return (Math.abs((double) (headingMod - dir)) < 2.5 || Math.abs((double) (headingMod - dir)) > 357.5);
	}

	//set the text of any text view in this application
	public void setText(final String str, final TextView tv)
	{
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				tv.setText(str);
			}
		});
	}

	/****************************************************** functions from IOIOActivity *********************************************************************************/

	/**
	 * Create the {@link IOIO_thread}. Called by the
	 * {@link IOIOAndroidApplicationHelper}. <br>
	 * Function copied from original IOIOActivity.
	 *
	 * */
	@Override
	public IOIOLooper createIOIOLooper(String connectionType, Object extra)
	{
		if(m_ioio_thread == null && connectionType.matches("ioio.lib.android.bluetooth.BluetoothIOIOConnection"))
		{
			// enableUi(true);
			m_ioio_thread = new IOIO_thread_rover_tank();
			return m_ioio_thread;
		}
		else
		{
			return null;
		}
	}

	@Override
	protected void onDestroy() {
		super.onDestroy();
		Log.i("activity cycle","main activity being destroyed");
		helper_.destroy();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
	}

	@Override
	protected void onStart() {
		super.onStart();
		Log.i("activity cycle","main activity starting");
		helper_.start();
	}

	@Override
	protected void onStop() {
		Log.i("activity cycle","main activity stopping");
		super.onStop();
		helper_.stop();
		try {
			if(socket != null)
				socket.close();
			if(serverSocket != null)
				serverSocket.close();
			if(clientSocket != null)
				clientSocket.close();
		} catch (IOException e) {
			Log.e("rescue robotics", e.getMessage());
		}

	}

	@Override
	protected void onNewIntent(Intent intent) {
		super.onNewIntent(intent);
		if ((intent.getFlags() & Intent.FLAG_ACTIVITY_NEW_TASK) != 0) {
			helper_.restart();
		}
	}

}