package edu.byu.rvl.myvisiondriveapp;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowManager;
import android.widget.TextView;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Timer;
import java.util.TimerTask;

import ioio.lib.api.DigitalOutput;
import ioio.lib.api.PulseInput;
import ioio.lib.api.PwmOutput;
import ioio.lib.api.exception.ConnectionLostException;
import ioio.lib.util.BaseIOIOLooper;
import ioio.lib.util.IOIOLooper;
import ioio.lib.util.android.IOIOActivity;

public class MyVisionDriveActivity extends IOIOActivity implements View.OnTouchListener, CvCameraViewListener2 {
    private final static int DEBUG = 1;
    private static final String  TAG = "MyVisionDrive::Activity";

    private CameraBridgeViewBase mOpenCvCameraView;

//    public static final int VIEW_MODE_RGBA      = 0;
//    public static final int VIEW_MODE_GRAY      = 1;
//    public static final int VIEW_MODE_CANNY     = 2;
//    public static final int VIEW_MODE_FEATURES  = 5;

//    private MenuItem mItemPreviewRGBA;
//    private MenuItem mItemPreviewDIFF;
//    private MenuItem mItemPreviewCanny;
//    private MenuItem mItemPreviewFeatures;

//    private static int mViewMode = VIEW_MODE_RGBA;

    private Mat mIntermediateMat;
    private Mat mGray;

    private Mat mRgbMat;
    private Mat mHsvMat;
    private Mat mHueMat;
    private Mat mHueBin0;
    private Mat mHueBin1;
    private Mat mSatMat;
    private Mat mSatBin0;
    private Mat mSatBin1;
    private Mat mValMat;
    private Mat mValBin0;
    private Mat mValBin1;
    private Mat mFinalOrange;
    private Mat mFinalBlue;
    private Mat mFinalGreen;
    private Mat mFinalObstacle;
    private Mat mWork;

    public int mFrameWidth;
    public int mFrameHeight;
//    private boolean mObstacle;
    private Rect mRect;
//    private Mat mSub;
    private boolean ledFlag = false;

    private static final int NUM_ROWS = 20;
    private static final int NUM_COLS = 15;

    TimerTask mMyTimerTask;
    Thread mFreqThread;
//    private PulseInput mEncoderInput_;
    private float mEncoderFreq_;
    private float mEncoderFreqPrev;
    private int mForward = 1;
    private int mGoCnt = 1;
    private float mDesiredPwrFreq = 550;
    private static final float GO_NORM = 550;
    private static final float GO_FAST = 1200;
    private float mError = 0;
    private float mErrorPrev = 0;
    private float mTurnErrorMin = 0;
    private float mTurnErrorMax = 0;
    private float mTurnErrorGreen = 0;
    private int mGreenCol = NUM_COLS/2;
    private float mPwrPW = 1500;
    private float mTurnPW = 1500;
    private static final float mKp = 0.005f;
    private static final float mKd = 0f;
    private static final float mKpTurnMin = 3600f;
    private static final float mKpTurnMax = 60f;
    private static final float mKpTurnHard = 400f;
    private int mMaxCol = NUM_COLS/2;
    private static final int mCenter = NUM_COLS/2;
    private int mMinCol = 0;
    private static final int mColsWeight[] = {4,3,2,1,0,0,0,0,0,0,0,1,2,3,4};
    private int mCntRev = 0;
    private int mCntRevStop = 0;
    private boolean mFast = false;
    private int mCntFast = 0;
    private boolean mReverse = false;
    private boolean mReversePrev = false;
    //private static final int	cols_ignore[] = {6, 4, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 4, 6};
    //private static final int	obstacleThreshMin = 7;
    //private static final int	obstacleThreshMax = 12;

    private boolean mNewTime = true;
    private int mCounter = 0;
    private long mLastTime = 0;
    private long mTimeChecker = 0;

    private static final int threshO[] = {13,4,255,150,255,100};
    private static final int threshB[] = {112,102,255,100,255,50};
    private static final int threshG[] = {90,80,255,100,255,50};

    int mMin = NUM_ROWS;
    int mCntGreen = 0;
    int mCntGreen2 = 0;
    int mCntGreen2Max = 0;
    int mGreenI = NUM_COLS/2;
    float mMax = 0;
    int mSumLeft = 0;
    int mSumRight = 0;
    int mCntGood = 0;
    int mCntGoodMax = 0;
    int mGoodI = NUM_COLS/2;
    float mGoodCol = NUM_COLS/2;
    int mCntMin = 0;
    int mMassGood = 0;
    int mMassGoodMax = 0;
    String mOption = "";
    private static final int mScalar = 4;

    private SensorManager mSensor;
    //private Sensor mOrient;
    private Sensor mAccelerometer;
    private Sensor mMagnetometer;
    private float mRollAngle;
    private float mRollAngleRaw;
    private static final float ROLL_ANGLE_FLAT = -60;
    private float mAzAngle;
    private float mPitchAngle;
    private float[] mLastAccelerometer = new float[3];
    private float[] mLastMagnetometer = new float[3];
    private boolean mLastAccelerometerSet = false;
    private boolean mLastMagnetometerSet = false;
    private float[] mR = new float[9];
    private float[] mOrientation = new float[3];
    private static final int SENSOR_ARRAY_SIZE = 50;
    private ArrayList<Float> mPitchArray = new ArrayList<>();
    private ArrayList<Float> mPitchArrayTemp = new ArrayList<>();
    private ArrayList<Float> mRollArray = new ArrayList<>();
    private ArrayList<Float> mRollArrayTemp = new ArrayList<>();
    private int mSensorCnt = 0;
    private double mCurrLatitude, mCurrLongitude;
    private ArrayList<Waypoint> mWaypoints = new ArrayList<>();

    public static int[]	TouchX, TouchY;
    public static int StartX, StartY;
    public static int actionCode;
    public static int pointerCount = 0;
    public static int inputValue;

    Mat mRgba;
    Mat mDisplay;
    String Msg;
    int	bufferIndex;
    int FrameHeight;
    int FrameWidth;

    // IOIO Control
    boolean LED = false;
    int Frequency;
    int SteerOutput;
    int PowerOutput;
    boolean IOIO_Setup = false;
    DigitalOutput led_;
    PwmOutput steerOutput_;		// pwm output for turn motor
    PwmOutput powerOutput_;		// pwm output for drive motor
    PulseInput encoderInput_;     // pulse input to measure speed
    public static final int STEER_MAX = 1000;
    public static final int POWER_MAX = 2000;
    public static final int STEER_OFF = 1000;
    public static final int POWER_OFF = 500;

    private PwmOutput mTurnOutput_;		// pwm output for turn motor
    private PwmOutput mPwrOutput_;		// pwm output for drive motor

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i(TAG, "OpenCV load successful.");
                    mOpenCvCameraView.enableView();
                    mOpenCvCameraView.enableFpsMeter(); // TODO: Figure out what this does
                    mOpenCvCameraView.setOnTouchListener(MyVisionDriveActivity.this);
                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };

    public MyVisionDriveActivity(){
        if(DEBUG!=0)
            Log.i(TAG, "Instantiated new " + this.getClass());
    }

    public void onCreate(Bundle savedInstanceState) {
        if(DEBUG!=0)
            Log.i(TAG, "called onCreate");

        // Keep the screen on so the action can be seen.
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        MySensorListener senseList = new MySensorListener();

        mSensor = (SensorManager)getSystemService(SENSOR_SERVICE);
        //mOrient = mSensor.getDefaultSensor(Sensor.TYPE_ORIENTATION);
        mAccelerometer = mSensor.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        mMagnetometer = mSensor.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        //mSensor.registerListener(senseList, mOrient, SensorManager.SENSOR_DELAY_NORMAL);
        mSensor.registerListener(senseList, mAccelerometer, SensorManager.SENSOR_DELAY_FASTEST);
        mSensor.registerListener(senseList, mMagnetometer, SensorManager.SENSOR_DELAY_FASTEST);
        //mSensor.getOrientation(mSensor.getRotationMatrix(R,I,gravity,geomagnetic), values);

        LocationManager locationManager = (LocationManager) this.getSystemService(Context.LOCATION_SERVICE);
        LocationListener locationListener = new MyLocationListener();
        //mLovationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 1000, 1, mLocationListener);

        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        setContentView(R.layout.surface_view);

        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.surface_view);
        mOpenCvCameraView.setCvCameraViewListener(this);

        mRect = new Rect();
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_my_vision_drive, menu);
        if(DEBUG != 0) {
            Log.i(TAG, "called onCreateOptionsMenu");
        }
//        mItemPreviewRGBA  = menu.add("RGB");
    //    mItemPreviewDIFF  = menu.add("Difference");
        return true;
    }

    @Override
    public void onPause() {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
//        mLocationManager.removeUpdates(mLocationListener);
    }

    @Override
    public void onResume() {
        super.onResume();
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_6, this, mLoaderCallback);
        mLastAccelerometerSet = false;
        mLastMagnetometerSet = false;
    }

    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
//        mLocationManager.removeUpdates(mLocationListener);
    }

    /**
     * Initialize all related fields when the camera is ready.
     * @param width -  the width of the frames that will be delivered
     * @param height - the height of the frames that will be delivered
     */
    public void onCameraViewStarted(int width, int height) {
//        int i;
//        mRgba = new Mat[N_BUFFERS];
        int NUM_FINGERS = 10;
        TouchX = new int[NUM_FINGERS];
        TouchY = new int[NUM_FINGERS];
//        inputValue = 0;
//        bufferIndex = 0;
//        FrameHeight = height;
//        FrameWidth = width;
//        for (i=0; i<N_BUFFERS; i++) {
//            mRgba[i]= new Mat(height, width, CvType.CV_8UC4);
//        }
//        mDisplay= new Mat(height, width, CvType.CV_8UC4);

        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mIntermediateMat = new Mat(height, width, CvType.CV_8UC4);
        mGray = new Mat(height, width, CvType.CV_8UC1);
        mRgbMat = new Mat();
        mHsvMat = new Mat();
        mHueMat = new Mat();
        mHueBin0 = new Mat();
        mHueBin1 = new Mat();
        mSatMat = new Mat();
        mSatBin0 = new Mat();
        mSatBin1 = new Mat();
        mValMat = new Mat();
        mValBin0 = new Mat();
        mValBin1 = new Mat();
        mFinalOrange = new Mat();
        mFinalBlue = new Mat();
        mFinalGreen = new Mat();
        mFinalObstacle = new Mat();
        mWork = new Mat();
        mFrameWidth = width;
        mFrameHeight = height;
        int PIXELS_PER_SUB = ((width / NUM_COLS) * (height / NUM_ROWS)) / 2;
    }


    private int[] countEachSubmat(Mat m, int draw){
        Mat mSub = new Mat();
        int i,j,count = 0;
        int cols[] = new int[NUM_COLS];
        Rect colRect = new Rect();

        mRect.x = 0;
        mRect.y = colRect.y = 0;
        mRect.height = colRect.height = (mFrameHeight/ mScalar) / NUM_ROWS;
        mRect.width = (mFrameWidth/ mScalar) / NUM_COLS;
        colRect.height = mFrameHeight;

        for(i = 0; i < NUM_COLS; i++){
            mRect.x = colRect.x = i * mRect.width;
            mRect.y = 0;
            if(draw != 0){
                Core.rectangle(mRgba, new Point(colRect.x, colRect.y),
                        new Point((colRect.x + colRect.width - 1), (colRect.y + colRect.height - 1)),
                        new Scalar(255, 255, 0, 0));
            }

            for(j = NUM_ROWS - 1; j >= 0; j--){
//				if(j > NUM_COLS-cols_ignore[i])
//					continue;
//				if(j==NUM_ROWS-1)
//				{
//					// Ignore bottom corners that are not in the path of the vehicle
//					j -= cols_ignore[i];
//					continue;
//				}
//				if(draw!=0)
//				{
//					Core.rectangle(mRgba, new Point(mRect.x,mRect.y), new Point((mRect.x+mRect.width-1),(mRect.y+mRect.height-1)), new Scalar(255,255,0,0));
//				}
                mRect.y = j * mRect.height;
                mSub = m.submat(mRect);
                count = Core.countNonZero(mSub);
                if(count >= mRect.height * mRect.width/3) { //PIXELS_PER_SUB
                    cols[i] = NUM_ROWS - j;
                    break;
                }
            }
            if(j == -1) //If no obstacles were found for the current column
                cols[i] = NUM_ROWS;
            if(draw != 0){
                Core.putText(mRgba, "" + cols[i], new Point(colRect.x, 10), 3, .5, new Scalar(255, 0, 200, 255), 1);
            }
        }
        return cols;
    }

    /**
     * Releases all Mat objects, deallocating the memory they required.
     */
    public void onCameraViewStopped() {
        mRgba.release();
        mGray.release();
        mIntermediateMat.release();
        mRgbMat.release();
        mHsvMat.release();
        mHueMat.release();
        mHueBin0.release();
        mHueBin1.release();
        mSatMat.release();
        mSatBin0.release();
        mSatBin1.release();
        mValMat.release();
        mValBin0.release();
        mValBin1.release();
        mFinalOrange.release();
        mFinalBlue.release();
        mFinalGreen.release();
        mFinalObstacle.release();
        mWork.release();
    }

    /**
     * Generates a binarized combined matrix from the HSV matrices into dst
     * @param hueMat The Hue Matrix
     * @param satMat The Saturation Matrix
     * @param valMat The Value Matrix
     * @param dst The destination matrix
     * @param thresholds An array of 6 threshold values for each of the operations.
     */
    public void Binarize(Mat hueMat, Mat satMat, Mat valMat, Mat dst, int thresholds[]){
        final int PIXEL_MAX_VAL = 255;
        assert(thresholds.length == 6);

        //Here is what the threshold method takes as parameters:
        //Imgproc.threshold(source image, destination image, threshold value, maximum value to use , type of thresholding);

        // First we threshold all Hue values above thresholds[0] to 0...
        Imgproc.threshold(hueMat, mHueBin0, thresholds[0], 255, Imgproc.THRESH_TOZERO_INV);
        // ...Then we will threshold all of the new values to binary.
        Imgproc.threshold(mHueBin0, mHueBin1, thresholds[1], 255, Imgproc.THRESH_BINARY);

        //Rinse and repeat for Saturation...
        Imgproc.threshold(satMat, mSatBin0, thresholds[2], 255, Imgproc.THRESH_TOZERO_INV);
        Imgproc.threshold(mSatBin0, mSatBin1, thresholds[3], 255, Imgproc.THRESH_BINARY);

        // ...And Value.
        Imgproc.threshold(valMat, mValBin0, thresholds[4], 255, Imgproc.THRESH_TOZERO_INV);
        Imgproc.threshold(mValBin0, mValBin1, thresholds[5], 255, Imgproc.THRESH_BINARY);

        // bitwise and the Hue and Value binarized matrices
        Core.bitwise_and(mHueBin1, mValBin1, mWork);
        // Bitwise and the matrix from the previous operataion with the Saturation binarized matrix.
        // dst is the final binarized combined matrix.
        Core.bitwise_and(mWork, mSatBin1, dst);
    }

    /**
     * Runs every time the camera captures a frame.
     * @param inputFrame
     * @return
     */
    public Mat onCameraFrame(CvCameraViewFrame inputFrame){

        // Calculate the frames per second for analysis.
        if(mNewTime){
            mCounter = 1;
            mLastTime = System.currentTimeMillis();
            mNewTime = false;
        }
        else{
            mCounter++;
            mTimeChecker = System.currentTimeMillis();
            if((mTimeChecker - mLastTime) >= 1000){
                mNewTime = true;
            }
        }

        //mRgba = inputFrame.rgba();

        // Downsample the input image by a scalar value so FPS can increase
        Size reducedSize = new Size(inputFrame.rgba().cols()/ mScalar, inputFrame.rgba().rows()/mScalar);
        Size fullSize = new Size(inputFrame.rgba().cols(), inputFrame.rgba().rows());

        mRgba = new Mat(reducedSize, 1);

        // Resize mRgba to the new, reduced size
        Imgproc.resize(inputFrame.rgba(), mRgba, mRgba.size());

        // Process mRgba into an RGB Matrix
        Imgproc.cvtColor(mRgba, mRgbMat, Imgproc.COLOR_RGBA2RGB, 3);

        // Process mRgbMat into an HSV Matrix (3 channels)
        Imgproc.cvtColor(mRgbMat, mHsvMat, Imgproc.COLOR_RGB2HSV, 3);

        // Extract out each channel as its own matrix.
        Core.extractChannel(mHsvMat, mHueMat, 0);
        Core.extractChannel(mHsvMat, mSatMat, 1);
        Core.extractChannel(mHsvMat, mValMat, 2);

        // Binarize each of the matrices into orange, blue, and green binary matrices.
        Binarize(mHueMat, mSatMat, mValMat, mFinalOrange, threshO);
        Binarize(mHueMat, mSatMat, mValMat, mFinalBlue, threshB);
        Binarize(mHueMat, mSatMat, mValMat, mFinalGreen, threshG);

        // Add the orange and blue Matrices together.
        Core.add(mFinalOrange, mFinalBlue, mFinalObstacle);

        // Add the orange/blue combination and the green matrices together.
        Core.add(mFinalObstacle, mFinalGreen, mRgba);

        // FIND MIN AND MAX COLUMNS
        // Draw this first one to the screen.
        int cols[] = countEachSubmat(mFinalObstacle, 23);
        int cols_green[] = countEachSubmat(mFinalGreen, 0);

        // Calculate min value and index
        mMax = 0;
        mMin = NUM_ROWS;
        mMaxCol = NUM_COLS/2;
        mMinCol = NUM_COLS/2;
        mSumRight = 0;
        mSumLeft = 0;
        mCntGreen = 0;

        for(int i = 0; i < cols.length; i++){
            cols[i] = cols[i] + mColsWeight[i];

//            if(mRollAngle >= 80 && i > 3 && i < 11){
//                cols[i] = NUM_ROWS;
//            }

            if(cols[i] < mMin){
                mMin = cols[i];
                mMinCol = i;
            }

            if(cols[i] > mMax){
                mMax = cols[i];
                mMaxCol = i;
            }

            if(i > NUM_COLS/2){
                mSumRight = mSumRight + cols[i];
            }
            else if(i < NUM_COLS/2){
                mSumLeft = mSumLeft + cols[i];
            }

//			if(cols[i] > obstacleThreshMax || cols[i] < obstacleThreshMin)
//				wall = false;
            if(cols_green[i] < 5 && i > 3 && i < 11)
                mCntGreen= mCntGreen + 1;
        }
        //Core.putText(mRgba, "Min Column: "+ minCol, new Point(500,500), 3, 2, new Scalar(255, 0, 0, 255), 3);
        Log.i(TAG, "Sum right: " + mSumRight);
        Log.i(TAG, "Sum left: " + mSumLeft);

        mCntMin = 0;
        mCntRev = 0;
        mCntRevStop = 0;
        mMassGood = 0;
        mCntGood = 0;
        mMassGoodMax = 0;
        mCntGoodMax= 0;
        mGoodI = NUM_COLS/2;
        mCntGreen2 = 0;
        mCntGreen2Max = 0;
        mGreenI = NUM_COLS/2;

        // Calculate largest space between minimums
        // good_i will correspond to the rightmost clear column of the largest open space
        for(int i = 0; i < cols.length; i++)
        {
            if(cols[i] <= (mMin + 2)) { // if there is an obstacle
                if(i == 0 || i > 0 && cols[i-1] > mMin + 2) { // if the last index was not an obstacle
                    mCntMin = mCntMin + 1;
                }
                mMassGood = 0;
                mCntGood = 0;
            } else {
                mMassGood = mMassGood + cols[i];
                mCntGood = mCntGood + 1;
                if(mCntGood > mCntGoodMax) {
                    mMassGoodMax = mMassGood;
                    mCntGoodMax = mCntGood;
                    mGoodI = i;
                }
            }
            if(cols_green[i] <= 12) { // if there is green
                mCntGreen2 = mCntGreen2 + 1;
            } else {
                if(mCntGreen2 > mCntGreen2Max) {
                    mCntGreen2Max = mCntGreen2;
                    mGreenI = i;
                }
                mCntGreen2 = 0;
            }
            if(i > 2 && i < NUM_COLS - 2 && cols[i] < 3)
                mCntRev = mCntRev + 1;
            if(i > 2 && i < NUM_COLS - 2 && cols[i] < 6)
                mCntRevStop = mCntRevStop + 1;

        }

		/*if(cnt_green>1 && !fast) {
			fast = true;
			desiredPwrFreq = GO_FAST;
		}

		if(fast && cnt_fast<10)
			cnt_fast = cnt_fast+1;
		else if(fast) {
			fast = false;
			cnt_fast = 0;
			desiredPwrFreq = GO_NORM;
		}*/

        if(mCntRev >= 3) {
            //pwrPW = 1400;
            mReverse = true;
        } else if(mCntRevStop < 3) {
            //pwrPW = 1550;
            mReverse = false;
        }

        if(mRollAngle <= -80 || mCntGreen>1) {
            mFast = true;
            mReverse = false;
        } else {
            mFast = false;
        }

        if(mFast)
            mDesiredPwrFreq = GO_FAST;
        else
            mDesiredPwrFreq = GO_NORM;

        //if(cnt_green>1) {
        //	desiredPwrFreq = GO_FAST;
        //	forward = 1;
        //} else {
        //	desiredPwrFreq = GO_NORM;
        //	forward = 1;
        //}

        mGoodCol = mGoodI - (mCntGoodMax - 1) / 2;
        mGreenCol = mGreenI - (mCntGreen2Max - 1) / 2;

        //center = NUM_COLS/2;// - ((int)turnPW-1500)/200;

        // TURN LOOP //
        float sum_sign = Math.signum(mSumLeft - mSumRight);
        mTurnErrorMin = mMinCol - mCenter;
        mTurnErrorMax = mGoodCol - mCenter;
        mTurnErrorGreen = mGreenCol - mCenter;

        boolean colCheck = ((cols[4] > 13) && (cols[5] > 13) && (cols[6] > 13) && (cols[7]>13) &&
                (cols[8]>13) && (cols[9]>13) && (cols[10]>13));

        // The roll angle on flat ground should be 60 degrees
        /*if(mRollAngle < ROLL_ANGLE_FLAT - 7 || mRollAngle > ROLL_ANGLE_FLAT + 7){
            mTurnPW = 1500 - mPitchAngle / 2 * 100;
        }
        else*/ /*if(mCntGreen2Max > 0){
            mOption = "Green";
            mTurnPW = 1500 - mKpTurnMax * mTurnErrorGreen;
        }
        else*/ if(mReverse || mMin > 17 || !mFast && colCheck) {
            // Go straight
            mOption = "Straight";
            mTurnPW = 1500;
        }
        else if(mCntMin > 1 && mMin < 10){ // Multiple minimums, getting close
            // Turn Hard
            mOption= "Split Close";
            mTurnPW = 1500 - mKpTurnMax * mTurnErrorMax;
        }
        else if(mCntMin > 1){ // Multiple minimums, still far away
            // Turn but not Hard
            mOption = "Split Far";
            mTurnPW = 1500 - mKpTurnMax * mTurnErrorMax/mMin;
        }
        else if(mTurnErrorMin == 0 && mMin < 10){
            mOption = "Obstacle ahead close";
            mTurnPW = 1500 - mKpTurnHard * sum_sign;
        }
        else if(mTurnErrorMin == 0){
            mOption = "Obstacle ahead far";
            mTurnPW = 1500 - mKpTurnMin * sum_sign / mMin;
        }
        else if(mMin < 10){
            mOption = "Single close min";
            mTurnPW = 1500 + mKpTurnHard * Math.signum(mTurnErrorMin);
        }
        else {
            mOption = "Else";
            mTurnPW = 1500 + mKpTurnMin * Math.signum(mTurnErrorMin)/mMin;
        }

        Log.i(TAG, "Nav option " + mOption + "!");


        //Upsamples the image to its original size before returning it.
        Imgproc.resize(mRgba, mRgba, fullSize);
        //Core.putText(mRgba, ((Integer)counter).toString(),new Point(mFrameHeight/2,mFrameHeight/2), 3, .5, new Scalar(255, 0, 0, 255), 1);
//        Core.putText(mRgba, "Pitch: "+ mRollAngle + ", Roll: "+ mPitchAngle,
//                new Point(0, mFrameHeight/5), 3, 2,
//                new Scalar(255, 0, 0, 255), 1);
        Core.putText(mRgba, "Turn PWM: " + mTurnPW + " Power PWM: " + mPwrPW,
                new Point(mFrameWidth/4,mFrameHeight*2/5), 3, 2, new Scalar(255, 255, 102, 255), 1);
//        Core.putText(mRgba, "Azimuth: " + mAzAngle,
//                new Point(mFrameWidth/4,mFrameHeight*3/5), 3, 2, new Scalar(255, 0, 0, 255), 1);
        return mRgba;

    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        if(DEBUG != 0){
            Log.i(TAG, "called onOptionsItemSelected. Selected item: " + item);
        }
        return super.onOptionsItemSelected(item);
    }

    //TEST CODE
    static {
        System.loadLibrary("hello-android-jni");
    }
    public native String getMsgFromJni();
    //////////////////////////////////////////////////

    // Note that the FindFeatures() function is implemented at the native level in C++.
    // Furthermore, it is declared in Java as a native method
    // The C/C++ code for this example resides in the jni directory of the project.
    // The Android.mk and Application.mk makefiles in the jni directory are required by the Android
    // NDK compiler
    public native void FindFeatures(long matAddrGr, long matAddrRgba);


    @Override
    public boolean onTouch(View v, MotionEvent event) {
        Log.i(TAG, "TOUCH");
        try {
            pointerCount = event.getPointerCount();
            actionCode = event.getAction();
            Log.i(TAG, "Pointer count: " + pointerCount);
            Log.i(TAG, "Action: " + actionCode);
            int i;
            if (actionCode == MotionEvent.ACTION_DOWN) {
                Log.i(TAG, "Starting touch");                                // get the starting location from the first touch
                StartX = (int) event.getX(0);
                StartY = (int) event.getY(0);
                return true;
            } else if (actionCode == MotionEvent.ACTION_MOVE) {
                Log.i(TAG, "Moving");
                TouchX[0] = (int) event.getX(0);
                TouchY[0] = (int) event.getY(0);

                if(pointerCount == 1){ // control power
                    mPwrOutput_.setPulseWidth((POWER_MAX / 2) + POWER_OFF - (TouchY[0] - StartY)*2);
                } if (pointerCount == 2) { // control steering
                    mTurnOutput_.setPulseWidth((STEER_MAX / 2) + STEER_OFF - (TouchX[0] - StartX)*2);
                }
                return true;
            } else if (actionCode == MotionEvent.ACTION_UP /*&& pointerCount == 1*/) {   // update the distance
                Log.i(TAG, "Released");
                inputValue = (int) (TouchX[0] - StartX);
                mTurnOutput_.setPulseWidth((STEER_MAX / 2) + STEER_OFF);
                mPwrOutput_.setPulseWidth((POWER_MAX / 2) + POWER_OFF);
                return false;
            }
            return false;
        } catch(ConnectionLostException cle){
            Log.e(TAG, "Error");
            Log.e(TAG, cle.toString());
            return false;
        }
    }


    /** This loop runs continuously on a thread separate from the UI thread to conttrol the
     * IOIO board.
     */
    class Looper extends BaseIOIOLooper {

        private DigitalOutput mLed_;			/** The on-board LED. */

        /**
         * Called every time a connection with IOIO has been established.
         * Typically used to open pins.
         *
         * @throws ConnectionLostException
         *             When IOIO connection is lost.
         *
         * @see ioio.lib.util.AbstractIOIOActivity.IOIOThread#setup()
         */
        @Override
        protected void setup() throws ConnectionLostException {
            Log.i(TAG, "Setting up IOIO");
            mLed_ = ioio_.openDigitalOutput(0, true);
            mTurnOutput_ = ioio_.openPwmOutput(12, 100);
            mPwrOutput_ = ioio_.openPwmOutput(14, 100);
            encoderInput_ = ioio_.openPulseInput(3, PulseInput.PulseMode.FREQ);
        }

        /**
         * Called repetitively while the IOIO is connected.
         *
         * @throws ConnectionLostException
         *             When IOIO connection is lost.
         *
         * @see ioio.lib.util.AbstractIOIOActivity.IOIOThread#loop()
         */
        @Override
        public void loop() throws ConnectionLostException {
            Log.i(TAG, "Looping");
            //locationManager.requestLocationUpdates( LocationManager.GPS_PROVIDER, 0, 0, locationListener);

//            mMyTimerTask = new TimerTask() {
//                public void run() {
//                    mFreqThread.interrupt();
//                }
//            };
//
//            mFreqThread = new Thread() {
//                @Override
//                public void run() {
//                    Timer timer = new Timer();
//
//                    // Sets the interrupt to occur in 50 ms (as a timeout?).
//                    timer.schedule(mMyTimerTask, 50);
//                    try {
//                        mEncoderFreq_ = encoderInput_.getFrequency();
//
//                        // Cancel the interrupt to prevent throwing an error.
//                        mMyTimerTask.cancel();
//                    } catch (InterruptedException | ConnectionLostException e) {
//                        mEncoderFreq_ = 0;
//                        mEncoderFreqPrev = 0;
//                    }
//                }
//            };
//
//            mFreqThread.start();
//
//
//
//            // DRIVE LOOP //
//
//            // The error comes from the difference between what we are giving the PWM and what
//            // it is actually reporting.
//            mError = mDesiredPwrFreq - mEncoderFreq_;
//
//            if (Math.abs(mError * mKp) > 70 && mGoCnt > 0) { // Could this threshold be raised?
//                mError = Math.signum(mError) * 70 / mKp;
//            }
//
//            //if(cnt_rev>=3) {
//            //	pwrPW = 1400;
//            //	reverse = true;
//            //} else if(cnt_rev_stop<3 && reverse) {
//            //	pwrPW = 1550;
//            //	reverse = false;
//
//            //The roll angle on flat ground is 75 degrees. ??
//            if(mRollAngle > ROLL_ANGLE_FLAT + 5) {
//                mPwrPW = 1410;
//            } else if(mReverse) {
//                mPwrPW = 1400;
//                //reverse = true;
//            } else if(mReversePrev && !mReverse) {
//                mPwrPW = 1550;
//                mReverse = false;
//                //else if(encoderFreq_<desiredPwrFreq/5)
//                //	pwrPW = pwrPW + 50;
//            } else if(mEncoderFreq_ < mDesiredPwrFreq / 2 && mPwrPW < 1550)
//                mPwrPW = 1550;
//            else if(mEncoderFreq_ < mDesiredPwrFreq / 2 && mPwrPW < 1600)
//                mPwrPW = 1600;
//            else if(mEncoderFreq_ > 2.4 * mDesiredPwrFreq)
//                mPwrPW = 1450;
//                //else if(pwrPW>1700)
//                //	pwrPW = 1700;
//                //else if(pwrPW<1300)
//                //	pwrPW = 1300;
//            else
//                mPwrPW = mPwrPW + mForward * mKp *mError - mKd * (mEncoderFreq_ - mEncoderFreqPrev);
//
//            mReversePrev = mReverse;
//
//            mGoCnt = 1;
//
//            mErrorPrev = mError;
//            mEncoderFreqPrev = mEncoderFreq_;
//
//            Log.i(TAG, "Turn PW: " + mTurnPW);
//            mTurnOutput_.setPulseWidth(mTurnPW);
//            mPwrOutput_.setPulseWidth(mPwrPW);
//
//            mLed_.write(ledFlag);
//            ledFlag = !ledFlag;
//
//            try {
//                // Sleep on the thread for 100 ms so we are only running 10x per second.
//                Thread.sleep(100);
//            } catch (InterruptedException e) {
//            }
        }
    }

    /**
     * A method to create our IOIO thread.
     *
     * @see ioio.lib.util.AbstractIOIOActivity#createIOIOThread()
     */
    protected IOIOLooper createIOIOLooper() {
        return new Looper();
    }

    public class MySensorListener implements SensorEventListener {

        @Override
        public void onAccuracyChanged(Sensor arg0, int arg1) {
        }

        @Override
        public void onSensorChanged(SensorEvent event) {

            // Copy the values off of the sensor event.
            if (event.sensor == mAccelerometer) {
                System.arraycopy(event.values, 0, mLastAccelerometer, 0, event.values.length);
                mLastAccelerometerSet = true;
            } else if (event.sensor == mMagnetometer) {
                System.arraycopy(event.values, 0, mLastMagnetometer, 0, event.values.length);
                mLastMagnetometerSet = true;
            }

            if (mLastAccelerometerSet && mLastMagnetometerSet) {
                SensorManager.getRotationMatrix(mR, null, mLastAccelerometer, mLastMagnetometer);
                SensorManager.getOrientation(mR, mOrientation);
                if (DEBUG != 0)
                    Log.i("OrientationTestActivity", String.format("Orientation: %f, %f, %f",
                            mOrientation[0], mOrientation[1], mOrientation[2]));
            }

            mRollAngleRaw = (float) (mOrientation[2] * 180 / Math.PI);
            mAzAngle = (float) (mOrientation[0] * 180 / Math.PI);
            float pitchAngleRaw = (float) (mOrientation[1] * 180 / Math.PI);

            mRollArray.add(mRollAngleRaw);
            if (mRollArray.size() > SENSOR_ARRAY_SIZE)
                mRollArray.remove(0);

            mRollArrayTemp = (ArrayList<Float>) mRollArray.clone();

            Collections.sort(mRollArrayTemp);

            mRollAngle = mRollArrayTemp.get(mRollArrayTemp.size() / 2);


            mPitchArray.add(pitchAngleRaw);
            if (mPitchArray.size() > SENSOR_ARRAY_SIZE)
                mPitchArray.remove(0);

            mPitchArrayTemp = (ArrayList<Float>) mPitchArray.clone();

            Collections.sort(mPitchArrayTemp);

            mPitchAngle = mPitchArrayTemp.get(mPitchArrayTemp.size() / 2);

        }
    }

    // For GPS
    public class MyLocationListener implements LocationListener {
        @Override
        public void onLocationChanged(Location loc)
        {
            mCurrLatitude = loc.getLatitude();
            mCurrLongitude = loc.getLongitude();
        }

        @Override
        public void onProviderDisabled(String provider){}

        @Override
        public void onProviderEnabled(String provider){}

        @Override
        public void onStatusChanged(String provider, int status, Bundle extras){}
    }

    public void saveWaypoint(View view)
    {
        mWaypoints.add(new Waypoint(mCurrLatitude, mCurrLongitude));

//        ((TextView)findViewById(R.id.textView1)).setText(Integer.toString(mWaypoints.size()));
    }

    class Waypoint{
        double Latitude,Longitude;
        public Waypoint(double lat, double longi)
        {
            Latitude = lat;
            Longitude = longi;
        }
    }

//    // IOIO Functions
//    class Looper extends BaseIOIOLooper {
//         /**
//         * Called every time a connection with IOIO has been established.
//         * Typically used to open pins.
//         *
//         * @throws ioio.lib.api.exception.ConnectionLostException
//         *             When IOIO connection is lost.
//         *
//         * @see ioio.lib.util.IOIOLooper# setup()
//         */
//        @Override
//        protected void setup() throws ConnectionLostException {
//            showVersions(ioio_, "IOIO connected!");
//            led_ = ioio_.openDigitalOutput(0, true);
//            steerOutput_ = ioio_.openPwmOutput(12, 100);     // Hard Left: 2000, Straight: 1400, Hard Right: 1000
//            powerOutput_ = ioio_.openPwmOutput(14, 100);      // Fast Forward: 2500, Stop: 1540, Fast Reverse: 500
//            encoderInput_ = ioio_.openPulseInput(3, PulseInput.PulseMode.FREQ);
//            SteerOutput = STEER_MAX/2;
//            PowerOutput = POWER_MAX/2;
//            IOIO_Setup = true;
//        }
//
//        /**
//         * Called repetitively while the IOIO is connected.
//         *
//         * @throws ConnectionLostException
//         *             When IOIO connection is lost.
//         * @throws InterruptedException
//         * 				When the IOIO thread has been interrupted.
//         *
//         * @see ioio.lib.util.IOIOLooper#loop()
//         */
//        @Override
//        public void loop() throws ConnectionLostException, InterruptedException {
//            LED = (LED == true) ? false : true;
//            led_.write(LED);
//            steerOutput_.setPulseWidth(STEER_OFF + SteerOutput);   // Offset by 1000
//            powerOutput_.setPulseWidth(POWER_OFF + PowerOutput);    // offset by 500
//            Thread.sleep(100);
//        }
//
//        /**
//         * Called when the IOIO is disconnected.
//         *
//         * @see ioio.lib.util.IOIOLooper#disconnected()
//         */
//        @Override
//        public void disconnected() {
//            toast("IOIO disconnected");
//        }
//
//        /**
//         * Called when the IOIO is connected, but has an incompatible firmware version.
//         *
//         * @see ioio.lib.util.IOIOLooper#incompatible(ioio.lib.api.IOIO)
//         */
//        @Override
//        public void incompatible() {
//            showVersions(ioio_, "Incompatible firmware version!");
//        }
//    }
//
//    private void showVersions(IOIO ioio, String title) {
//        toast(String.format("%s\n" +
//                        "IOIOLib: %s\n" +
//                        "Application firmware: %s\n" +
//                        "Bootloader firmware: %s\n" +
//                        "Hardware: %s",
//                title,
//                ioio.getImplVersion(IOIO.VersionType.IOIOLIB_VER),
//                ioio.getImplVersion(IOIO.VersionType.APP_FIRMWARE_VER),
//                ioio.getImplVersion(IOIO.VersionType.BOOTLOADER_VER),
//                ioio.getImplVersion(IOIO.VersionType.HARDWARE_VER)));
//    }
//
//    private void toast(final String message) {
//        final Context context = this;
//        runOnUiThread(new Runnable() {
//            @Override
//            public void run() {
//                Toast.makeText(context, message, Toast.LENGTH_LONG).show();
//            }
//        });
//    }
//
//    class Waypoint{
//        double mLatitude,mLongitude;
//        public Waypoint(double lat, double longi){
//            mLatitude = lat;
//            mLongitude = longi;
//        }
//    }
}
