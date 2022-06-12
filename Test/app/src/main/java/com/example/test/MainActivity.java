package com.example.test;

import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.view.WindowManager;

import androidx.appcompat.app.AppCompatActivity;

import com.intel.realsense.librealsense.DeviceListener;
import com.intel.realsense.librealsense.RsContext;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.OpenCVLoader;

import com.example.test.databinding.ActivityMainBinding;

public class MainActivity extends AppCompatActivity{

    private static final String TAG = "MainActivity";
    private RsContext mRsContext;
    private static final int IMG_Width = 1280;
    private static final int IMG_Height = 720;

    private View sufure_view;
    private CameraBridgeViewBase mOpenCvCameraView;

    static {
        System.loadLibrary("slam");
    }

    private ActivityMainBinding binding;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        Log.d(TAG, "onCreate=======================>>>>>>>>>>>>");
        binding = ActivityMainBinding.inflate(getLayoutInflater());
        setContentView(binding.getRoot());

        RsContext.init(getApplicationContext());//realsense 必须进行init

        //Register to notifications regarding RealSense devices attach/detach events via the DeviceListener.
        mRsContext = new RsContext();

        mRsContext.setDevicesChangedCallback(new DeviceListener() {
            @Override
            public void onDeviceAttach() {
                Log.v(TAG, "onDeviceAttach");

            }
            @Override
            public void onDeviceDetach() {
                Log.v(TAG, "onDeviceDetach");
            }
        });

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        mOpenCvCameraView = findViewById(R.id.tutorial2_activity_surface_view);
        mOpenCvCameraView.setVisibility(CameraBridgeViewBase.VISIBLE);
        mOpenCvCameraView.setMaxFrameSize(IMG_Width,IMG_Height);
        mOpenCvCameraView.setClickable(true);

        sufure_view = findViewById(R.id.tutorial_view);
        sufure_view.setLongClickable(true);

        nativeStartLogger();//使能C++的cout输出
        Log.d(TAG, "创建成功!");
    }

    @Override
    protected void onResume() {
        Log.d(TAG, "onResume=======================>>>>>>>>>>>>");
        super.onResume();
        nativeOrbInitialised();

        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            mOpenCvCameraView.enableView();
        }
        Log.d(TAG, "onResume:==========>> ");
    }

    @Override
    protected void onPause() {
        super.onPause();
        Log.d(TAG, "onPause=======================>>>>>>>>>>>>");
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
        nativeOrbUninitialised();
    }

    @Override
    protected void onStop() {
        super.onStop();
        Log.d(TAG, "onStop=======================>>>>>>>>>>>>");
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();

        Log.d(TAG, "onDestroy:==========>>");
    }

    /**
     * A native method that is implemented by the 'orbslam' native library,
     * which is packaged with this application.
     */
    public native void nativeOrbInitialised();
    public native void nativeOrbUninitialised();
    public native void nativeStartLogger();
}
