package com.example.test;

import android.content.Context;
import android.util.AttributeSet;
import android.util.Log;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class D435iView extends CameraBridgeViewBase {

    static {
        System.loadLibrary("slam");
    }

    private static final String TAG = "D435iView";

    private Thread mThread;
    private boolean mStopThread;
    private Mat mFrameChainLeftRgb;
    protected DisplayFrame mCameraFrameLeft;
    private boolean bHaveStereo = false;

    //需要根据实际的图像尺寸改变
    private int FrameWidth = 640;
    private int FrameHeight = 480;

    public D435iView(Context context, int cameraId) {
        super(context, cameraId);
    }

    public D435iView(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    @Override
    protected boolean connectCamera(int width, int height) {
        Log.v(TAG, "Starting processing thread width:" + width +" height: "+  height);//屏幕的大小

        mFrameWidth = FrameWidth;
        mFrameHeight = FrameHeight;
        mFrameChainLeftRgb = new Mat(FrameHeight,FrameWidth,CvType.CV_8UC3);

        mCameraFrameLeft = new DisplayFrame();

        AllocateCache();//mCacheBitmap

        if (mFpsMeter != null) {
            mFpsMeter.setResolution(mFrameWidth, mFrameHeight);
        }

        mStopThread = false;
        mThread = new Thread(new CameraWorker());
        mThread.start();

        return true;
    }

    @Override
    protected void disconnectCamera() {
        Log.d(TAG, "Disconnecting from camera");
        try {
            mStopThread = true;
            Log.d(TAG, "Notify thread");
            synchronized (this) {
                this.notify();
            }
            Log.d(TAG, "Waiting for thread");
            if (mThread != null)
                mThread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        } finally {
            mThread =  null;
        }
    }

    private class DisplayFrame implements CvCameraViewFrame{

        @Override
        public Mat rgba() {
            return mRgba;
        }

        @Override
        public Mat gray() {
            return mgray;
        }

        public DisplayFrame()
        {
            super();
            mRgba = new Mat(mFrameWidth, mFrameWidth, CvType.CV_8UC4);
            mgray = new Mat(mFrameWidth, mFrameWidth, CvType.CV_8UC1);
        }

        public Mat mgray;
        public Mat mRgba;
    }


    private class CameraWorker implements Runnable{

        @Override
        public void run() {
            do {
                Log.d(TAG, "第一步：进入jni获取界面");
//                Log.d(TAG,"最初的宽高是："+mFrameChainLeftRgb.cols() +" * "+  mFrameChainLeftRgb.height());
                nativeCheckCameraNew(mFrameChainLeftRgb.getNativeObjAddr());
//                Log.d(TAG, "这里就开始显示");
                Log.v(TAG, "mFrameChainLeftRgb的尺寸：宽：" + mFrameChainLeftRgb.cols() +" 高： "+  mFrameChainLeftRgb.height());//屏幕的大小
                Imgproc.cvtColor(mFrameChainLeftRgb, mCameraFrameLeft.mRgba, Imgproc.COLOR_RGB2BGRA);
                Imgproc.cvtColor(mFrameChainLeftRgb, mCameraFrameLeft.mgray, Imgproc.COLOR_RGB2GRAY);
                deliverAndDrawFrame(mCameraFrameLeft);//默认使用的图像格式为Rgba;
                Log.d(TAG, "run: runaround");

            }while (!mStopThread);
            Log.d(TAG, "Finish processing thread");
        }
    }

    public native void nativeCheckCameraNew(long ImageRgb);

}
