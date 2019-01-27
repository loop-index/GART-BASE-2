package org.usfirst.frc.team6520.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
	Thread m_visionThread;
	double centerX = 0;
	Preferences prefs = Preferences.getInstance();
	double avgSize = 0;

	public void vision() {
		m_visionThread = new Thread(() -> {
			
			
			CvSink cvSink = CameraServer.getInstance().getVideo();
			CvSource outputStream
					= CameraServer.getInstance().putVideo("Vision", 320, 240);

			Mat mat;
			Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3));
			
			Mat hierarchy = new Mat();
			List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

			
			
			int R = prefs.getInt("R", 0);
			int G = prefs.getInt("G", 0);
			int B = prefs.getInt("B", 0);
			int RE = prefs.getInt("RE", 20);
			int GE = prefs.getInt("GE", 20);
			int BE = prefs.getInt("BE", 20);
			
			SmartDashboard.putNumber("R", R);
			SmartDashboard.putNumber("G", G);
			SmartDashboard.putNumber("B", B);
			SmartDashboard.putNumber("RE", RE);
			SmartDashboard.putNumber("GE", GE);
			SmartDashboard.putNumber("BE", BE);
			
			while (!Thread.interrupted()) {
				contours.removeAll(contours);
				mat = new Mat();

				boolean center = false;
				double bigSize = 0;
				
				prefs.putInt("R", R);
				prefs.putInt("G", G);
				prefs.putInt("B", B);
				prefs.putInt("RE", RE);
				prefs.putInt("GE", GE);
				prefs.putInt("BE", BE);

				if (cvSink.grabFrame(mat) == 0) {
					outputStream.notifyError(cvSink.getError());
					continue;
				}
				
				R = (int) SmartDashboard.getNumber("R", 255);
				G = (int) SmartDashboard.getNumber("G", 255);
				B = (int) SmartDashboard.getNumber("B", 255);
				RE = (int) SmartDashboard.getNumber("RE", 20);
				GE = (int) SmartDashboard.getNumber("GE", 20);
				BE = (int) SmartDashboard.getNumber("BE", 20);
				
				Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);
				Core.inRange(mat, new Scalar(R - RE, G - GE, B - BE), new Scalar(R + RE, G + GE, B + BE), mat);
				Imgproc.erode(mat, mat, kernel);
				
				Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

				List<Double[]> detected = new ArrayList<>();
				
				for (int i = 0; i < contours.size(); i++){
					
					if (Imgproc.contourArea(contours.get(i)) > 100){
						
						if (Imgproc.contourArea(contours.get(i)) > bigSize){
							bigSize = Imgproc.contourArea(contours.get(i));
						}
						
						MatOfPoint2f mat2 = new MatOfPoint2f(contours.get(i).toArray());
						RotatedRect rect = Imgproc.minAreaRect(mat2);
//						Imgproc.putText(mat, "" + (int) (rect.center.x) + ", " + (int) (rect.angle), new Point(rect.center.x, rect.center.y - 30), 0, 0.5, new Scalar(255, 255, 255));
						
						Imgproc.putText(mat, "" + Imgproc.contourArea(contours.get(i)), new Point(rect.center.x, rect.center.y - 30), 0, 0.5, new Scalar(255, 255, 255));
						
						Mat line = new Mat();
						Imgproc.fitLine(mat, line, Imgproc.CV_DIST_L2, 0, 0.01, 0.01);
						
						Double[] info = {rect.center.x, rect.angle, Imgproc.contourArea(contours.get(i))};
						detected.add(info);
					}
				}

				for (int i = detected.size() - 1; i > 0; i--){
					if (detected.get(i - 1)[1] > detected.get(i)[1] 
//							&&detected.get(i)[0] > detected.get(i + 1)[0]
									){
						center = true;
						centerX = (detected.get(i - 1)[0] + detected.get(i)[0])/2; 
						Imgproc.putText(mat, "*" , new Point(centerX, 160), 0, 0.5, new Scalar(255, 255, 255));
						avgSize = (detected.get(i - 1)[2] + detected.get(i)[2])/2;
						break;
					}
				}
				if (!center){
					avgSize = -1;
				}
				
				SmartDashboard.putBoolean("detected?", center);
				SmartDashboard.putNumber("size", bigSize);
				SmartDashboard.putNumber("contours", detected.size());
				outputStream.putFrame(mat);
//				RobotMap.drivebase.followX(25);
			}
		});
		m_visionThread.setDaemon(true);
		m_visionThread.start();
		
//		Power Cube: 0 - 170 - 200 - E: 50
		/*
		 * Target:
		 * R: 75 +- 50
		 * G: 200 +- 150
		 * B: 200 +- 75
		 */
	}
}