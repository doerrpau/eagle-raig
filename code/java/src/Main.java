import java.io.*;
import java.util.*;
import java.lang.*;
import java.awt.*;
import java.awt.image.*;
import javax.swing.*;

import april.jcam.*;
import april.util.*;
import april.jmat.*;
import april.vis.*;

public class Main
{
    static IMU imu_mpu = new IMU(IMU.IMUType.MPU6050);
    
    static ParameterGUI pg_controls = new ParameterGUI();
    static ParameterGUI pg_gyros = new ParameterGUI();
    static ParameterGUI pg_algos = new ParameterGUI();

    volatile static boolean running = false;

    static long time_started = 0;

    public Main() throws IOException {
    
         // Set up GUI
        JFrame jf = new JFrame("RAIG Control Panel (Z Axis)");
        jf.setLayout(null);
        ParameterGUIListener pgl = new ParameterGUIListener();
       
        // Add elements
        pg_controls.addButtons("reset", "Reset", "start", "Start", "psd", "PSD");
        pg_controls.addInt("time","Seconds", 0);
        for (int i = 0; i < imu_mpu.getNumSensors(); i++) {
            pg_gyros.addDouble("mpu" + i, "Z Axis MPU6050-" + i, 0.0);
        }
        for (int i = 0; i < imu_mpu.getNumSensors(); i++) {
            pg_gyros.addDouble("mpu_psd" + i, "PSD Z Axis MPU6050-" + i, 0.0);
        }

        pg_algos.addDouble("average", "Average of Gyros", 0.0);
        pg_algos.addDouble("average_psd", "PSD of Averaged Signal", 0.0);
        pg_algos.addDouble("best", "Best Single Gyro", 0.0);
        pg_algos.addDouble("wa", "Weighted Average", 0.0);
        pg_algos.addDouble("wa2", "Weighted Average 2", 0.0);
        
        jf.add(pg_controls);
        pg_controls.setBounds(0,400,900,150);
        pg_controls.addListener(pgl);
        jf.add(pg_gyros);
        pg_gyros.setBounds(0,0,350,400);
        jf.add(pg_algos);
        pg_algos.setBounds(500,0,350,400);

        // Show GUI
        jf.setSize(1000, 550);
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setVisible(true);   
    
    }

    class ParameterGUIListener implements ParameterListener
    {
        public void parameterChanged(ParameterGUI pg, String name)
        {

            if (name == "reset") {
                if (running) {
                    running = false;
                    imu_mpu.exit();
                    imu_mpu = new IMU(IMU.IMUType.MPU6050);;
                    System.gc();
                }
            } else if (name == "start") {
                if (!running) {
                    // Calibrate and start IMU
                    imu_mpu.calibrate(1000);
                    imu_mpu.start();
                    time_started = System.currentTimeMillis();
                    running = true;
                }
            } else if (name == "psd") {
                imu_mpu.calculatePSD(1000);
                for (int i = 0; i < imu_mpu.getNumSensors(); i++) {
                    pg_gyros.sd("mpu_psd" + i, IMU.toDegrees(IMU.toRRW(imu_mpu.getPSDs()[i][2])));
                }
                pg_algos.sd("average_psd", IMU.toDegrees(IMU.toRRW(imu_mpu.getAveragePSDs()[2])));
            }
        }
    }
    public static void main(String args[]) throws IOException
    {
        new Main();

        while (true) {

            if (running) {
                for (int i = 0; i < imu_mpu.getNumSensors(); i++) {
                    pg_gyros.sd("mpu" + i, imu_mpu.getSensorHeadings()[i][2]);
                }
                pg_algos.sd("average", imu_mpu.getAverageHeadings()[2]);
                pg_algos.sd("best", imu_mpu.getBestHeadings()[2]);
                pg_algos.sd("wa", imu_mpu.getWAverageHeadings()[2]);
                pg_algos.sd("wa2", imu_mpu.getWAverage2Headings()[2]);
                
                pg_controls.si("time", (int)(System.currentTimeMillis() - time_started)/1000);
                
            } else {
                for (int i = 0; i < imu_mpu.getNumSensors(); i++) {
                    pg_gyros.sd("mpu" + i, 0.0);
                }
                pg_algos.sd("average", 0.0);
                pg_algos.sd("best", 0.0);
                pg_algos.sd("wa", 0.0);
                pg_algos.sd("wa2", 0.0);
                
                pg_algos.si("time", 0);
            }
            try {
                Thread.sleep(200);
            } 
            catch (Exception e) { return; }

        }
        
        
        
        
        
        /*long start_time = System.currentTimeMillis();

        while (imu_mpu.getSensorHeadings()[0][2] > -3.14 && imu_mpu.getSensorHeadings()[0][2] < 3.14) {
            //for (int i = 0; i < imu_mpu.getNumSensors(); i++) {
            //    System.out.println("MPU" + i + ":\t" + imu_mpu.getSensorHeadings()[i][2]);
            //}
            System.out.println("MPU0:\t" + imu_mpu.getSensorHeadings()[0][2]);
            System.out.println("Average:\t" + imu_mpu.getAverageHeadings()[2]);
            try {
                Thread.sleep(200);
            } 
            catch (Exception e) { return; }
        }

        System.out.println("It took " + ((System.currentTimeMillis() - start_time)/1000) + 
                           " seconds for mpu random walk to exceed Pi radians");

        // Stop other threads
        imu_mpu.exit();
        RAIGDriver.getSingleton().close();*/
    }
    
}
