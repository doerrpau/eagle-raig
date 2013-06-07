import java.io.*;
import java.util.*;
import java.lang.*;
import java.util.Enumeration;

public class IMU implements Runnable
{
    // Singletons
    private static IMU lsmIMU = null;
    private static IMU mpuIMU = null;

    // Constants from the various implemented sensors
    private final double LSM330_RADIANS_PER_COUNT = 1.0;
    private final double MPU6050_RADIANS_PER_COUNT = 1.0;

    // Enum for implemented sensor types
    public enum IMUType 
    {
        LSM330, MPU6050
    }

    private int num_sensors;

    // IMU constants
    private Hashtable<int,double> radians_per_count;
    // IMU state data
    private Hashtable<int,double> current_heading;
    private Hashtable<int,long> time_prev;
    private volatile LinkedList<RAIGDriver2.IMUSamples> data_stream;

    public static IMU getSingleton(IMUType type)
    {
        switch (type)
        {
            case LSM330:
                if (lsmIMU == null) {
                    lsmIMU = new IMU(LSM330);
                }
                return lsmIMU;
            case MPU6050:
                if (mpuIMU == null) {
                    mpuIMU = new IMU(MPU6050);
                }
                return mpuIMU;
        }
    }

    protected IMU(IMUType type)
    {
        switch (type)
        {
            case LSM330:
                radians_per_count = LSM330_RADIANS_PER_COUNT;
                data_stream = RAIGDriver2.getSingleton().lsm_data;
                break;
            case MPU6050:
                radians_per_count = MPU6050_RADIANS_PER_COUNT;
                data_stream = RAIGDriver2.getSingleton().mpu_data;
                break;
        }

        current_heading = 0.0;
        time_prev = 0;
    }

    // Calibrate the IMU for calib_millis time
    public void calibrate(long calib_millis)
    {
        long start = System.currentTimeMillis();

        while (System.currentTimeMillis() - start < calib_millis) {
            
        }
    }

    public static void main(String args[]) throws IOException
    {
        RAIGDriver2 driver = new RAIGDriver2();
        while (true) {
            while (!driver.lsm_data.isEmpty()) {
                long time = driver.lsm_data.getFirst().timestamp;
                for (int i = 0; i < driver.lsm_data.getFirst().samples.size(); i++) {
                    RAIGDriver2.IMUSample lsm_samp = driver.lsm_data.getFirst().samples.get(i);
                    System.out.println("LSM" + lsm_samp.id + ": " + 
                            lsm_samp.rateX + " " +
                            lsm_samp.rateY + " " +
                            lsm_samp.rateZ + " " +
                            time
                            );
                }
                driver.lsm_data.clear();
            }
            while (!driver.mpu_data.isEmpty()) {
                long time = driver.mpu_data.getFirst().timestamp;
                for (int i = 0; i < driver.mpu_data.getFirst().samples.size(); i++) {
                    RAIGDriver2.IMUSample mpu_samp = driver.mpu_data.getFirst().samples.get(i);
                    System.out.println("MPU" + mpu_samp.id + ": " + 
                            mpu_samp.rateX + " " +
                            mpu_samp.rateY + " " +
                            mpu_samp.rateZ + " " +
                            time
                            );
                }
                driver.mpu_data.clear();
            }
            try {
                Thread.sleep(100);
            } 
            catch (Exception e) { return; }
        }
    }
}
