import java.io.*;
import java.util.*;
import java.lang.*;
import java.util.Enumeration;

public class IMU implements Runnable
{
    // Singletons
    private static IMU lsmIMU = null;
    private static IMU mpuIMU = null;

    // Used to calculate heading for each axis of a single sensor
    // Stores constants, calculated offsets, and headings
    private class IMUData
    {
        // Conversion constants (Raw to radians per millisec)
        private double kRateX = 0.0;
        private double kRateY = 0.0;
        private double kRateZ = 0.0;
        // 0-bias of the gyroscopes
        private double oRateX = 0.0;
        private double oRateY = 0.0;
        private double oRateZ = 0.0;
        // Total calculated heading of each axis
        public double headX = 0.0;
        public double headY = 0.0;
        public double headZ = 0.0;
        // Calibration duration
        private long calib_total_time = 0;
        // Power Spectral Density values
        private double noiseSqX = 0.0;
        private double noiseSqY = 0.0;
        private double noiseSqZ = 0.0;
        private double psd_total_time = 0.0;

        public IMUData(double krx, double kry, double krz)
        {
            kRateX = krx;
            kRateY = kry;
            kRateZ = krz;
        }

        public void add_cal_samp(RAIGDriver2.IMUSample samp, long prev_time, long curr_time)
        {
            long time_diff = curr_time - prev_time;
            // Ignore new sample if it came before previous sample
            if (time_diff < 0) {
                time_diff = 0;
            }
            oRateX += time_diff*samp.rateX;
            oRateY += time_diff*samp.rateY;
            oRateZ += time_diff*samp.rateZ;
            calib_total_time += time_diff;
        }

        public double xOffset()
        {
            if (calib_total_time == 0) {
                return 0.0;
            } else {
                return oRateX /= (double)calib_total_time;
            }
        }
        public double yOffset()
        {
            if (calib_total_time == 0) {
                return 0.0;
            } else {
                return oRateY /= (double)calib_total_time;
            }
        }
        public double zOffset()
        {
            if (calib_total_time == 0) {
                return 0.0;
            } else {
                return oRateZ /= (double)calib_total_time;
            }
        }

        public void add_samp(RAIGDriver2.IMUSample samp, long prev_time, long curr_time)
        {
            double time_diff = ((double)(curr_time - prev_time))/(1000.0*60.0*60.0);
            // Ignore new sample if it came before previous sample
            if (time_diff < 0.0) {
                time_diff = 0.0;
            }
            headX += time_diff*(samp.rateX - xOffset())*kRateX;
            headY += time_diff*(samp.rateY - yOffset())*kRateY;
            headZ += time_diff*(samp.rateZ - zOffset())*kRateZ;
        }

        // Used to calculate the noise power spectral density in (rad/sqrt(hr))^2/Hz
        public void add_psd_samp(RAIGDriver2.IMUSample samp, long prev_time, long curr_time)
        {
            double time_diff = ((double)(curr_time - prev_time))/(1000.0*60.0*60.0);
            // Ignore new sample if it came before previous sample
            if (time_diff < 0.0) {
                time_diff = 0.0;
            }
            noiseSqX += time_diff*Math.pow((samp.rateX - xOffset())*kRateX,2.0);
            noiseSqY += time_diff*Math.pow((samp.rateY - yOffset())*kRateY,2.0);
            noiseSqZ += time_diff*Math.pow((samp.rateZ - zOffset())*kRateZ,2.0);
            psd_total_time += time_diff;
        }

        // Rate Noise PSD in (rad/sqrt(hr))^2/Hz
        public double xPSD()
        {
            if (psd_total_time == 0.0) {
                return 0.0;
            } else {
                return noiseSqX /= psd_total_time;
            }
        }
        public double yPSD()
        {
            if (psd_total_time == 0.0) {
                return 0.0;
            } else {
                return noiseSqY /= psd_total_time;
            }
        }
        public double zPSD()
        {
            if (psd_total_time == 0.0) {
                return 0.0;
            } else {
                return noiseSqZ /= psd_total_time;
            }
        }

        // Angle Random Walk in rad/sqrt(hr)
        public double xARW()
        {
            return (1/60)*Math.sqrt(xPSD());
        }
        public double yARW()
        {
            return (1/60)*Math.sqrt(yPSD());
        }
        public double zARW()
        {
            return (1/60)*Math.sqrt(zPSD());
        }

        // Clear data so we can recalibrate etc.
        public void clearOffsets()
        {
            oRateX = 0.0;
            oRateY = 0.0;
            oRateZ = 0.0;
            calib_total_time = 0;
        }
        public void clearPSD()
        {
            noiseSqX = 0.0;
            noiseSqY = 0.0;
            noiseSqZ = 0.0;
            psd_total_time = 0.0;
        }
        
    }

    // Pre-defined IMUData from the various implemented sensors
    // Add new entry for each sensor, with proper measurements
    private final IMUData[] LSM330_DATA = new IMUData[]{
        new IMUData(1.0, 1.0, 1.0),
        new IMUData(1.0, 1.0, 1.0),
        new IMUData(1.0, 1.0, 1.0)
    };
    private final IMUData[] MPU6050_DATA = new IMUData[]{
        new IMUData(1.0, 1.0, Math.PI/6.66667),
        new IMUData(1.0, 1.0, Math.PI/6.66667),
        new IMUData(1.0, 1.0, Math.PI/6.66667),
        new IMUData(1.0, 1.0, Math.PI/6.66667)
    };

    // Enum for implemented sensor types
    public enum IMUType 
    {
        LSM330, MPU6050
    }

    // Total number of sensors that are usable
    private int num_sensors;
    // Number of active sensors
    private int active_sensors;

    // IMU state data
    private IMUData[] imu_data;
    private long[] prev_samp_time;
    private volatile LinkedList<RAIGDriver2.IMUSamples> data_stream;
    private boolean calibrated;

    // Warning: this function will take approx. 4 seconds on first call (to establish port connection)
    public static IMU getSingleton(IMUType type)
    {
        switch (type)
        {
            case LSM330:
                if (lsmIMU == null) {
                    lsmIMU = new IMU(IMUType.LSM330);
                }
                return lsmIMU;
            case MPU6050:
                if (mpuIMU == null) {
                    mpuIMU = new IMU(IMUType.MPU6050);
                }
                return mpuIMU;
        }
        return null;
    }

    protected IMU(IMUType type)
    {
        switch (type)
        {
            case LSM330:
                imu_data = LSM330_DATA;
                data_stream = RAIGDriver2.getSingleton().lsm_data;
                break;
            case MPU6050:
                imu_data = MPU6050_DATA;
                data_stream = RAIGDriver2.getSingleton().mpu_data;
                break;
        }

        // Initialize state structures
        num_sensors = imu_data.length;
        prev_samp_time = new long[num_sensors];
        calibrated = false;
        active_sensors = num_sensors;
    }

    // Calibrate the gyroscope offsets for calib_millis time
    // Requires still sensor for the duration
    public void calibrate(long calib_millis)
    {
        for (int i = 0; i < num_sensors; i++) {
            imu_data[i].clearOffsets();
        }

        long start = System.currentTimeMillis();

        // Sum gyroscope samples
        while (System.currentTimeMillis() - start < calib_millis) {
            while (!data_stream.isEmpty()) {
                for (int i = 0; i < data_stream.getFirst().samples.size(); i++) {
                    RAIGDriver2.IMUSample imu_samp = data_stream.getFirst().samples.get(i);
                    if (imu_samp.id >= 0 && imu_samp.id < num_sensors) {
                        long timestamp = data_stream.getFirst().timestamp;
                        if (prev_samp_time[imu_samp.id] != 0) {
                            imu_data[imu_samp.id].add_cal_samp(imu_samp, prev_samp_time[imu_samp.id], timestamp);
                        } 
                        prev_samp_time[imu_samp.id] = timestamp;
                    }
                }
                data_stream.remove();
            }
            try {
                Thread.sleep(10);
            } 
            catch (Exception e) { return; }
        }

        calibrated = true;
        return;
    }

    // Calculate PSD for psd_millis time
    // Requires still sensor for the duration
    public void calculatePSD(long psd_millis)
    {
        for (int i = 0; i < num_sensors; i++) {
            imu_data[i].clearPSD();
        }

        long start = System.currentTimeMillis();

        // Sum gyroscope samples
        while (System.currentTimeMillis() - start < psd_millis) {
            while (!data_stream.isEmpty()) {
                for (int i = 0; i < data_stream.getFirst().samples.size(); i++) {
                    RAIGDriver2.IMUSample imu_samp = data_stream.getFirst().samples.get(i);
                    if (imu_samp.id >= 0 && imu_samp.id < num_sensors) {
                        long timestamp = data_stream.getFirst().timestamp;
                        if (prev_samp_time[imu_samp.id] != 0) {
                            imu_data[imu_samp.id].add_psd_samp(imu_samp, prev_samp_time[imu_samp.id], timestamp);
                        } 
                        prev_samp_time[imu_samp.id] = timestamp;
                    }
                }
                data_stream.remove();
            }
            try {
                Thread.sleep(10);
            } 
            catch (Exception e) { return; }
        }

        return;
    }
    // Process any IMU data incoming from the driver
    public void process()
    {
        while (!data_stream.isEmpty()) {
            for (int i = 0; i < data_stream.getFirst().samples.size(); i++) {
                RAIGDriver2.IMUSample imu_samp = data_stream.getFirst().samples.get(i);
                if (imu_samp.id >= 0 && imu_samp.id < num_sensors) {
                    long timestamp = data_stream.getFirst().timestamp;
                    if (prev_samp_time[imu_samp.id] != 0) {
                        imu_data[imu_samp.id].add_samp(imu_samp, prev_samp_time[imu_samp.id], timestamp);
                    } 
                    prev_samp_time[imu_samp.id] = timestamp;
                }
            }
            data_stream.remove();
            try {
                Thread.sleep(10);
            } 
            catch (Exception e) { return; }
        }
        
    }

    // Pick how many sensors contribute to the fused data
    public void setActiveSensors(int num)
    {
        if (num > num_sensors) {
            active_sensors = num_sensors;
        } else if (num > 0) {
            active_sensors = num;
        } else {
            active_sensors = 1;
        }
    }

    // Returns total number of sensors
    public int getNumSensors()
    {
        return num_sensors;
    }

    // Return an array of XYZ 0-offsets for all sensors
    public double[][] getOffsets()
    {
        double[][] offsets = new double[num_sensors][3];
        for (int i = 0; i < num_sensors; i++) {
            offsets[i][0] = imu_data[i].xOffset();
            offsets[i][1] = imu_data[i].yOffset();
            offsets[i][2] = imu_data[i].zOffset();
        }
        return offsets;
    }
    
    // Return an array of XYZ PSDs for all sensors
    public double[][] getPSDs()
    {
        double[][] psds = new double[num_sensors][3];
        for (int i = 0; i < num_sensors; i++) {
            psds[i][0] = imu_data[i].xPSD();
            psds[i][1] = imu_data[i].yPSD();
            psds[i][2] = imu_data[i].zPSD();
        }
        return psds;
    }

    // Return an array of XYZ ARWs for all sensors
    public double[][] getARWs()
    {
        double[][] arws = new double[num_sensors][3];
        for (int i = 0; i < num_sensors; i++) {
            arws[i][0] = imu_data[i].xARW();
            arws[i][1] = imu_data[i].yARW();
            arws[i][2] = imu_data[i].zARW();
        }
        return arws;
    }

    // Return an array of XYZ headings for all sensors
    public double[][] getSensorHeadings()
    {
        double[][] headings = new double[num_sensors][3];
        for (int i = 0; i < num_sensors; i++) {
            headings[i][0] = imu_data[i].headX;
            headings[i][1] = imu_data[i].headY;
            headings[i][2] = imu_data[i].headZ;
        }
        return headings;
    }
    // Return an array of XYZ headings for all sensors
    public double[] getSensorZHeadings()
    {
        double[] headings = new double[num_sensors];
        for (int i = 0; i < num_sensors; i++) {
            headings[i] = imu_data[i].headZ;
        }
        return headings;
    }
    // Thread execution
    public void run()
    {
        if (!calibrated) {
            calibrate(500);
        }

        while (true) {
            process();


            try {
                Thread.sleep(10);
            } 
            catch (Exception e) { return; }
        }
    }
}
