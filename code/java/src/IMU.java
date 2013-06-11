import java.io.*;
import java.util.*;
import java.lang.*;
import java.util.Enumeration;

// This class interfaces with the RAIG driver to process
// data from each invididual sensor, and fuse that data together
public class IMU implements Runnable
{
    // Singletons
    private static IMU lsmIMU = null;
    private static IMU mpuIMU = null;

    // Fused Sensor Information
    // Power Spectral Density values
    private double fNoiseSqX = 0.0;
    private double fNoiseSqY = 0.0;
    private double fNoiseSqZ = 0.0;
    private double f_psd_total_time = 0.0;
    // Total calculated heading of each axis
    private double fHeadX = 0.0;
    private double fHeadY = 0.0;
    private double fHeadZ = 0.0;

    // This class calculates and stores data for a single sensor
    // Data is not automatically added to this class -
    // use add_cal_samp, add_samp, and add_psd_samp to put sensor
    // samples into the class
    private class IMUData
    {
        // Conversion constants (Raw to radians per hour)
        private double kRateX = 0.0;
        private double kRateY = 0.0;
        private double kRateZ = 0.0;
        // 0-bias of the gyroscopes
        private double oRateX = 0.0;
        private double oRateY = 0.0;
        private double oRateZ = 0.0;
        // Total calculated heading of each axis
        private double headX = 0.0;
        private double headY = 0.0;
        private double headZ = 0.0;
        // Calibration duration
        private long calib_total_time = 0;
        // Power Spectral Density values
        private double noiseSqX = 0.0;
        private double noiseSqY = 0.0;
        private double noiseSqZ = 0.0;
        private double psd_total_time = 0.0;
        // Most recent change in heading for this sensor
        private double deltaX = 0.0;
        private double deltaY = 0.0;
        private double deltaZ = 0.0;
        // Most recent noise (deviation from bias value)
        // Used to calculate fused PSDs
        private double noiseX = 0.0;
        private double noiseY = 0.0;
        private double noiseZ = 0.0;

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
        public void add_samp(RAIGDriver2.IMUSample samp, long prev_time, long curr_time)
        {
            double time_diff = ((double)(curr_time - prev_time))/(1000.0*60.0*60.0);
            // Ignore new sample if it came before previous sample
            if (time_diff < 0.0) {
                time_diff = 0.0;
            }
            deltaX = (samp.rateX - getOffsetX())*kRateX;
            deltaY = (samp.rateY - getOffsetY())*kRateY;
            deltaZ = (samp.rateZ - getOffsetZ())*kRateZ;

            headX += time_diff*deltaX;
            headY += time_diff*deltaY;
            headZ += time_diff*deltaZ;
        }
        // Used to calculate the noise power spectral density in (rad/sqrt(hr))^2/Hz
        public void add_psd_samp(RAIGDriver2.IMUSample samp, long prev_time, long curr_time)
        {
            double time_diff = ((double)(curr_time - prev_time))/(1000.0*60.0*60.0);
            // Ignore new sample if it came before previous sample
            if (time_diff < 0.0) {
                time_diff = 0.0;
            }
            noiseX = (samp.rateX - getOffsetX())*kRateX;
            noiseY = (samp.rateY - getOffsetY())*kRateY;
            noiseZ = (samp.rateZ - getOffsetZ())*kRateZ;

            noiseSqX += time_diff*Math.pow(noiseX,2.0);
            noiseSqY += time_diff*Math.pow(noiseY,2.0);
            noiseSqZ += time_diff*Math.pow(noiseZ,2.0);
            psd_total_time += time_diff;
        }

        // Return sensor 0-biases in units of raw sensor counts
        public double getOffsetX()
        {
            if (calib_total_time == 0) {
                return 0.0;
            } else {
                return oRateX / (double)calib_total_time;
            }
        }
        public double getOffsetY()
        {
            if (calib_total_time == 0) {
                return 0.0;
            } else {
                return oRateY / (double)calib_total_time;
            }
        }
        public double getOffsetZ()
        {
            if (calib_total_time == 0) {
                return 0.0;
            } else {
                return oRateZ / (double)calib_total_time;
            }
        }
        
        // Rate Noise
        public double getNoiseX()
        {
            return noiseX;
        }
        public double getNoiseY()
        {
            return noiseY;
        }
        public double getNoiseZ()
        {
            return noiseZ;
        }

        // Rate Noise PSD in (rad/sqrt(hr))^2/Hz
        public double getPSDX()
        {
            if (psd_total_time == 0.0) {
                return 0.0;
            } else {
                return noiseSqX / psd_total_time;
            }
        }
        public double getPSDY()
        {
            if (psd_total_time == 0.0) {
                return 0.0;
            } else {
                return noiseSqY / psd_total_time;
            }
        }
        public double getPSDZ()
        {
            if (psd_total_time == 0.0) {
                return 0.0;
            } else {
                return noiseSqZ / psd_total_time;
            }
        }

        // Headings in radians
        public double getHeadingX()
        {
            return headX;
        }
        public double getHeadingY()
        {
            return headY;
        }
        public double getHeadingZ()
        {
            return headZ;
        }
        
        // Most recent change of heading in radians
        public double getDeltaX()
        {
            return deltaX;
        }
        public double getDeltaY()
        {
            return deltaY;
        }
        public double getDeltaZ()
        {
            return deltaZ;
        }

        // Clear data so we can recalibrate etc.
        public void clearOffset()
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
        public void clearHeading()
        {
            headX = 0.0;
            headY = 0.0;
            headZ = 0.0;
        }

        // Set the heading to a specific value (if we can ground-truth heading etc.)
        public void setHeading(double hx, double hy, double hz)
        {
            headX = hx;
            headY = hy;
            headZ = hz;
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
            imu_data[i].clearOffset();
        }

        long start = System.currentTimeMillis();

        // Sum gyroscope samples
        while (System.currentTimeMillis() - start < calib_millis) {
            while (!data_stream.isEmpty()) {
                long timestamp = data_stream.getFirst().timestamp;
                for (int i = 0; i < data_stream.getFirst().samples.size(); i++) {
                    RAIGDriver2.IMUSample imu_samp = data_stream.getFirst().samples.get(i);
                    if (imu_samp.id >= 0 && imu_samp.id < num_sensors) {
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

        // Store timestamps from sensor messages
        long curr_time = 0;
        long prev_time = 0;

        // Send gyroscope samples to respective IMU object for processing
        while (System.currentTimeMillis() - start < psd_millis) {
            while (!data_stream.isEmpty()) {
                curr_time = data_stream.getFirst().curr_time;
                for (int i = 0; i < data_stream.getFirst().samples.size(); i++) {
                    RAIGDriver2.IMUSample imu_samp = data_stream.getFirst().samples.get(i);
                    if (imu_samp.id >= 0 && imu_samp.id < num_sensors) {
                        if (prev_samp_time[imu_samp.id] != 0) {
                            imu_data[imu_samp.id].add_psd_samp(imu_samp, prev_samp_time[imu_samp.id], curr_time);
                        }
                        prev_samp_time[imu_samp.id] = curr_time;
                    }
                }

                // Calculate fused sensor PSD
                if (time_prev != 0) {
                    double fNoiseX = 0.0;
                    double fNoiseY = 0.0;
                    double fNoiseZ = 0.0;

                    for (int i = 0; i < num_sensors; i++) {
                        fNoiseX += imu_data[i].getNoiseX();
                        fNoiseY += imu_data[i].getNoiseY();
                        fNoiseZ += imu_data[i].getNoiseZ();
                    }

                    fNoiseX /= num_sensors;
                    fNoiseY /= num_sensors;
                    fNoiseZ /= num_sensors;

                    double time_diff = ((double)(curr_time - prev_time))/(1000.0*60.0*60.0);
                    fNoiseSqX += time_diff*Math.pow(fNoiseX,2.0);
                    fNoiseSqY += time_diff*Math.pow(fNoiseY,2.0);
                    fNoiseSqZ += time_diff*Math.pow(fNoiseZ,2.0);
                    f_psd_total_time += time_diff;
                }

                prev_time = curr_time;
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
    public void run()
    {
        if (!calibrated) {
            calibrate(500);
        }
        
        // Store timestamps from sensor messages
        long curr_time = 0;
        long prev_time = 0;

        while (true) {
            while (!data_stream.isEmpty()) {
                long timestamp = data_stream.getFirst().timestamp;
                for (int i = 0; i < data_stream.getFirst().samples.size(); i++) {
                    RAIGDriver2.IMUSample imu_samp = data_stream.getFirst().samples.get(i);
                    if (imu_samp.id >= 0 && imu_samp.id < num_sensors) {
                        if (prev_samp_time[imu_samp.id] != 0) {
                            imu_data[imu_samp.id].add_samp(imu_samp, prev_samp_time[imu_samp.id], timestamp);
                        }
                        prev_samp_time[imu_samp.id] = timestamp;
                    }
                }

                // Calculate fused sensor heading
                if (time_prev != 0) {
                    double time_diff = ((double)(curr_time - prev_time))/(1000.0*60.0*60.0);
                    for (int i = 0; i < num_sensors; i++) {
                        fHeadX += time_diff*imu_data[i].getDeltaX()/num_sensors;
                        fHeadY += time_diff*imu_data[i].getDeltaY()/num_sensors;
                        fHeadZ += time_diff*imu_data[i].getDeltaZ()/num_sensors;
                    }
                }

                prev_time = curr_time;
                data_stream.remove();
            }
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

    //
    //
    // Getters for data from individual sensors
    // Methods return arrays of data
    //
    //

    // Return an array of XYZ 0-offsets for all sensors
    public double[][] getOffsets()
    {
        double[][] offsets = new double[num_sensors][3];
        for (int i = 0; i < num_sensors; i++) {
            offsets[i][0] = imu_data[i].getOffsetX();
            offsets[i][1] = imu_data[i].getOffsetY();
            offsets[i][2] = imu_data[i].getOffsetZ();
        }
        return offsets;
    }
    // Return an array of 1 axis of 0-offsets for all sensors
    public double[] getOffsetsX()
    {
        double[] offsets = new double[num_sensors];
        for (int i = 0; i < num_sensors; i++) {
            offsets[i] = imu_data[i].getOffsetX();
        }
        return offsets;
    }
    public double[] getOffsetsY()
    {
        double[] offsets = new double[num_sensors];
        for (int i = 0; i < num_sensors; i++) {
            offsets[i] = imu_data[i].getOffsetY();
        }
        return offsets;
    }
    public double[] getOffsetsZ()
    {
        double[] offsets = new double[num_sensors];
        for (int i = 0; i < num_sensors; i++) {
            offsets[i] = imu_data[i].getOffsetZ();
        }
        return offsets;
    }
    
    // Return an array of XYZ PSDs for all sensors
    public double[][] getPSDs()
    {
        double[][] psds = new double[num_sensors][3];
        for (int i = 0; i < num_sensors; i++) {
            psds[i][0] = imu_data[i].getPSDX();
            psds[i][1] = imu_data[i].getPSDY();
            psds[i][2] = imu_data[i].getPSDZ();
        }
        return psds;
    }
    // Return an array of 1 axis of PSDs for all sensors
    public double[] getPSDsX()
    {
        double[] psds = new double[num_sensors];
        for (int i = 0; i < num_sensors; i++) {
            psds[i] = imu_data[i].getPSDX();
        }
        return psds;
    }
    public double[] getPSDsY()
    {
        double[] psds = new double[num_sensors];
        for (int i = 0; i < num_sensors; i++) {
            psds[i] = imu_data[i].getPSDY();
        }
        return psds;
    }
    public double[] getPSDsZ()
    {
        double[] psds = new double[num_sensors];
        for (int i = 0; i < num_sensors; i++) {
            psds[i] = imu_data[i].getPSDZ();
        }
        return psds;
    }

    // Return an array of XYZ headings for all sensors
    public double[][] getSensorHeadings()
    {
        double[][] headings = new double[num_sensors][3];
        for (int i = 0; i < num_sensors; i++) {
            headings[i][0] = imu_data[i].getHeadingX();
            headings[i][1] = imu_data[i].getHeadingY();
            headings[i][2] = imu_data[i].getHeadingZ();
        }
        return headings;
    }
    // Returns array of 1 axis of headings for all sensors
    public double[] getSensorHeadingsX()
    {
        double[] headings = new double[num_sensors];
        for (int i = 0; i < num_sensors; i++) {
            headings[i] = imu_data[i].getHeadingX();
        }
        return headings;
    }
    public double[] getSensorHeadingsY()
    {
        double[] headings = new double[num_sensors];
        for (int i = 0; i < num_sensors; i++) {
            headings[i] = imu_data[i].getHeadingY();
        }
        return headings;
    }
    public double[] getSensorHeadingsZ()
    {
        double[] headings = new double[num_sensors];
        for (int i = 0; i < num_sensors; i++) {
            headings[i] = imu_data[i].getHeadingZ();
        }
        return headings;
    }

    //
    //
    // Getters for fused sensor data
    // Methods return single values or 3-element arrays
    //
    //

    // Return an array of fused XYZ PSDs
    public double[][] getPSDs()
    {
        double[] psds = new double[3];
        psds[0] = fNoiseSqX / f_psd_total_time;
        psds[1] = fNoiseSqY / f_psd_total_time;
        psds[2] = fNoiseSqZ / f_psd_total_time;
        return psds;
    }
    // Return 1 axis of fused PSDs
    public double getPSDX()
    {
        return fNoiseSqX / f_psd_total_time;
    }
    public double getPSDY()
    {
        return fNoiseSqY / f_psd_total_time;
    }
    public double getPSDZ()
    {
        return fNoiseSqZ / f_psd_total_time;
    }
    
    // Return an array of fused XYZ headings
    public double[] getFusedHeadings()
    {
        double[] headings = new double[3];
        headings[0] = fHeadX;
        headings[1] = fHeadY;
        headings[2] = fHeadZ;
        return headings;
    }
    // Returns 1 axis of fused headings
    public double getSensorHeadingX()
    {
        return fHeadX;
    }
    public double getFusedHeadingY()
    {
        return fHeadY;
    }
    public double getFusedHeadingZ()
    {
        return fHeadZ;
    }

    // Use to ground truth/reset calculated headings
    public void clearFusedHeadings()
    {
        fHeadX = 0.0;
        fHeadY = 0.0;
        fHeadZ = 0.0;
    }
    public void setFusedHeadings(double fx, double fy, double fz)
    {
        fHeadX = fx;
        fHeadY = fy;
        fHeadZ = fz;
    }

    //
    //
    // Miscellaneous utility methods
    //
    //

    // Convert a power spectral density to an angle random walk
    public double toARW(double psd)  
    {
        return (1.0/60.0)*Math.sqrt(psd);
    }
    // Convert an array of power spectral densities to angle random walks
    public double[] toARW(double[] psds)
    {
        double[] arws = new double[psds.size];
        for (int i = 0; i < psds.length; i++) {
            arws[i] = (1.0/60.0)*Math.sqrt(psds[i]);
        }
        return arws;
    }
    // Convert a power spectral density to a rate random walk
    public double toRRW(double psd)
    {
        return Math.sqrt(psd);

    }
    // Convert an array of power spectral densities to rate random walks
    public double[] toRRW(double[] psds)
    {
        double[] rrws = new double[psds.size];
        for (int i = 0; i < psds.length; i++) {
            rrws[i] = Math.sqrt(psds[i]);
        }
        return rrws;
    }
}
