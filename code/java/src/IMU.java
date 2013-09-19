import java.io.*;
import java.util.*;
import java.lang.*;
import java.util.Enumeration;
import java.util.Properties;

// This class interfaces with the RAIG driver to process
// data from each invididual sensor, and fuse that data together
public class IMU extends Thread
{
    // Constants
    static final int NUM_AXES = 3;

    // Control Variables
    volatile private boolean halt = false;

    // Averaged Sensor Information
    // Power Spectral Density values
    private double aNoiseSq[] = new double[NUM_AXES];
    private double a_psd_total_time = 0.0;
    // Total calculated heading of each axis (rad)
    private double aHead[] = new double[NUM_AXES];

    // EM Sensor Information
    // Power Spectral Density values
    private double waNoiseSq[] = new double[NUM_AXES];
    private double wa_psd_total_time = 0.0;
    // Calculated rate of each axis (rad/sec)
    private double waHead[] = new double[NUM_AXES];
    // Sensor weights
    private double waK[][];


    // This class calculates and stores data for a single sensor
    // Data is not automatically added to this class -
    // use add_cal_samp, add_samp, and add_psd_samp to put sensor
    // samples into the class
    public class IMUData
    {
        // Gyroscope Data
        // Conversion constants (Raw to radians per sec)
        private double kRate[] = new double[NUM_AXES];
        // 0-bias of the gyroscopes
        private double oRate[] = new double[NUM_AXES];
        // Rate of each axis (rad/sec)
        private double rate[] = new double[NUM_AXES];
        // Total calculated heading of each axis
        private double head[] = new double[NUM_AXES];
        // Calibration duration
        private double calib_total_time = 0;
        // Power Spectral Density values
        private double noiseSq[] = new double[NUM_AXES];
        // PSD Calibration duration
        private double psd_total_time = 0.0;
        // Most recent change in heading for this sensor
        private double delta[] = new double[NUM_AXES];
        // Most recent noise (deviation from bias value)
        // Used to calculate fused PSDs
        private double noise[] = new double[NUM_AXES];

        // Accelerometer Data
        // Conversion constants (Raw to m/s^2)
        private double kAccel[] = new double[NUM_AXES];
        // 0-bias of accelerometers
        private double oAccel[] = new double[NUM_AXES];
        // Acceleration (m/s^2)
        private double accel[] = new double[NUM_AXES];

        // Temperature Data
        private double temp = 0.0;
        // Temperature offset constants
        private double tOff[] = new double[NUM_AXES];
        // Temperature sensitivity constants
        private double tSen[] = new double[NUM_AXES];

        public IMUData(double kr[])
        {
            kRate = kr;
        }

        public IMUData(double kr[], double ka[], double oa[], double to[], double ts[])
        {
            kRate = kr;
            kAccel = ka;
            oAccel = oa;
            tOff = to;
            tSen = ts;
        }

        public void add_cal_samp(RAIGDriver.IMUSample samp, double time_diff)
        {
            // Ignore new sample if it came before previous sample
            if (time_diff < 0) {
                time_diff = 0;
            }
            for (int i = 0; i < NUM_AXES; i++) {
                oRate[i] += time_diff*samp.rate[i];
            }
            calib_total_time += time_diff;
        }
        public void add_samp(RAIGDriver.IMUSample samp, double time_diff)
        {
            // Ignore new sample if it came before previous sample
            if (time_diff < 0.0) {
                time_diff = 0.0;
            }

            for (int i = 0; i < NUM_AXES; i++) {
                // Update gyroscope heading
                //rate[i] = (samp.rate[i] - getOffset()[i])*kRate[i] + temp*tSen[i];
                rate[i] = (samp.rate[i] - tOff[i] - temp*tSen[i])*kRate[i];
                delta[i] = time_diff*rate[i];
                head[i] += delta[i];
                // Update acceleration
                accel[i] = (samp.accel[i] - getAccelOffset()[i])*kAccel[i];
            }

            // Update temperature
            temp = (double)samp.temp;
        }
        // Used to calculate the noise power spectral density in (rad/sqrt(sec))^2/Hz
        public void add_psd_samp(RAIGDriver.IMUSample samp, double time_diff)
        {
            // Ignore new sample if it came before previous sample
            if (time_diff < 0.0) {
                time_diff = 0.0;
            }
            for (int i = 0; i < NUM_AXES; i++) {
                noise[i] = (samp.rate[i] - getOffset()[i])*kRate[i];

                noiseSq[i] += time_diff*Math.pow(noise[i],2.0);
            }
            psd_total_time += time_diff;
        }

        // Return sensor 0-biases in units of raw sensor counts
        public double[] getOffset()
        {
            if (calib_total_time == 0) {
                return new double[NUM_AXES];
            } else {
                double result[] = new double[NUM_AXES];
                for (int i = 0; i < NUM_AXES; i++) {
                    result[i] = oRate[i]/calib_total_time + temp*tOff[i];
                }
                return result;
            }
        }
       
        // Return sensor 0-biases in units of raw sensor counts
        public double[] getAccelOffset()
        {
            return oAccel;
        }
        
        // Rate Noise
        public double[] getNoise()
        {
            return noise;
        }

        // Rate Noise PSD in (rad/sqrt(sec))^2/Hz
        public double[] getPSD()
        {
            if (psd_total_time == 0.0) {
                return new double[3];
            } else {
                double result[] = new double[NUM_AXES];
                for (int i = 0; i < NUM_AXES; i++) {
                    result[i] = noiseSq[i]/psd_total_time;
                }
                return result;
            }
        }

        // Headings in radians
        public double[] getHeading()
        {
            return head;
        }
        
        // Current gyroscope rate in rad/sec
        public double[] getRate()
        {
            return rate;
        }

        // Most recent change of heading in radians
        public double[] getDelta()
        {
            return delta;
        }
        
        // Acceleration in m/s^2
        public double[] getAccel()
        {
            return accel;
        }

        // Temperature value (in ???)
        public double getTemp()
        {
            return temp;
        }

        // Clear data so we can recalibrate etc.
        public void clearOffset()
        {
            oRate = new double[3];
            calib_total_time = 0;
        }
        public void clearPSD()
        {
            noiseSq = new double[3];
            psd_total_time = 0.0;
        }
        public void clearHeading()
        {
            head = new double[3];
        }

        // Set the heading to a specific value (if we can ground-truth heading etc.)
        // Requires 3 element array
        public void setHeading(double[] h)
        {
            head = h;
        }
    }

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
    public IMUData[] imu_data;
    private long[] prev_samp_time;
    private volatile LinkedList<RAIGDriver.IMUSamples> data_stream;
    private boolean calibrated;

    // Warning: this function will take approx. 4 seconds on first call (to establish port connection)
    public IMU(IMUType type)
    {
        String prefix = new String();

        switch (type)
        {
            case LSM330:
                prefix = "LSM330";
                data_stream = RAIGDriver.getSingleton().lsm_data;
                break;
            case MPU6050:
                prefix = "MPU6050";
                data_stream = RAIGDriver.getSingleton().mpu_data;
                break;
        }

        // Load values from properties file
        Properties conf = new Properties();
        try {
            conf.load(new FileInputStream("imu.conf"));
            // Load number of sensors
            num_sensors = Integer.parseInt(conf.getProperty(prefix + "_NUM_SENSORS"));
            
            imu_data = new IMUData[num_sensors];

            // Loop through sensors
            for (int i = 0; i < num_sensors; i++) {

                // Constants for current sensor
                double kr[] = new double[3];
                double ka[] = new double[3];
                double oa[] = new double[3];
                double to[] = new double[3];
                double ts[] = new double[3];
                
                // Loop through axes
                for (int j = 0; j < 3; j++) {
                    kr[j] = Double.parseDouble(conf.getProperty(prefix + "_" + i + "_" + j + "_KR"));
                    ka[j] = Double.parseDouble(conf.getProperty(prefix + "_" + i + "_" + j + "_KA"));
                    oa[j] = Double.parseDouble(conf.getProperty(prefix + "_" + i + "_" + j + "_OA"));
                    to[j] = Double.parseDouble(conf.getProperty(prefix + "_" + i + "_" + j + "_TO"));
                    ts[j] = Double.parseDouble(conf.getProperty(prefix + "_" + i + "_" + j + "_TS"));

                    //System.out.println("Constants" + i + j + ":");
                    //System.out.println(kr[j] + " " + ka[j] + " " + to[j] + " " + ts[j]);
                }

                // Create IMUData with constants
                imu_data[i] = new IMUData(kr, ka, oa, to, ts);
            }
        } catch (IOException ex) {
            ex.printStackTrace();
        }

        // Initialize state structures
        num_sensors = imu_data.length;
        prev_samp_time = new long[num_sensors];
        // Initialize weighted average weights
        waK = new double[num_sensors][NUM_AXES];
        for (int i = 0; i < num_sensors; i++) {
            for (int n = 0; n < NUM_AXES; n++) {
                waK[i][n] = 1.0/num_sensors;
            }
        }
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

        // Store timestamps from sensor messages
        long curr_time = 0;

        // Sum gyroscope samples
        while (System.currentTimeMillis() - start < calib_millis) {
            while (!data_stream.isEmpty()) {
                curr_time = data_stream.getFirst().timestamp;
                for (int i = 0; i < data_stream.getFirst().samples.size(); i++) {
                    RAIGDriver.IMUSample imu_samp = data_stream.getFirst().samples.get(i);
                    if (imu_samp.id >= 0 && imu_samp.id < num_sensors) {
                        if (prev_samp_time[imu_samp.id] != 0) {
                            imu_data[imu_samp.id].add_cal_samp(imu_samp, diffSecs(prev_samp_time[imu_samp.id], curr_time));
                        }
                        prev_samp_time[imu_samp.id] = curr_time;
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
                curr_time = data_stream.getFirst().timestamp;
                for (int i = 0; i < data_stream.getFirst().samples.size(); i++) {
                    RAIGDriver.IMUSample imu_samp = data_stream.getFirst().samples.get(i);
                    if (imu_samp.id >= 0 && imu_samp.id < num_sensors) {
                        if (prev_samp_time[imu_samp.id] != 0) {
                            imu_data[imu_samp.id].add_psd_samp(imu_samp, diffSecs(prev_samp_time[imu_samp.id], curr_time));
                        }
                        prev_samp_time[imu_samp.id] = curr_time;
                    }
                }

                // Calculate fused sensor PSD
                if (prev_time != 0) {
                    double time_diff = diffSecs(prev_time, curr_time);
                    
                    for (int i = 0; i < NUM_AXES; i++) {
                        double aNoise = 0.0;

                        for (int n = 0; n < num_sensors; n++) {
                            aNoise += imu_data[n].getNoise()[i]/num_sensors;
                        }

                        aNoiseSq[i] += time_diff*Math.pow(aNoise,2.0);
                    }
                    a_psd_total_time += time_diff;
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
        halt = false;
        
        // Store timestamps from sensor messages
        long curr_time = 0;

        while (!halt) {
            while (!data_stream.isEmpty() && !halt) {
                curr_time = data_stream.getFirst().timestamp;
                for (int i = 0; i < data_stream.getFirst().samples.size(); i++) {
                    RAIGDriver.IMUSample imu_samp = data_stream.getFirst().samples.get(i);
                    if (imu_samp.id >= 0 && imu_samp.id < active_sensors) {
                        if (prev_samp_time[imu_samp.id] != 0) {
                            imu_data[imu_samp.id].add_samp(imu_samp, diffSecs(prev_samp_time[imu_samp.id], curr_time));
                            for (int n = 0; n < NUM_AXES; n++) {
                                aHead[n] += imu_data[i].getDelta()[n]/active_sensors;
                                waHead[n] += waK[i][n]*imu_data[i].getDelta()[n];
                            }

          
                        }
                        prev_samp_time[imu_samp.id] = curr_time;
                    }
                    // Recalculate and normalize weights
                    double delta_sum[] = new double[NUM_AXES];
                    for (int k = 0; k < active_sensors; k++) {
                        for (int n = 0; n < NUM_AXES; n++) {
                            delta_sum[n] += Math.abs(aHead[n] - imu_data[k].getHeading()[n]);
                        }
                    }
                    for (int k = 0; k < active_sensors; k++) {
                        for (int n = 0; n < NUM_AXES; n++) {
                            waK[k][n] = delta_sum[n]/(Math.abs(aHead[n] - imu_data[k].getHeading()[n]));
                        }
                    }
                    double weight_sum[] = new double[NUM_AXES];
                    for (int k = 0; k < active_sensors; k++) {
                        for (int n = 0; n < NUM_AXES; n++) {
                            weight_sum[n] += waK[k][n];
                        }
                    }
                    for (int k = 0; k < active_sensors; k++) {
                        for (int n = 0; n < NUM_AXES; n++) {
                            waK[k][n] /= weight_sum[n];
                        }
                        // Debug weights
                        //System.out.println("Sensor" + k + ": " + waK[k][2]);
                    }
                }
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
    
    // Returns number of sensors contributing to fused data
    public int getActiveSensors(int num)
    {
        return active_sensors; 
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
        double[][] offsets = new double[num_sensors][NUM_AXES];
        for (int i = 0; i < num_sensors; i++) {
            offsets[i] = imu_data[i].getOffset();
        }
        return offsets;
    }
    
    // Return an array of XYZ PSDs for all sensors
    public double[][] getPSDs()
    {
        double[][] psds = new double[num_sensors][NUM_AXES];
        for (int i = 0; i < num_sensors; i++) {
            psds[i] = imu_data[i].getPSD();
        }
        return psds;
    }

    // Return an array of XYZ headings for all sensors
    public double[][] getSensorHeadings()
    {
        double[][] headings = new double[num_sensors][NUM_AXES];
        for (int i = 0; i < num_sensors; i++) {
            headings[i] = imu_data[i].getHeading();
        }
        return headings;
    } 

    // Return Temperature of all sensors
    public double[] getTemps()
    {
        double[] temps = new double[num_sensors];
        for (int i = 0; i < num_sensors; i++) {
            temps[i] = imu_data[i].getTemp();
        }
        return temps;
    }

    //
    //
    // Getters for fused sensor data
    // Methods return single values or 3-element arrays
    //
    //

    // Return an array of averaged XYZ PSDs
    public double[] getAveragePSDs()
    {
        double[] psds = new double[NUM_AXES];
        for (int i = 0; i < NUM_AXES; i++) {
            psds[i] = aNoiseSq[i] / a_psd_total_time;
        }
        return psds;
    }
    
    // Return an array of average XYZ headings
    public double[] getAverageHeadings()
    {
        double[] headings = new double[NUM_AXES];
        for (int i = 0; i < NUM_AXES; i++) {
            headings[i] = aHead[i];
        }
        return headings;
    }
    
    // Return an array of averaged XYZ rates
    public double[] getAverageRates()
    {
        double[] rates = new double[NUM_AXES];
        for (int i = 0; i < active_sensors; i++) {
            for (int n = 0; n < NUM_AXES; n++) {
                rates[n] += imu_data[i].getRate()[n]/active_sensors;
            }
        }
        return rates;
    }

    // Return an array of averaged XYZ accelerations
    public double[] getAverageAccels()
    {
        double[] accels = new double[NUM_AXES];
        for (int i = 0; i < active_sensors; i++) {
            for (int n = 0; n < NUM_AXES; n++) {
                accels[n] += imu_data[i].getAccel()[n]/active_sensors;
            }
        }
        return accels;
    }

    // Use to ground truth/reset calculated headings
    public void clearHeadings()
    {
        aHead = new double[NUM_AXES];
        waHead = new double[NUM_AXES];
    }
    public void setHeadings(double[] a)
    {
        aHead = a;
        waHead = a;
    }
    
    // Return the best XYZ headings of any single sensor (rel. to average)
    public double[] getBestHeadings()
    {
        double[] headings = new double[NUM_AXES];
        for (int i = 0; i < NUM_AXES; i++) {
            double lowestDiff = Math.abs(getAverageHeadings()[i] - imu_data[0].getHeading()[i]);
            headings[i] = imu_data[0].getHeading()[i];
            for (int j = 0; j < active_sensors; j++) {
                if (Math.abs(getAverageHeadings()[i] - imu_data[j].getHeading()[i]) < lowestDiff) {
                    lowestDiff = Math.abs(getAverageHeadings()[i] - imu_data[j].getHeading()[i]);                
                    headings[i] = imu_data[j].getHeading()[i];
                }
            }
        }
        return headings;
    }
    
    // Return an array of weighted average XYZ headings
    public double[] getWAverageHeadings()
    {
        double[] headings = new double[NUM_AXES];
        for (int i = 0; i < NUM_AXES; i++) {
            headings[i] = waHead[i];
        }
        return headings;
    }

    //
    //
    // Miscellaneous utility methods
    //
    //

    // Convert a power spectral density to an angle random walk
    static public double toARW(double psd)  
    {
        return (1.0/60.0)*Math.sqrt(psd);
    }
    // Convert an array of power spectral densities to angle random walks
    static public double[] toARW(double[] psds)
    {
        double[] arws = new double[psds.length];
        for (int i = 0; i < psds.length; i++) {
            arws[i] = (1.0/60.0)*Math.sqrt(psds[i]);
        }
        return arws;
    }
    // Convert a power spectral density to a rate random walk
    static public double toRRW(double psd)
    {
        return Math.sqrt(psd);

    }
    // Convert an array of power spectral densities to rate random walks
    static public double[] toRRW(double[] psds)
    {
        double[] rrws = new double[psds.length];
        for (int i = 0; i < psds.length; i++) {
            rrws[i] = Math.sqrt(psds[i]);
        }
        return rrws;
    }
    // Convert radians to degrees
    static public double toDegrees(double rad)
    {
        return rad*(180.0/Math.PI);
    }
    static public double[] toDegrees(double[] rads)
    {
        double[] degs = new double[rads.length];
        for (int i = 0; i < rads.length; i++) {
            degs[i] = rads[i]*(180.0/Math.PI);
        }
        return degs;
    }

    //
    //
    // IMU Thread Control
    //
    //

    // Signal threads to stop running
    public void exit() 
    {
        halt = true;

    }

    // Check if running
    public boolean isRunning()
    {
        return !halt;
    }

    //
    //
    // IMU Helper Functions
    //
    //

    // Takes 2 'long' times in milliseconds
    // Arguments should be in forware chronological order
    // Returns their difference in seconds
    private double diffSecs(long first, long second)
    {
        return ((double)(second - first))/(1000.0);
    }
}
