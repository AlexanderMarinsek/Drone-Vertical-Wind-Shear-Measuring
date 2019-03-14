import numpy as np
import csv
from math import degrees


# Measurement object used for storing data while in flight and later moving
# data to .csv file. Measurement data consists of vehicle attitude (pitch,
# roll, yaw) and vehicle velocity (vx, vy, vz - NEU)
class Meas:


    # Initiate internal variables
    def __init__(self, filepath, msg_period_ms):
        self._msg_period_ms = msg_period_ms
        self._filepath = filepath
        self._labels_att = np.array([['time_boot_ms', 'pitch', 'roll', 'yaw']])
        self._labels_gps = np.array([['time_boot_ms', 'vx', 'vy', 'vz', 'relative_alt']])
        self._labels_combined = np.array(
            [['time_boot_ms', 'pitch', 'roll', 'yaw', 'vx', 'vy', 'vz', 'relative_alt']])
        self._meas_gps = np.zeros((1,5), dtype=float)
        self._meas_att = np.zeros((1,4), dtype=float)


    # Callback method for new GLOBAL_POSITION_INT messages
    def msg_gps_handler (self, selff, name, msg):
        #row = np.array(
        #    [[msg.time_boot_ms, msg.vx * 0.01, msg.vy * 0.01, msg.vz * 0.01]])
        row = np.array(
            [[msg.time_boot_ms, msg.vx * 0.01, msg.vy * 0.01, msg.vz * 0.01,
            msg.relative_alt * 0.001]])
        #print msg
        self.add_row_gps(row)


    # Callback method for new ATTITUDE messages
    def msg_att_handler (self, selff, name, msg):
        row = np.array(
            [[msg.time_boot_ms, degrees(msg.pitch), degrees(msg.roll),
            degrees(msg.yaw)]])
        #print msg
        self.add_row_att(row)


    # Add row to attitude measurement array
    def add_row_att (self, row):
        #print "add_row_att, ", row
        if row.shape == (1,4):
            self._meas_att = np.concatenate((self._meas_att, row))
        else:
            print "# add_row_att, incorrect dimensions"


    # Add row to gps (speed) measurement array
    def add_row_gps (self, row):
        #print "add_row_gps, ", row
        if row.shape == (1,5):
            self._meas_gps = np.concatenate((self._meas_gps, row))
        else:
            print "# add_row_gps, incorrect dimensions"


    # Save measurement data to .csv file
    def save_meas (self):

        # Check if measurement arrays contain data and delete zero-row
        if (self._meas_gps.shape[0] > 1):
            self._meas_gps = self._meas_gps[1:,:]
            self._meas_att = self._meas_att[1:,:]
        else:
            print "# save_meas: measurement arrays empty"
            return

        # Save attitude data
        filename_att = self._filepath + "/data_raw_att.csv"

        with open(filename_att, mode='w') as meas_file:
            meas_writer = csv.writer (meas_file, delimiter=',')

            # Make sure '[]' isn't written
            for row in self._labels_att:
                meas_writer.writerow(row)

            # Iterate rows
            for row in self._meas_att:
                meas_writer.writerow(row)


        # Save gps data
        filename_gps = self._filepath + "/data_raw_gps.csv"

        with open(filename_gps, mode='w') as meas_file:
            meas_writer = csv.writer (meas_file, delimiter=',')

            # Make sure '[]' isn't written
            for row in self._labels_gps:
                meas_writer.writerow(row)

            # Iterate rows
            for row in self._meas_gps:
                meas_writer.writerow(row)


        # Combine gps and attitude into one file after comparing timestamps
        meas_combined = self.combine_measurements(
            self._meas_gps, self._meas_att)

        filename_comb = self._filepath + "/data_raw_comb.csv"

        with open(filename_comb, mode='w') as meas_file:
            meas_writer = csv.writer (meas_file, delimiter=',')

            # Make sure '[]' isn't written
            for row in self._labels_combined:
                meas_writer.writerow(row)

            # Iterate rows
            for row in meas_combined:
                meas_writer.writerow(row)


    # Combine attitude and gps measurement sets by comparing timestamps
    def combine_measurements (self, meas_gps, meas_att):

        # Get rid of innitial shift between both measurement arrays
        # Shift gps array
        if (meas_att[0,0] > meas_gps[0,0]):
            print 1
            [meas_gps, meas_att] = self.shift_meas(meas_gps, meas_att)
        # Shift attitude array
        elif (meas_att[0,0] < meas_gps[0,0]):
            [meas_att, meas_gps] = self.shift_meas(meas_att, meas_gps)

        num_of_rows_att = meas_att.shape[0]
        num_of_rows_gps = meas_gps.shape[0]

        # Cut away any redundant measurements at the end
        if num_of_rows_att > num_of_rows_gps:
            meas_att = meas_att[:num_of_rows_gps,:]
        elif num_of_rows_att < num_of_rows_gps:
            meas_gps = meas_gps[:num_of_rows_att,:]

        # Get rid of timestamp
        meas_gps = meas_gps[:,1:]

        # Concatenate (add columns)
        return np.concatenate((meas_att, meas_gps), axis=1)


    # Determine, if measurement arrays need shifting and shift them
    def shift_meas (self, shifted_arr, reference_arr):

        # Check, if time delay is less than self._msg_period_ms
        if not abs(shifted_arr[0,0] - reference_arr[0,0]) <= self._msg_period_ms/2:

            # Shift array, till requirements are met
            for i, row in enumerate(shifted_arr):

                if (abs(row[0] - reference_arr[0,0])) < self._msg_period_ms/2:

                    shifted_arr = shifted_arr[i:,:]
                    return [shifted_arr, reference_arr]

            print "# shift_meas: Didn't find timestamp"
            return

        # Time delay less than self._msg_period_ms, no trimming needed
        return [shifted_arr, reference_arr]
