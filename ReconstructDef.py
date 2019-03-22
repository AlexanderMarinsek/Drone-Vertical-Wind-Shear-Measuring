import sys, os
import csv

from math import sqrt, degrees, radians, sin, cos, tan, asin, acos, atan, atan2

import matplotlib.pyplot as plt

import numpy as np


class Reconstruct:


    def __init__ (self, filepath):
        self._filepath = filepath
        self._wind_labels_row = \
            ['Relative altitude [m]', 'Wind speed [m/s]', 'Wind direction [deg]']
        self.full_filename_raw = filepath + '/data_raw_comb.csv'
        self.full_filename_wind = filepath + '/data_wind.csv'


    # Open raw data file, transform to wind parameters and save to new file
    def transform(self):
        f1 = file(self.full_filename_raw, 'r')
        f2 = file(self.full_filename_wind, 'w')

        c1 = csv.reader(f1, delimiter=',')
        c2 = csv.writer(f2, delimiter=',')

        c2.writerow(self._wind_labels_row)

        rows = list(c1)
        row_prev = None

        row_num = 0

        for row in rows:
            # Filter out double rows (happens when freq. demands are not met)
            if row_num > 0 and not row == row_prev:
                [altitude, wind_speed, wind_dir] = self.calc_wind(
                    float(row[1]), float(row[2]), float(row[3]), float(row[4]),
                    float(row[5]), float(row[6]), float(row[7]))
                c2.writerow([altitude, wind_speed, wind_dir])
            row_prev = row
            row_num += 1

        f1.close()
        f2.close()


    # From attitude and position, get wind speed and direction
    def calc_wind(self, pitch, roll, yaw, vx, vy, vz, alt):
        alpha = degrees(acos(cos(radians(pitch))*cos(radians(roll))))
        tan_alpha = tan((acos(cos(radians(pitch))*cos(radians(roll)))))

        gs_abs = sqrt(vx**2 + vy**2)
        gs_dir = degrees(atan2(vy, vx))     # (y, x) - oposite of Libre Calc

        as_abs = self.calc_airspeed(tan_alpha)
        as_dir_N = -sin(radians(roll)) * cos(radians(pitch))
        as_dir_D = cos(radians(roll)) * sin(radians(pitch))
        as_dir = degrees(atan(as_dir_N / as_dir_D)) + yaw

        ws_re = gs_abs*cos(radians(gs_dir)) + as_abs*cos(radians(as_dir))
        ws_im = gs_abs*sin(radians(gs_dir)) + as_abs*sin(radians(as_dir))

        ws_abs = sqrt(ws_re**2 + ws_im**2)
        ws_dir = degrees(atan2(ws_im, ws_re))

        return [alt, ws_abs, ws_dir]


    # Analytical function connecting airspeed with attitude
    def calc_airspeed(self, tan_alpha):
        #return sqrt(tan_alpha**2 * 225)
        return tan_alpha * 15


    # Draw vertical wind shear (wind gradient profile)
    def draw_wind(self):

        with open(self.full_filename_wind, 'r') as csv_file:

            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count = 0

            for row in csv_reader:

                if line_count == 0:
                    data_labels = np.array(
                        [[row[0], row[1], row[2]]])
                elif line_count == 1:
                    data = np.array(
                        [[float(row[0]), float(row[1]), float(row[2])]])
                elif line_count > 1:
                    data_row = np.array(
                        [[float(row[0]), float(row[1]), float(row[2])]])
                    data = np.concatenate((data, data_row))

                line_count += 1


        padd_left = 0.1
        padd_right = 0.85
        padd_bottom = 0.1
        padd_top = 0.93

        fig = plt.figure();
        plt.subplots_adjust(left = padd_left, bottom = padd_bottom,
            right = padd_right, top = padd_top, wspace = None, hspace = None)

        ax1 = fig.add_subplot(111)
        ax2 = ax1.twinx()

        lns1 = ax1.plot(data[1:,0], data[1:,1], color = "#006bb3",
            label = "Wind speed")

        ax1.set(xlabel=data_labels[0,0], ylabel=data_labels[0,1],
            title="Vertical wind shear")

        ax1.grid()

        lns2 = ax2.plot(data[1:,0], data[1:,2], color = "#913D88",
            label = "Wind direction")

        ax2.set(ylabel=data_labels[0,2])

        lns = lns1 + lns2
        labs = [l.get_label() for l in lns]
        ax1.legend(lns, labs, loc = "lower right")

        fig.savefig(self._filepath + '/vertical_wind_shear.png')
        plt.show()



# For testing purposes ---------------------------------------------------------
def main():

    pathname = os.path.dirname(sys.argv[0])
    fullpath = os.path.abspath(pathname)

    rec_obj = Reconstruct (fullpath)
    rec_obj.transform()
    rec_obj.draw_wind()

    # Get rid of object reference, so memory can be (automatically) freed
    del rec_obj


if __name__ == "__main__":
    main()
