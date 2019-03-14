import sys, os
sys.path.append('/media/jawa/R2D2/dev/flight-mechanics-lab/Sandbox/wind_measuring/speed_to_attitude_v4')

from MeasDef import *


def add_data (obj, fullpath):

    with open(fullpath + "/data_att_c.csv") as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            #print row
            if line_count == 0:
                line_count += 1
            else:
                tmp = np.array([[
                    float(row[0]), float(row[1]),
                    float(row[2]), float(row[3])]])
                line_count += 1
                obj.add_row_att(tmp)

    with open(fullpath + "/456c.csv") as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            #print row
            if line_count == 0:
                line_count += 1
            else:
                tmp = np.array([[
                    float(row[0]), float(row[1]),
                    float(row[2]), float(row[3])]])
                line_count += 1
                obj.add_row_gps(tmp)

    return obj


def main():

    pathname = os.path.dirname(sys.argv[0])
    fullpath = os.path.abspath(pathname)

    meas_obj = Meas(fullpath, 200.0)
    meas_obj = add_data(meas_obj, fullpath)
    meas_obj.save_meas()

    # Get rid of object reference, so memory can be (automatically) freed
    del meas_obj


if __name__ == "__main__":
    main()
