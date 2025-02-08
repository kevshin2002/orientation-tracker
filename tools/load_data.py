import pickle
import sys
import time 
import csv
import numpy as np

def tic():
  return time.time()
def toc(tstart, nm=""):
  print('%s took: %s sec.\n' % (nm,(time.time() - tstart)))

def read_data(fname):
  d = []
  with open(fname, 'rb') as f:
    if sys.version_info[0] < 3:
      d = pickle.load(f)
    else:
      d = pickle.load(f, encoding='latin1')  # needed for python 3
  return d


def create_vicon_csv(data, theStep):
    # Assuming data is a dictionary with 'rots' as a 3D numpy array
    rots = data['rots']
    ts = data['ts']
    num_rows, num_cols, num_time_steps = rots.shape

    with open('viconRot' + theStep + '.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        
        # Create the header row
        header = ['Time'] + [f'R{i}{j}' for i in range(num_rows) for j in range(num_cols)]
        writer.writerow(header)
        
        # Write the data
        for time_step in range(num_time_steps):
            row_data = [ts[0, time_step]]
            for i in range(num_rows):
                for j in range(num_cols):
                    row_data.append(rots[i, j, time_step])
            writer.writerow(row_data)
    
    print("Transposed CSV file has been created.")

def create_imu_csv(data, theStep):
    with open('imuRaw' + theStep + '.csv', 'w', newline='') as file:
        writer = csv.writer(file)

        header = ['Time', 'Ax', 'Ay', 'Az', 'Wx', 'Wy', 'Wz']
        writer.writerow(header)

        for i in range(data.shape[1] - 1):
            row = data[:, i].tolist()
            writer.writerow(row)

    print(f"IMU CSV files has been created")
 
#dataset = ["1", "2", "3", "4", "5", "6", "7", "8", "9"]
dataset = "1"
ifile = "/home/d3shin/HDD/School/ECE276A/Projects/IMU/data/zips/testset/imu/imuRaw10.p" #"../data/cam/cam" + dataset + ".p"
#ifile = "../data/imu/imuRaw" + dataset + ".p"

#for theStep in dataset:
#    vfile = "../data/vicon/viconRot" + theStep + ".p"
    #ifile = "../data/imu/imuRaw" + theStep + ".p"
ts = tic()
    #imud = read_data(ifile)
    #create_imu_csv(imud, theStep)
#    vicd = read_data(vfile)
#    create_vicon_csv(vicd, theStep)

#toc(ts, "Data import")


#ts = tic()
#camd = read_data(cfile)
toc(ts, "Data import")
imud = read_data(ifile)
#print(imud)
#vicd = read_data(vfile)
#print(vicd)
#create_imu_csv(imud)
#create_vicon_csv(vicd)
#toc(ts,"Data import")
create_imu_csv(imud, "10")

#rots = vicd['rots']
#ts = vicd['ts']

# Number of time steps
#num_time_steps = rots.shape[2]

# Loop through each time step and print the corresponding rotation matrix and timestamp
#for i in range(num_time_steps):
#    print(f"Time step {i+1}:")
#    print(f"Timestamp: {ts[0, i]}")
#    print("Rotation Matrix:")
#    print(rots[:, :, i])
#    print("\n")





