from mems import *

BASE_DATA_PATH = "C:\\Users\\andre\\Desktop\\glass\\NordicGlass-14\\data\\"
ORIGINAL_DATA_PATH = "C:\\Users\\andre\\Desktop\\glass\\NordicGlass-14\\data\\original_data\\"
ACC_DATA_PATH = BASE_DATA_PATH + "acc_data\\"
GYR_DATA_PATH = BASE_DATA_PATH + "gyr_data\\"
CURRENT_DATA_PATH = GYR_DATA_PATH

if __name__ == '__main__':
  nonblink_file_path = ORIGINAL_DATA_PATH + "gyr1.0.log"
  blink_file_path = ORIGINAL_DATA_PATH + "gyr1.1.log"
  blink = MEMSDataFile(blink_file_path)
  nonblink = MEMSDataFile(nonblink_file_path)

  blink_vector_data = blink.get_vector_sum()
  blink_square_data = blink.get_square_sum()
  nonblink_vector_data = nonblink.get_vector_sum()
  nonblink_square_data = nonblink.get_square_sum()

  blink_vector_data = MEMSDataProcess.instance().med_filter(blink_vector_data, 3)
  blink_square_data = MEMSDataProcess.instance().med_filter(blink_square_data, 3)
  nonblink_vector_data = MEMSDataProcess.instance().med_filter(nonblink_vector_data, 3)
  nonblink_square_data = MEMSDataProcess.instance().med_filter(nonblink_square_data, 3)

  blink_vector_data = MEMSDataProcess.instance().band_pass_filter(blink_vector_data, 30, 5, 10, 4)
  blink_square_data = MEMSDataProcess.instance().band_pass_filter(blink_square_data, 30,  5, 10, 4)
  nonblink_vector_data = MEMSDataProcess.instance().band_pass_filter(nonblink_vector_data, 30,  5, 10, 4)
  nonblink_square_data = MEMSDataProcess.instance().band_pass_filter(nonblink_square_data, 30,  5, 10, 4)

  blink_vector_data = MEMSDataProcess.instance().band_pass_filter(blink_vector_data, 30, 0.2, 0.5, 2)
  blink_square_data = MEMSDataProcess.instance().band_pass_filter(blink_square_data, 30,  0.2, 0.5, 2)
  nonblink_vector_data = MEMSDataProcess.instance().band_pass_filter(nonblink_vector_data, 30,  0.2, 0.5, 2)
  nonblink_square_data = MEMSDataProcess.instance().band_pass_filter(nonblink_square_data, 30,   0.2, 0.5, 2)


  MEMSDataProcess.instance().generate_signal_echart(blink_vector_data, 30, CURRENT_DATA_PATH, "blink_vector_data", False)
  MEMSDataProcess.instance().generate_signal_echart(blink_square_data, 30, CURRENT_DATA_PATH, "blink_square_data", False)
  MEMSDataProcess.instance().generate_signal_echart(nonblink_vector_data, 30, CURRENT_DATA_PATH, "nonblink_vector_data", False)
  MEMSDataProcess.instance().generate_signal_echart(nonblink_square_data, 30, CURRENT_DATA_PATH, "nonblink_square_data", False)
