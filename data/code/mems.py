import math
import time
import threading
import scipy
import numpy as np
from numpy import fft
import scipy.signal as signal
import matplotlib.pyplot as plt
import pyecharts.options as opts
from enum import Enum
from collections import deque
from pyecharts.charts import Line
from matplotlib.animation import FuncAnimation

class MEMSDataType(Enum):
  MEMS_X = 0
  MEMS_Y = 1
  MEMS_Z = 2
  MEMS_VECTOR = 3
  MEMS_SQUARE = 4
  MEMS_ALL = 5

class MEMSData(object):

  def __init__(self, data_type = MEMSDataType.MEMS_ALL) -> None:
    self.m_data_type = data_type
    if self.m_data_type == MEMSDataType.MEMS_ALL:
      self.m_x = list()
      self.m_y = list()
      self.m_z = list()
      self.m_vector_sum = list()
      self.m_square_sum = list()
    else:
      self.m_data = list()

  def raise_type_error(self):
    print("MEMS data type error")
    exit(0)
    
  def calculate_vector_sum(self, x, y, z):
    vector_sum = 0
    if x >= 0:
      vector_sum += math.pow(x, 2)
    else:
      vector_sum -= math.pow(x, 2)
    
    if y >= 0:
      vector_sum += math.pow(y, 2)
    else:
      vector_sum -= math.pow(y, 2)

    if z >= 0:
      vector_sum += math.pow(z, 2)
    else:
      vector_sum -= math.pow(z, 2)
    
    if vector_sum >= 0:
      vector_sum = math.sqrt(vector_sum)
    else:
      vector_sum = -1 * math.sqrt(-1 * vector_sum)
    return vector_sum

  def calculate_square_sum(self, x, y, z):
    return math.sqrt(math.pow(x, 2) + math.pow(y, 2) + math.pow(z, 2))
  
  def get_x(self) -> list():
    if self.m_data_type == MEMSDataType.MEMS_X:
      return self.m_data
    elif self.m_data_type == MEMSDataType.MEMS_ALL:
      return self.m_x
    else:
      self.raise_type_error()
  
  def get_y(self) -> list():
    if self.m_data_type == MEMSDataType.MEMS_Y:
      return self.m_data
    elif self.m_data_type == MEMSDataType.MEMS_ALL:
      return self.m_y
    else:
      self.raise_type_error()

  def get_z(self) -> list():
    if self.m_data_type == MEMSDataType.MEMS_Z:
      return self.m_data
    elif self.m_data_type == MEMSDataType.MEMS_ALL:
      return self.m_z
    else:
      self.raise_type_error()

  def get_square_sum(self) -> list():
    if self.m_data_type == MEMSDataType.MEMS_SQUARE:
      return self.m_data
    elif self.m_data_type == MEMSDataType.MEMS_ALL:
      return self.m_square_sum
    else:
      self.raise_type_error()

  def get_vector_sum(self) -> list():
    if self.m_data_type == MEMSDataType.MEMS_VECTOR:
      return self.m_data
    elif self.m_data_type == MEMSDataType.MEMS_ALL:
      return self.m_vector_sum
    else:
      self.raise_type_error()

class MEMSDataFile(MEMSData):
  def __init__(self, file_path):
    super().__init__(data_type = MEMSDataType.MEMS_ALL)
    self.m_file = open(file_path)
    lines = self.m_file.readlines()
    for line in lines:
      if "00> " in line and "," in line:
        x = (int)(line.split("00> ")[1].split(",")[0])
        y = (int)(line.split("00> ")[1].split(",")[1])
        z = (int)(line.split("00> ")[1].split(",")[2].split("\n")[0])
        
        vector_sum = self.calculate_vector_sum(x, y, z)
        square_sum = self.calculate_square_sum(x, y, z)
        
        self.m_x.append(x)
        self.m_y.append(y)
        self.m_z.append(z)
        self.m_square_sum.append(square_sum)
        self.m_vector_sum.append(vector_sum)    
    self.m_x = np.array(self.m_x)
    self.m_y = np.array(self.m_y)
    self.m_z = np.array(self.m_z)
    self.m_square_sum = np.array(self.m_square_sum)
    self.m_vector_sum = np.array(self.m_vector_sum)

  def __del__(self):
    self.m_file.close()

class MEMSDataBuffer(MEMSData):
  def __init__(self, data_type, buffer_size, step_size, frame_rate, data_process, visualize = False) -> None:
      #only can visual one kind of data in x, y, z, vector, square currently
      self.m_visualize = visualize

      if data_type == MEMSDataType.MEMS_ALL:
        self.raise_type_error()
      else:
        super().__init__(data_type = data_type)

      self.m_frame_rate = frame_rate
      self.m_buffer_size = buffer_size * self.m_frame_rate
      self.m_step_size = step_size * self.m_frame_rate
      
      self.m_data = deque(maxlen = self.m_buffer_size)
      self.m_processed_data = self.m_data
      self.m_step_cnt = 0

      self.m_data_process = data_process
      if self.m_visualize:
        self.visual_init()

  def __def__(self):
    pass
  
  def visual_init(self):
    #lt.ion() #开启interactive mode 成功的关键函数
    plt.figure(1)
    plt.plot(range(0, self.m_buffer_size), [0] * self.m_buffer_size) # 一条轨迹
    plt.ylim(-50, 50)


  def visual_update(self):
    self.m_processed_data = self.m_data_process(self.m_data)
    """
    fft_data = fft.fft(self.m_processed_data)
    abs_data=np.abs(fft_data)                # 取复数的绝对值，即复数的模(双边频谱)
    angle_data=np.angle(fft_data)
    amplitude_data = abs_data[0:12]
    amplitude_data = amplitude_data / 25
    """
    plt.clf() # 清空画布上的所有内容。此处不能调用此函数，不然之前画出的轨迹，将会被清空。
    plt.plot(range(0, len(self.m_processed_data)), self.m_processed_data) # 一条轨迹
    plt.ylim(-50, 50)
    plt.draw()#注意此函数需要调用  
    plt.pause(0.01)
  
  def append(self, data):
    self.m_data.append(data)
    if len(self.m_data) == self.m_buffer_size:
      self.m_step_cnt += 1
      if self.m_step_cnt == self.m_step_size:
        self.m_step_cnt = 0
        self.visual_update()


class MEMSDataProcess(object):
  
  _instance_lock = threading.Lock()
  
  def __init__(self):
    pass

  def __def__(self):
    pass

  @classmethod
  def instance(cls, *args, **kwargs):
      with MEMSDataProcess._instance_lock:
          if not hasattr(MEMSDataProcess, "_instance"):
              MEMSDataProcess._instance = MEMSDataProcess(*args, **kwargs)
      return MEMSDataProcess._instance

  def get_timestamp(self):
    return int(round(time.time() * 1000))
  
  def z_score_normaliz(self, data):
    return scipy.stats.zscore(data)

  def mean_align(self, data):
    return data - np.mean(data)

  def med_filter(self, data, win) -> list(): #平滑
    with MEMSDataProcess._instance_lock:
      return signal.medfilt(data, win)

  def high_pass_filter(self, data, freq,  threshold, order) -> list():
    with MEMSDataProcess._instance_lock:
      filter_arg = 2.0 * threshold / freq
      b, a = signal.butter(order, filter_arg, 'highpass')    #配置滤波器 8 表示滤波器的阶数
      return signal.filtfilt(b, a, data)

  def low_pass_filter(self, data, freq,  threshold, order) -> list():
    with MEMSDataProcess._instance_lock:
      filter_arg = 2.0 * threshold / freq
      b, a = signal.butter(order, filter_arg, 'lowpass')    #配置滤波器 8 表示滤波器的阶数
      return signal.filtfilt(b, a, data)

  def band_pass_filter(self, data, freq,  low_threshold, up_threshold, order) -> list():
    with MEMSDataProcess._instance_lock:
      filter_arg0 = 2.0 * low_threshold / freq
      filter_arg1 = 2.0 * up_threshold / freq
      b, a = signal.butter(order, [filter_arg0, filter_arg1], 'bandpass')
      return signal.filtfilt(b, a, data)
    
  def sg_filter(self, data, win, order):
    return signal.savgol_filter(data, win, order)

  def generate_spectrogram(self, data, freq, path, name, isshow = False):
    with MEMSDataProcess._instance_lock:
      plt.specgram(data, Fs=freq, scale_by_freq=True, sides='default')  # 绘制频谱
      plt.xlabel('Time(s)')
      plt.ylabel('Frequency')
      plt.title(name + "(Spectrogram)")
      plt.savefig(path + name + "(Spectrogram)" + ".jpg")
      if isshow:
        plt.show()
      plt.close()
    return

  def generate_signal(self, data, freq, path, name, isshow = False):
    with MEMSDataProcess._instance_lock:
      time = np.arange(0, len(data)) * (1.0 / freq)
      time = np.reshape(time, [len(data), 1]).T
      plt.plot(time[0, :len(data)], data, c="b")
      plt.xlabel("time(seconds)")
      plt.ylabel("amplitude")
      plt.title(name + "(Signal)")
      plt.savefig(path + name + "(Signal)" + ".jpg")
      if isshow:
        plt.show()
      plt.close()
    return  
  
  def generate_signal_echart(self, data, freq, path, name, isshow = False):
    line = (
      Line(init_opts=opts.InitOpts(width="1400px", height="800px"))
        .add_xaxis(
            xaxis_data=np.arange(0, len(data)) * (1.0 / freq),
        )
        .add_yaxis(
            series_name="Hz",
            y_axis=data,
            label_opts=opts.LabelOpts(is_show=True),
            markpoint_opts=opts.MarkPointOpts(
                data=[
                    opts.MarkPointItem(type_="max", name="最大值"),
                    opts.MarkPointItem(type_="min", name="最小值"),
                ]
            ),
            markline_opts=opts.MarkLineOpts(
                data=[opts.MarkLineItem(type_="average", name="平均值")]
            ),
        )
        .set_global_opts(
            title_opts=opts.TitleOpts(title=name, subtitle="(Signal)"),
            tooltip_opts=opts.TooltipOpts(trigger="axis"),
            toolbox_opts=opts.ToolboxOpts(is_show=True),
            xaxis_opts=opts.AxisOpts(type_="category", boundary_gap=False),
            datazoom_opts=opts.DataZoomOpts(is_show=True, type_='slider')
        )
    )
    line.render(path + name + "(Signal)" + ".html")





