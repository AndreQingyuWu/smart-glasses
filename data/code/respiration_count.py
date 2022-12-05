
from collections import deque
from enum import Enum

class RespirationCount:   #25, 2s
  class Stat(Enum):       #通过两条线的上一个时刻的状态总能推断出下一个时刻状态
    UP = 0
    DOWN = 1
    #UP_CROSS = 2
    #DOWN_CROSS = 3
    #EQUAL = 4

  class EMA:
    def __init__(self, sample_rate, win_size) -> None:
      self.sample_rate = sample_rate
      self.win_size = win_size
      self.last_effect_cross_inter = None #最近一次的有效交点时间
      self.last_effect_cross_value = None
      self.last_cross_inter = None
      self.last_cross_value = None
      self.cur_cross_inter = None
      self.cur_cross_value = None
      self.last_stat = None
      self.cur_stat = None
      self.last_val = None
      self.val = None
    def calculate(self, data):
      self.last_val = self.val
      N = self.win_size
      if self.val == None:
        self.val = 0
        for i in range(N):
          self.val += (2/(N + 1) * pow(((N-1)/(N+1)), N-i) * data)
      else:
        self.val = (2*data+(N-1)*self.last_val)/(N+1)  
  class MA:
    def __init__(self, sample_rate, win_size) -> None:
      self.sample_rate = sample_rate
      self.win_size = win_size
      self.last_effect_cross_inter = None #最近一次的有效交点时间
      self.last_effect_cross_value = None
      self.last_cross_inter = None
      self.last_cross_value = None
      self.cur_cross_inter = None
      self.cur_cross_value = None
      self.last_val = None
      self.val = None
      self.buffer = deque(maxlen = self.win_size)
    def calculate(self, data):
      self.last_val = self.val
      N = self.win_size
      if self.val == None:
        for i in range(N):
          self.buffer.append(data)
        self.val = data
      else:
        self.val = (self.last_val * N - self.buffer[0] + data) / N
        self.buffer.append(data)

  def __init__(self, sample_rate, win_size) -> None:
      self.m_sample_rate = sample_rate
      self.m_win_size = win_size * sample_rate
      self.m_EMA = self.EMA(self.m_sample_rate, self.m_win_size)
      self.m_MA = self.MA(self.m_sample_rate, self.m_win_size)
      

  def upcross_callback(self):
    print("up cross")
    pass

  def downcross_callback(self):
    print("down cross")
    pass

  def update_stat(self):
    if self.m_EMA.val != self.m_MA.val:
      self.m_EMA.last_stat = self.m_EMA.cur_stat
      if self.m_EMA.val > self.m_MA.val:
        self.m_EMA.cur_stat = self.Stat.UP
        if self.m_EMA.last_stat == self.Stat.DOWN:
          self.upcross_callback()

      if self.m_EMA.val < self.m_MA.val:
        self.m_EMA.cur_stat = self.Stat.DOWN
        if self.m_EMA.last_stat == self.Stat.UP:
          self.downcross_callback()
    else:
      pass

  def append(self, frame_data):
    self.m_EMA.calculate(frame_data)
    self.m_MA.calculate(frame_data)
    self.update_stat()    

if __name__ == '__main__':
  res_cnt = RespirationCount(25, 1)
  example = [-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4, -3, -2, -1, 0, \
             1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, \
            -25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4, -3, -2, -1, 0, \
             1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, \
            -25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4, -3, -2, -1, 0, \
             1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, \
            -25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4, -3, -2, -1, 0, \
             1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, \
            -25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4, -3, -2, -1, 0, \
             1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, \
            -25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4, -3, -2, -1, 0, \
             1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25]
  for _, n in enumerate(example):
    res_cnt.append(n)