file = open('C:\\Users\\andre\\Desktop\\glass\\NordicGlass-14\\data\\freq.txt')
lines = file.readlines()
freq = 0
for line in lines:
  temp = float(line.split("freq:")[1].split("Hz")[0])
  freq += temp
freq = freq / len(lines)
print(freq)