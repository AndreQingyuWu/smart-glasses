from mems import *
import time
import asyncio

buffer = MEMSDataBuffer(MEMSDataType.MEMS_VECTOR, 5, 1, 10, True)

async def show_figure():
  buffer.visual_show()
async def data_update():
  i = 0
  while True:
    time.sleep(0.01)
    buffer.append(buffer.calculate_vector_sum(i, i, i))
    i += 0.02
    if i == 0.4:
      i = 0

if __name__ == "__main__":
  coroutine1 = show_figure()
  coroutine2 = data_update()

  tasks = [
      asyncio.ensure_future(coroutine1),
      asyncio.ensure_future(coroutine2),
  ]
  loop = asyncio.get_event_loop()
  loop.run_until_complete(asyncio.wait(tasks))