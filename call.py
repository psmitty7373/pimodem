import gpiod
import time

# Define the GPIO chip and line
c = gpiod.Chip('gpiochip0')
line = gpiod.find_line('GPIO18')
line.request(consumer='try line', type=gpiod.LINE_REQ_DIR_OUT)

print('chip name:', c.name())
print('chip label:', c.label())

line.set_value(0)
time.sleep(45)
line.set_value(1)
