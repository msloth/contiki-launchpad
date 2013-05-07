# Author Marcus Linderoth <linderoth.marcus@gmail.com>
# print out a sinus look-up table
import math

min = 0
max = 90
steps = 45

sinlut = "const uint8_t sin_lut[] = {"
for x in xrange(min, max, max/steps):
	val = 100 * math.sin(math.pi*x/180)
	sinlut += str(int(val)) + ", "
	#print(str(int(100 * math.sin(math.pi*x/180)))),

sinlut += '}'

print(sinlut)
