from agile_eye import AGILE

gazer = AGILE('/dev/ttyUSB0', warnMe='off')
print(gazer.jacob([30, 30, 30], [0, 0, 0]))
