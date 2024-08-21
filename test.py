slp = 3
yin = 2
ptx = 4
pty = 6
inx = (-1/slp+(pty+1/slp*ptx)-yin)/slp
iny = -1/slp*(inx-ptx)+pty

print(inx, iny)