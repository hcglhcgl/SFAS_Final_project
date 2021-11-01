#!/usr/bin/env python


#Read QR-code
finalWord = ["0", "0", "0", "0", "0"]

data = "X=2.35\r\nY=3.24\r\nX_next=5.3\r\nY_next=5.9\r\nN=3\r\nL=M"

x = float(data.split("\r\n")[0].split("=")[1])
y = float(data.split("\r\n")[1].split("=")[1])
x_next = float(data.split("\r\n")[2].split("=")[1])
y_next = float(data.split("\r\n")[3].split("=")[1])
N = int(data.split("\r\n")[4].split("=")[1])
L = data.split("\r\n")[5].split("=")[1]

finalWord[N-1] = L

#The main script file