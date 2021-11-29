import QR_Frame_calc as QR

qr_posx_world1 = -6.75
qr_posy_world1 = 0.409

qr_posx_world2 = -6.26
qr_posy_world2 = 2.888

qr_posx_world3 = -4.713
qr_posy_world3 = 3.06

qr_posx_hidden1 = 0.1
qr_posy_hidden1 = 3.5

qr_posx_hidden2 = 2.67
qr_posy_hidden2 = 3.23

qr_posx_hidden3 = 3.03
qr_posy_hidden3 = 1.15

world_array = [qr_posx_world1,qr_posy_world1,qr_posx_world2,qr_posy_world2]
hidden_array = [qr_posx_hidden1,qr_posy_hidden1,qr_posx_hidden2,qr_posy_hidden2]
print world_array
print hidden_array
framepoint,angle = QR.QR_frame_calc(world_array,hidden_array)

print "Framepoint: ",framepoint, "Angle: ",angle