#oding: utf-8
#!usr/bin/python3
import tkinter as tk
import PIL.Image, PIL.ImageTk
from smbus import SMBus
from time import sleep
from lsm303d import LSM303D
import serial
import micropyGPS
import threading
import gpiozero as gp
import signal
import os
gps = micropyGPS.MicropyGPS(9, 'dd')
bus_number  = 1
i2c_address = 0x76
lsm = LSM303D(0x1d)
bus = SMBus(bus_number)

digT = []
digP = []
digH = []
gp.Button.was_held = False
calc_count = 0
t_fine = 0.0
sea_P = -1
T = 0
P = 0
H = 0

def btn13_held(btn):
	btn.was_held = True
	global calc_count
	calc_count += 1

def btn16_held(btn):
	btn.was_held = True
	global sumdist
	global start
	global net_start
	global sumup
	global max_speed
	global gradi
	gradi = 0
	max_speed = 0
	sumdist = 0
	start = 1
	net_start = 1
	sumup = 0

def btn13_released(btn):
	if not btn.was_held:
		global calc_count
		calc_count += 1
	btn.was_held = False

def btn16_released(btn):
	btn.was_held = False

def calc_stop():
	btn13 = gp.Button(13)
	btn16 = gp.Button(16)
	btn13.when_held = btn13_held
	btn13.when_released = btn13_released
	btn16.when_held = btn16_held
	btn16.when_released = btn16_released
	signal.pause()

def rungps():
    s = serial.Serial('/dev/serial0', 9600, timeout=10)
    s.readline()
    while True:
        sentence = s.readline().decode('utf-8', errors='ignore')
        if sentence[0] != '$':
            continue
        for x in sentence:
            gps.update(x)

def writeReg(reg_address, data):
	bus.write_byte_data(i2c_address,reg_address,data)

def get_calib_param():
	calib = []
	
	for i in range (0x88,0x88+24):
		calib.append(bus.read_byte_data(i2c_address,i))
	calib.append(bus.read_byte_data(i2c_address,0xA1))
	for i in range (0xE1,0xE1+7):
		calib.append(bus.read_byte_data(i2c_address,i))

	digT.append((calib[1] << 8) | calib[0])
	digT.append((calib[3] << 8) | calib[2])
	digT.append((calib[5] << 8) | calib[4])
	digP.append((calib[7] << 8) | calib[6])
	digP.append((calib[9] << 8) | calib[8])
	digP.append((calib[11]<< 8) | calib[10])
	digP.append((calib[13]<< 8) | calib[12])
	digP.append((calib[15]<< 8) | calib[14])
	digP.append((calib[17]<< 8) | calib[16])
	digP.append((calib[19]<< 8) | calib[18])
	digP.append((calib[21]<< 8) | calib[20])
	digP.append((calib[23]<< 8) | calib[22])
	digH.append( calib[24] )
	digH.append((calib[26]<< 8) | calib[25])
	digH.append( calib[27] )
	digH.append((calib[28]<< 4) | (0x0F & calib[29]))
	digH.append((calib[30]<< 4) | ((calib[29] >> 4) & 0x0F))
	digH.append( calib[31] )
	
	for i in range(1,2):
		if digT[i] & 0x8000:
			digT[i] = (-digT[i] ^ 0xFFFF) + 1

	for i in range(1,8):
		if digP[i] & 0x8000:
			digP[i] = (-digP[i] ^ 0xFFFF) + 1

	for i in range(0,6):
		if digH[i] & 0x8000:
			digH[i] = (-digH[i] ^ 0xFFFF) + 1  

def readData():
	data = []
	for i in range (0xF7, 0xF7+8):
		data.append(bus.read_byte_data(i2c_address,i))
	pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
	temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
	hum_raw  = (data[6] << 8)  |  data[7]
	
	T = compensate_T(temp_raw)
	P = compensate_P(pres_raw)
	H = compensate_H(hum_raw)
	return T, P, H

def compensate_P(adc_P):
	global  t_fine
	pressure = 0.0
	
	v1 = (t_fine / 2.0) - 64000.0
	v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048) * digP[5]
	v2 = v2 + ((v1 * digP[4]) * 2.0)
	v2 = (v2 / 4.0) + (digP[3] * 65536.0)
	v1 = (((digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192)) / 8)  + ((digP[1] * v1) / 2.0)) / 262144
	v1 = ((32768 + v1) * digP[0]) / 32768
	
	if v1 == 0:
		return 0
	pressure = ((1048576 - adc_P) - (v2 / 4096)) * 3125
	if pressure < 0x80000000:
		pressure = (pressure * 2.0) / v1
	else:
		pressure = (pressure / v1) * 2
	v1 = (digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096
	v2 = ((pressure / 4.0) * digP[7]) / 8192.0
	pressure = pressure + ((v1 + v2 + digP[6]) / 16.0)
	return pressure/100
	print("pressure : %7.2f hPa" % (pressure/100))

def compensate_T(adc_T):
	global t_fine
	v1 = (adc_T / 16384.0 - digT[0] / 1024.0) * digT[1]
	v2 = (adc_T / 131072.0 - digT[0] / 8192.0) * (adc_T / 131072.0 - digT[0] / 8192.0) * digT[2]
	t_fine = v1 + v2
	temperature = t_fine / 5120.0
	return temperature
	print("temp : %-6.2f ℃" % (temperature))

def compensate_H(adc_H):
	global t_fine
	var_h = t_fine - 76800.0
	if var_h != 0:
		var_h = (adc_H - (digH[3] * 64.0 + digH[4]/16384.0 * var_h)) * (digH[1] / 65536.0 * (1.0 + digH[5] / 67108864.0 * var_h * (1.0 + digH[2] / 67108864.0 * var_h)))
	else:
		return 0
	var_h = var_h * (1.0 - digH[0] * var_h / 524288.0)
	if var_h > 100.0:
		var_h = 100.0
	elif var_h < 0.0:
		var_h = 0.0
	
	return var_h
	print("hum : %6.2f ％" % (var_h))


def setup():
	osrs_t = 1			#Temperature oversampling x 1
	osrs_p = 1			#Pressure oversampling x 1
	osrs_h = 1			#Humidity oversampling x 1
	mode   = 3			#Normal mode
	t_sb   = 5			#Tstandby 1000ms
	filter = 0			#Filter off
	spi3w_en = 0			#3-wire SPI Disable

	ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode
	config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en
	ctrl_hum_reg  = osrs_h

	writeReg(0xF2,ctrl_hum_reg)
	writeReg(0xF4,ctrl_meas_reg)
	writeReg(0xF5,config_reg)


setup()
get_calib_param()

class App(tk.Tk):
    # 呪文
    def __init__(self, *args, **kwargs):
        # 呪文
        tk.Tk.__init__(self, *args, **kwargs)

        # ウィンドウタイトルを決定
        self.title("cycle_computer")

        # ウィンドウの大きさを決定
        self.geometry("320x240")

        # ウィンドウのグリッドを 1x1 にする
        # この処理をコメントアウトすると配置がズレる
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)
#-----------------------------------main_frame-----------------------------
        # メインページフレーム作成
        self.main_frame = tk.Frame()
        self.main_frame.grid(row=0, column=0, sticky="nsew")

        #self.titleLabel = tk.Label(self.main_frame, text="速度 km", font=('Helvetica', '12'))
        #self.titleLabel.pack(anchor='center', expand=True)
        # フレーム1に移動するボタン
        self.changePageButton = tk.Button(self.main_frame, text="Next Page", command=lambda : self.changePage(self.frame1))
        self.changePageButton.place(relheight=0.25, relwidth=1, relx=0, rely=0.75)
#--------------------------------------------------------------------------
#-----------------------------------frame1---------------------------------
        # 移動先フレーム作成
        self.frame1 = tk.Frame()
        self.frame1.grid(row=0, column=0, sticky="nsew")
        # フレーム1からmainフレームに戻るボタン
        self.back_button = tk.Button(self.frame1, text="Back", command=lambda : self.changePage(self.main_frame))
        self.back_button.place(relheight=0.25, relwidth=0.5, relx=0, rely=0.75)
        self.back_button = tk.Button(self.frame1, text="Quit", command=lambda : self.close_window())
        self.back_button.place(relheight=0.25, relwidth=0.5, relx=0.5, rely=0.75)
        self.back_button = tk.Button(self.frame1, text="斜度計算", command=lambda : self.calibrate_P())
        self.back_button.place(relheight=0.25, relwidth=1, relx=0, rely=0.5)
#--------------------------------------------------------------------------

        #main_frameを一番上に表示
        self.main_frame.tkraise()

    def changePage(self, page):
        '''
        画面遷移用の関数
        '''
        page.tkraise()
	
    def close_window(self):
        global sumdist
        global start
        global net_start
        global sumup
        global max_speed
        path_f = "log.txt"
        s = str(max_speed) + ' ' + str(sumdist) + ' ' + str(start) + ' ' + str(net_start) + ' ' + str(sumup)
        with open(path_f, mode='w') as f:
            f.write(s)
        self.destroy()

	
    def calibrate_P(self):
        global sea_P
        global prev_alt
        global P
        global T
        global cal_temp
        global sumdist
        global cal_prevh
        temph = gps.altitude
        cal_temp = sumdist
        cal_prevh = temph
        prev_alt = temph
        sea_P = P * ((1 - 0.0065 * temph / (T + 0.0065 * temph + 273.15)) ** (-5.257))

if __name__ == "__main__":
	app = App()
	gpsthread = threading.Thread(target=rungps, args=())
	count_thread = threading.Thread(target=calc_stop, args=())
	gpsthread.daemon = True
	count_thread.daemon = True
	gpsthread.start()
	count_thread.start()
	sumdist = 0
	sumup = 0
	prev_alt = -1
	prev_speed = -1
	max_speed = 0
	start = 1
	net_start = 1
	cal_temp = 100000
	path_f = "log1.txt"
	if os.path.isfile(path_f):
		with open(path_f, mode='r') as f:
			max_speed, sumdist, start, net_start, sumup = f.read().split()
			sumdist = float(sumdist)
			max_speed = float(max_speed)
			start = int(start)
			net_start = int(start)
			sumup = float(sumup)
	gx = -1
	gy = -1
	gz = -100
	gradi = 0
	cal_prevh = 0
	while True:
		xyz = lsm.accelerometer()
		start += 1
		tempspeed = gps.speed[2]
		T, P, H = readData()
		nowh = gps.altitude if sea_P < P else ((sea_P / P) ** (1 / 5.257) - 1) * (T + 273.15) / 0.0065
		tempdist = 0 if prev_speed == -1 else (prev_speed + tempspeed) / 7.2
		net_start += 1
		if sumdist - cal_temp > 20:
			gradi = (nowh - cal_prevh) / (sumdist - cal_temp) * 100
			cal_prevh = nowh
			cal_temp = sumdist
		if (abs(xyz[0] - gx < 0.02 and abs(xyz[1] - gy) < 0.02 and abs(xyz[2] - gz) < 0.02)) or calc_count % 2 == 1:
			tempspeed = 0
			tempdist = 0
			net_start -= 1
			prev_alt = nowh
			cal_temp -= 1
			if calc_count % 2 == 1: start -= 1
		max_speed = max(max_speed, tempspeed)
		T = int(T * 100) / 100
		h = gps.timestamp[0] if gps.timestamp[0] < 24 else gps.timestamp[0] - 24
		speed = tk.Label(app.main_frame, text="速度 [km/h]\n" + str(int(tempspeed * 10) / 10), font=('Helvetica', '14'), relief=tk.RIDGE, bd=1)
		speed.place(relheight=0.25, relwidth=0.5, relx=0, rely=0)
		speed.update()
		speed.forget()
		temperture = tk.Label(app.main_frame, text="最高速度 [km/h]\n" + str(int(max_speed * 10) / 10), font=('Helvetica',14), relief=tk.RIDGE, bd=1)
		temperture.place(relheight=0.25, relwidth=0.5, relx=0.5, rely=0)
		temperture.update()
		temperture.forget()
		grad = tk.Label(app.main_frame, text="斜度 [%]\n" + str(int(gradi * 10) / 10), font=('Helvetica', '14'), relief=tk.RIDGE, bd=1)
		grad.place(relheight=0.25, relwidth=0.5, relx=0.5, rely=0.25)
		grad.update()
		grad.forget()
		nowtime = tk.Label(app.main_frame, text="時間\n" + str(h) + ":" + str(gps.timestamp[1]), font=('Helvetica', '14'), relief=tk.RIDGE, bd=1)
		nowtime.place(relheight=0.25, relwidth=0.5, relx=0, rely=0.25)
		nowtime.update()
		nowtime.forget()
		dist = tk.Label(app.main_frame, text="総走行距離 [km]\n" + str(int(sumdist / 10) / 100), font=('Helvetica', '14'), relief=tk.RIDGE, bd=1)
		dist.place(relheight=0.25, relwidth=1, relx=0, rely=0.5)
		dist.update()
		dist.forget()
		gross = tk.Label(app.frame1, text="gross [km/h]\n" + str(int(sumdist / start * 3.6 * 10) / 10), font=('Helvetica', '14'), relief=tk.RIDGE, bd=1)
		gross.place(relheight=0.25, relwidth=0.5, relx=0, rely=0)
		gross.update()
		gross.forget()
		net = tk.Label(app.frame1, text="net [km/h]\n" + str(int(sumdist / net_start * 3.6 * 10) / 10), font=('Helvetica', '14'), relief=tk.RIDGE, bd=1)
		net.place(relheight=0.25, relwidth=0.5, relx=0.5, rely=0)
		net.update()
		net.forget()
		humid = tk.Label(app.frame1, text="温度 [℃]\n" + str(T), font=('Helvetica', '14'), relief=tk.RIDGE, bd=1)
		humid.place(relheight=0.25, relwidth=0.5, relx=0.5, rely=0.25)
		humid.update()
		humid.forget()
		up = tk.Label(app.frame1, text="獲得標高 [m] \n" + str(int(sumup * 10) / 10), font=('Helvetica', '14'), relief=tk.RIDGE, bd=1)
		up.place(relheight=0.25, relwidth=0.5, relx=0, rely=0.25)
		up.update()
		up.forget()
		if prev_alt != -1 and prev_alt < nowh: sumup += nowh - prev_alt
		sumdist += tempdist
		prev_alt = nowh
		prev_speed = tempspeed
		cal_temp += 1
		gx = xyz[0]
		gy = xyz[1]
		gz = xyz[2]
		sleep(1)
