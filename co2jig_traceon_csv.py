# TODO :
# - Check the ppm target tolerance is matched when testing (ie calibrating boards)
# - use jig injection calibration table instead of hardcoded injection times
# - reinject no2 if co2 ppm is to high with respect to the target + tol

import pylibftdi
import serial
import serial.tools.list_ports
from time import sleep
import logging
import os
from time import time
from time import strftime
from time import localtime
import datetime
import re
import sys
import csv
import io
import configparser
import threading
import queue

# Version 1 :
#   - 1st version, used for NPI
# Version 2 (20130130_0700) :
#   - Continue testing even if the verification of some duts fails
#   - generate "MAC_CO2_RESULTS.txt" file for Fowcomm software integration
# Version 3 (20130216_1600) :
#   - wait 60s for DUT co2 sensor stabilisation (was 20s)
#   - calibrate and verify with 3 timings: slow, fast, very fast
#     only "slow" verif. is checked against tolerances.
# Version 4 (20130221_1600) :
#   - fix: don't stop the test if the a DUT fails the verification more than once
#   - send "co2 get_tr0_tp0_photo 1" and "co2 get_tr0_tp0_photo 0" for logging purposes
#   - fix: don't crash anymore if we receive non-ascii characters from DUT or CO2 meter
# Version 5 (20130225_1000) :
#   - Update calibration and verification points (calibration start at 300ppm)
# Version 6 (20130225_1400) :
#   - Erase calibration and verification table before calibration
#   - Measure co2 ppm before each calibration/verification
#   - Fix co2jig version display
# Version 7 (20130226_1100) :
#   - Add verification point at 600ppm, 20% limits
#   - Slightly change calibration/verification timings to optimize DFT computation
#   - Udate MFGID to 00080006
# Version 8 (20130226_1400) :
#   - Fix verification error ratio display for fast and veryfast verifications
# Version 9 (20130305_1900) :
#   - Fix verification error ratio display for fast verification
#   - Do not ignore bad UART character, replace them with '?' instead
#   - Do "aging" of CO2 lamps for ~30 seconds with fast calibration timings
#   - log tracebacks on exception
# Version 10 (20130306_2030)
#   - disable DUT timelimit (20 min)
# Version 11 (20130312_1530)
#   - Do update the MFGID anymore
# Version 11b (20130313_1000.py)
#   - No changes
#   - v11 notes are wrong and should be read as below:
#     - Do NOT update the MFGID anymore
# Version 12 (20130320_1200)
#   - Increase "aging" of CO2 lamps from ~30 seconds to ~3 minutes
# Version 13 (20130320_1600)
#   - In '-nocal' mode: do not erase calibration tables, do not do lamp aging
# Version 14 (20130422_1530)
#   - Display "50100" error code when jig needs recalibration
#   - Check "fast" verif. Now "slow" and "fast" verif. are checked against tolerances.
# Version 15 (20130806_1715)
#   - Do not stop the whole test if a DUT cmd fails (except DUT 1st probe cmd).
#     Instead, the DUT is declared as failed, and the test continues as if the cmd had succeeded
# Version 16 (20130828_1430)
#   - Fix crash when the co2 verification command failed on a DUT (bug introduced in v15).
#   - Disable DUT traces to prevent UART RX buffer overflow
#     Since we ignore cmd errors since v15, DUTs with trace enabled trigs a 30s timeout on all co2 cmds.
# Version 17 (20131029_1700)
#   - Decrease "aging" of CO2 lamps from ~3 linutes to ~12 seconds
#   - Remove "slow" and "veryfast" calibration
#   - Remove "slow" verification, and "veryfast" always-pass verification
#   - Fix logs: "SLOW" was displayed when starting or failing at the FAST verification.
# Version 18 (20131220_1000)
#	- Tolerance for verification at 1100ppm enlarged from 9% to 15%
#	- Verification point at 4600 ppm moved to 3600ppm
# Version 19 (20140106_1700)
#	- Tolerance for verification at 2100 ppm and 3600 ppm changed from 9% to 12 %
#	- function excludeDut() added in DutSet: commands are no longer send to a DUT that has been "excluded",  
#	  i.e. after a raise ValueError("cmd timeout")
#	  (avoids waisting time since the DUT will not answer anymore)
#
# TODO: log the list of enabled slots
#
# Version 20 (20150129_2000)
#   - Add air injection, choose 1600ppm as the threshold.
#   - Fix the bug of calculation of injection time of N2 and air.
#	- Add config file
#
# Globals
software_version = "Co2 jig software TRACE ON version"
logger = None

CO2CMD = 	{"probe" : "",        							#0
			"timelimit off": ['rc',0],    						#1
			"co2 get_tr0_tp0_photo 1"	: ['rc',0], 				#2
			"co2 get_tr0_tp0_photo 0"	: "", 			#3
"co2 calib 100 252 100 252 5 0 0 0.45 1": "",  		#4
					"perso del_co2cal"	: "",     		#5 
				"perso del_co2cal_fast"	: "",  	#6
				"perso del_co2cal_veryfast"	: "",  	#7 
	"co2 calib 100 252 100 252 5 x x 0.45 1": "",  		#8
						"co2 verif x x 1"	: "",   		#9
					"perso get_co2cal_fast"	: "" ,		#10
					"perso get_co2cal_fast" : ""
}





def configget(section,key=None):
	config = configparser.ConfigParser()
	with open("Config.ini",'r') as cfg:
		config.readfp(cfg)
		if key is None:
			optList =[]
			for option in config.options(section):
				optList.append([option,config.get(section,option)])
			return optList
		else:
			value = config.get(section,key)
			return value


class Relay:
	def __init__(self, id, name):
		"""Class describing devices connected to the relay board
			- id : (int) Relay #ID on the relay board (1->8)
			- name : (string) name of the device. ex: "fan power"
		"""
		self.id = id
		self.name = name
		#self.enabled_state = enabled_state

class RelayBoard:
	#__ftdi
	# Warning: relay 1, 3, 5 toggles at USB enumeration
	relay_fan_pwr  = Relay(5, "fan_pwr")
	relay_gas_air  = Relay(6, "gas_air")
	relay_gas_no2  = Relay(7, "gas_no2")
	relay_gas_co2  = Relay(8, "gas_co2")
	relay_dut_pwr  = Relay(2, "dut_pwr")
	relay_pump_pwr = Relay(4, "pump_pwr")
	relay_gas_out  = Relay(1, "gas_out")
	
	relays = (	relay_fan_pwr,
			relay_gas_out,
			relay_gas_no2,
			relay_gas_co2,
			relay_dut_pwr,
			relay_pump_pwr,
			relay_gas_air)

	def __init__(self, ftdi_sn = None):
		if (ftdi_sn == None):
			# Find the FTDI of the relay board automatically
			dev_list = pylibftdi.Driver().list_devices()			
			if len(dev_list) != 1:
				# Found none or more than 1 device matching...
				raise ValueError('Found %d relay board(s)' % len(dev_list))
			ftdi_sn = dev_list[0][2].decode()
			
		logger.info("Open relayboard device S/N <%s>" % ftdi_sn)
		self.__ftdi = pylibftdi.BitBangDevice(ftdi_sn)
		self.__ftdi.direction = 0xFF		# All I/O are outputs
		#self.__ftdi.port = 0x00		# All relay OFF		

	def setRelay(self, relay, on):
		state = "On" if on else "Off"
		logger.info("Set relay <%s> %s" % (relay.name, state))
		if(on):
			self.__ftdi.port |= (1<<(relay.id-1))
		else:
			self.__ftdi.port &= ~(1<<(relay.id-1))
			
	def enableRelay(self, relay):
		self.setRelay(relay, True)

	def disableRelay(self, relay):
		self.setRelay(relay, False)
		
	def disableAllRelays(self):
		self.__ftdi.port &= 0x00
		
	def relayIsOpen(self, relay):
		if (self.__ftdi.port & (1<<(relay.id-1))):
			return True
		return False

	def powerDutSet(self, on):
		self.setRelay(self.relay_dut_pwr, on)
			
	def powerFan(self, on):
		self.setRelay(self.relay_fan_pwr, on)
		
	def powerPump(self, on):
		self.setRelay(self.relay_pump_pwr, on)


class CalDot:
	def __init__(self, ppm, ppm_tol, no2_ton_ms, co2_ton_ms, dut_tol_coef = None):
		self.co2_ppm = ppm			# CO2 concentration target in ppm
		self.co2_ppm_tol = ppm_tol		# CO2 concentration tolerance
		self.no2_ton_ms = no2_ton_ms		# approx. NO2 injection time to reach target
		self.co2_ton_ms = co2_ton_ms		# approx. CO2 injection time to reach target
		self.dut_tol_coef = dut_tol_coef	# DUT verification tolerance (versus co2meter), or 'None' for calibration
		
	def refMatchTol(self, co2_ppm):
		dist = abs(co2_ppm - self.co2_ppm)
		if(dist > self.co2_ppm_tol):
			return False
		else:
			return True

	def refCompareTol(self, co2_ppm):
		'''co2_ppm : co2 level in ppm
		   Return 1, -1 or 0 if respectively the co2 level is
		   higher than, lower than, or match the co2 level target'''
		target = self.co2_ppm
		tol = self.co2_ppm_tol
		dist = (co2_ppm - target)
		if(dist > tol):
			return +1
		elif(dist < -tol):
			return -1
		else:
			return 0
	
	def dutTolError(self, ref_ppm, dut_ppm):
		''' ref_ppm : (int) ppm measured by the reference C02 meter
		    dut_ppm : (int) ppm measure by the DUT '''
		return abs(1.0 - float(dut_ppm) / float(ref_ppm))
	
	def dutMatchTol(self, ref_ppm, dut_ppm):
		''' ref_ppm : (int) ppm measured by the reference C02 meter
		    dut_ppm : (int) ppm measure by the DUT '''
		error = self.dutTolError(ref_ppm, dut_ppm)
		if error > self.dut_tol_coef:
			return False
		else:
			return True
		
class CalSettings:
	#light_preheat_ton_ms = 200
	#light_preheat_toff_ms = 600
	#light_ton_ms = 200
	#light_toff_ms = 600
	#light_nb_period = 7	# 1 pre-heat + 7 periods
	cal_dots = [	
			# CalDot(270,     30, 5000,   0),
			# CalDot(800,     80, 0,   1000),
			# CalDot(1600,   100, 0,   1000),
			# CalDot(2800,   100, 0,   1000),
			# CalDot(4600,   200, 0,   1000),

			# CalDot(5400,   200, 0,   1000, 0.12),
			# CalDot(4300,   200, 0,   1000, 0.12),
			# CalDot(3600,   200, 0,   1000, 0.12),
			# CalDot(2500,   100, 0,   1000, 0.12),
			#CalDot(3600,   200, 0,   1000, 0.12),
			#CalDot(2100,   100, 0,   1000, 0.12),
			CalDot(1000,   100, 0,   1000, 0.15),
			#CalDot( 600,    80, 0,   1000, 0.20),
			]
	fan_postinject_time_ms = 5000

class CmdResult:
	def __init__(self, rc, data,datalist):
		'''rc : (int) cmd return code
		   data : (dict) key = value result'''
		if type(rc) is not int:
			raise ValueError('rc is not a int')
		if type(data) is not dict:
			raise ValueError('data is not a dict')
		if type(datalist) is not list:
			raise ValueError('datalist is not a list')
		self.rc = rc
		self.data = data
		self.datalist = datalist
	@staticmethod
	def parse(string):
		rc = -1
		data = dict()
		datalist = list()
		lines = string.splitlines()
		for line in lines:
			keyval = line.split('=', 1)
			if(len(keyval) == 2):
				data[keyval[0]] = keyval[1]
				datalist.append(keyval)
		#logger.debug(data)
		
		# Parse cmd "rc=xxx"
		#if data.has_key('rc'):
		if 'rc' in data:
			rc = int(data['rc'])
			del data['rc']
			
		#logger.debug("rc=%s" % rc)
		return CmdResult(rc, data,datalist)
		
		

class Dut:
	def __init__(self, name, uart_name):
		self.__name = name
		self.__mac = None
		self.__uart = serial.Serial()
		self.__logfile = None
		self.__logname = ""
		self.__backlog = bytes()
		self.__pass = None		# Pass/Fail status (tristate: None, True, False)
		self.__failure_reason = None	# reason of failure if __pass = False

		
		# Configure UART
		self.__uart.setPort(uart_name)
		self.__uart.setBaudrate(1000000)
		self.__uart.setTimeout(0.1)
		
		# All the info for csv log		
		self.__infodic = {
			'mac_label'					: "",
			'dut_secret'				: "",
			'dut_bootloader_version'	: "",
			'dut_firmware_version'		: "",
			'dut_mfgid'					: "",	
		}
		self.co2cal_fast = list()
		self.co2verif_fast= list()
		
	def log_open(self, mac):
		# Create logger directory, if needed
		log_dirname = './logs'
		if not os.path.isdir(log_dirname):
			os.makedirs(log_dirname)
		
		# Create logger file
		date = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
		filename = '%s/%s_%s_%s.log' % (log_dirname, mac, date, self.__name)
		self.__logname = filename
		self.__logfile = open(filename, "a+b")
		
		# Flush the backlog
		self.__logfile.write(self.__backlog)
		self.__backlog = None

	def log(self, uart_bytes):
		# Strip "\n" to avoid "doubled" carriage returns
		# For Bytes type, must add [b'']
		stripped_uart_bytes = uart_bytes.replace(b'\n', b'')
		if(self.__logfile):
			self.__logfile.write(stripped_uart_bytes)
		else:
			self.__backlog += stripped_uart_bytes
		
	def getName(self):
		return self.__name
	
	def getLog(self):
		return self.__logname
	
	def get_cal_verif(self):
		return (self.__co2cal_fast, self.__co2verif_fast)
	
	def set_cal(self,datalist):
		self.__co2cal_fast = datalist
	def set_verif(self,datalist):
		self.__co2verif_fast = datalist
	
	def getMac(self):
		return self.__mac
	
	def getInfo(self):
		return self.__infodic
	
	def setPass(self, ok, failure_reason = None):
		if (self.__pass != None) and (ok != self.__pass):
			msg = 'Attempting to set dut <%s> to %s but dut alreay set to  %s ' % (
					self.getName(),
					ok,
					self.__pass
					)
			logger.warn(msg)
			raise ValueError(msg)

		if ok:
			msg = "Set DUT <%s> status as %s" % (self.getName(), ok)
			__failure_reason = None
		else:
			msg = "Set DUT <%s> status as %s (%s)" % (self.getName(), ok, failure_reason)
			__failure_reason = failure_reason
		logger.debug(msg)
		self.__pass = ok
		
	def getPass(self):
		return self.__pass
	
	def open(self):
		# Open UART
		self.__uart.open()
		self.__uart.flushInput()
		
		# Get DUT MAC address
		self.sendCmd("")
		self.sendCmd("probe")
		probeRes = self.getResult(1000)
		if(probeRes.rc != 0):
			logger.warn("Probing <%s> failed, rc=%d !" % (self.__name, probeRes.rc))
			raise ValueError("Probing <%s> failed !" % self.__name)
		decorated_mac = probeRes.data['mac']
		mac = decorated_mac.replace(":", "")
		
		# Get Probe info
		self.__infodic['mac_label'] = mac
		self.__infodic['dut_secret']= probeRes.data['secret']
		self.__infodic['dut_bootloader_version'] = probeRes.data['bl_version']
		self.__infodic['dut_firmware_version']= probeRes.data['soft_version']
		self.__infodic['dut_mfgid'] = probeRes.data['mfg_id']
		
		# Set mac attribute
		self.__mac = decorated_mac
		
		# Open logfile
		self.log_open(mac)
		
	def close(self):
		if self.__uart.isOpen():
			self.__uart.close()
		if type(self.__logfile) is io._io.BufferedReader:
			if not self.__logfile.closed:
				self.__logfile.close()
	
#	def isOpen(self):
#		return self.__uart.isOpen()

	def sendCmd(self, cmd):
		logger.debug("Dut <%s> TX :" % self.__name)
		logger.debug("\t%s" % cmd)
		#self.__uart.flushInput()
		# Workaround : the byte should be sent slowly...
		#self.__uart.write("%s\r" % cmd)
		cmd += '\r'
		for byte in cmd:
			self.__uart.write(byte.encode())
			sleep(0.001)
	
	def getResult(self, timeout_ms):
		text = ""
		text_pos = 0
		time_start = time()
		logger.debug("Dut <%s> RX :" % self.__name)
		logger.debug("-----------------------------")
		while True:
			chunk = self.__uart.read(512)
			elapsed_time = (time() - time_start) * 1000
			#logger.debug("elapsed time = %d" % elapsed_time)
			self.log(chunk)
			# Do we need to decode/encode !?
			# Replace non-ascii characters with '?' because we sometimes receive - because of a bad uart connection ? -
			# non-ascii characters from the DUT
			text += chunk.decode('ascii', 'replace')
			
			
			# Display board RX in logs, in "realtime"
			while True:
				cr_pos = text.find("\r", text_pos)
				if cr_pos == -1:
					break;
				stripped = text[text_pos:cr_pos].rstrip('\r\n')
				if len(stripped) > 0:
					logger.debug("  %s" % stripped)
				text_pos = cr_pos + 1
			
			# Return on shell prompt, or timeout
			if (text.find('shell>') != -1):
				logger.debug("-->prompt found")
				break
			if (elapsed_time > timeout_ms):
				logger.debug("-->timeout: %d ms", elapsed_time)
				raise ValueError("cmd timeout")
				break
			
		# Flush Display RX in logs
		stripped = text[text_pos:].rstrip('\r\n')
		if len(stripped) > 0:
			logger.debug("  %s" % stripped)	
		
		#for line in text.splitlines():
		#	stripped = line.rstrip('\r\n')
		#	if len(stripped) > 0:
		#		logger.debug("  %s" % stripped)
		logger.debug("-----------------------------")
		return CmdResult.parse(text)

class DutSetResult:
	def __init__(self, dut, cmd_result):
		''' dut		: (class Dut)
		    cmd_result	: (class CmdResult)'''
		self.dut = dut
		self.cmd_result = cmd_result
		
class DutSet:
	__excluded_duts = list()
	
	def __init__(self, nb_slots):
		duts = []
		namelist = configget('Dut')
		for item in namelist:
			duts.append(Dut(item[0],item[1]))
		self.__all_duts = tuple(duts)
		self.__duts = self.__all_duts[0:nb_slots]
	
	def open(self):
		for dut in self.__duts:
			dut.open()
			
	def close(self):
		for dut in self.__duts:
			dut.close()
	
	def getDuts(self):
		'''Return the list of duts currently in the set, including "excluded duts" (list() of Dut); read-only'''
		return self.__duts

	def excludeDut(self, dut):
		'''Exclude a DUT from the set of current DUTs.
		   Commands won't be sent anymore to this DUT.'''
		# only duts in current duts list (__duts) can be excluded
		if dut not in self.__duts:
			return
		if dut in self.__excluded_duts:
			return
		logger.info("Exclude DUT %s from set" % dut.getName())
		self.__excluded_duts.append(dut)
		
#	def removeDut(self, dut):
#		duts = self.__duts
#		if dut not in duts:
#			msg = ("Attempting to remove dut <%s> from dutset, but dut no in set" % dut.name)
#			log.error(msg)
#			raise ValueError(msg)
#		if dut.isOpen():
#			dut.close()
#		duts.remove(dut)
#		return dut
		
	def sendCmd(self, cmd, timeout_ms):
		logger.info("Send cmd <%s>" % cmd)
		dut_set_included = list()
		dut_set_results = list()
		
		dut_set_included = [dut for dut in self.__duts if dut not in self.__excluded_duts]
		for dut in dut_set_included:
			dut.sendCmd(cmd)
		for dut in dut_set_included:
			try:
				result = dut.getResult(timeout_ms)
				logger.info("board <%s>: rc=%d" %
					(dut.getName(), result.rc) )
				if(result.rc != 0):
					raise ValueError("board <%s>: rc=%d" %
					(dut.getName(), result.rc) )
				
				if cmd == "perso get_co2cal_fast":
					dut.set_cal(result.datalist)
				elif cmd == "perso get_co2verif_fast":
					dut.set_verif(result.datalist)
				
			except ValueError as e:
				error_reason = str(e)
				logger.info("board <%s>: command error (%s)" %
				(dut.getName(), error_reason))
				result = None
				dut.setPass(False, error_reason)
				# This dut got a cmd timeout error
				# We want to exclude it (i.e. don't send cmds to it anymore) because this probably means
				# the DUT will not answer anymore to the next cmds. Excluding the DUT saves timeouts
				# for the next cmds.
				self.excludeDut(dut)
			#####for test
			dut_set_results.append(DutSetResult(dut, result))
			

		return dut_set_results
		
	def sendCmdTraceon(self, cmd, timeout_ms):
		logger.info("Send cmd <%s>" % cmd)
		dut_set_included = list()
		dut_set_results = list()
		
		dut_set_included = [dut for dut in self.__duts if dut not in self.__excluded_duts]
		for dut in dut_set_included:
			dut.sendCmd(cmd)
		#for dut in dut_set_included:	
			try:
				result = dut.getResult(timeout_ms)
				logger.info("board <%s>: rc=%d" %
					(dut.getName(), result.rc) )
				if(result.rc != 0):
					raise ValueError("board <%s>: rc=%d" %
					(dut.getName(), result.rc) )
			except ValueError as e:
				error_reason = str(e)
				logger.info("board <%s>: command error (%s)" %
				(dut.getName(), error_reason))
				result = None
				dut.setPass(False, error_reason)
				# This dut got a cmd timeout error
				# We want to exclude it (i.e. don't send cmds to it anymore) because this probably means
				# the DUT will not answer anymore to the next cmds. Excluding the DUT saves timeouts
				# for the next cmds.
				self.excludeDut(dut)
			dut_set_results.append(DutSetResult(dut, result))

		return dut_set_results
		
		
	def sendCmdMultithreading(self,cmd,timeout_ms):
		dut_set_included = list()
		dut_set_results = list()
		res_queue = list()		
		dut_set_included = [dut for dut in self.__duts if dut not in self.__excluded_duts]
		thread_list = []
		def execute(dut,cmd,timeout_ms,queue):
			dut.sendCmd(cmd)
			try:
				result = dut.getResult(timeout_ms)
				logger.info("board <%s>: rc=%d" %
					(dut.getName(), result.rc) )
				if(result.rc != 0):
					raise ValueError("board <%s>: rc=%d" %
					(dut.getName(), result.rc) )
			except ValueError as e:
				error_reason = str(e)
				logger.info("board <%s>: command error (%s)" %
				(dut.getName(), error_reason))
				result = None
				dut.setPass(False, error_reason)
				self.excludeDut(dut)
			queue.put(DutSetResult(dut,result))
		
		try:
			for dut in dut_set_included:
				q = queue.Queue()
				res_queue.append(q)
				t = threading.Thread(target = execute,args =(dut,cmd,timeout_ms,q))
				thread_list.append(t)
			for t in thread_list:
				t.start()
		except:
			print("Error: unable to start thread")
		
		for q in res_queue:
			dut_set_results.append(q.get())
			
		return dut_set_results
	
		
# TODO : this class is not implemented
# Output example (CR added for convenience) :
# <li840>
# 	<data>
#		<celltemp>4.73654e1</celltemp>
#		<cellpres>1.00003e2</cellpres>
#		<co2>1.37793e3</co2>
#		<co2abs>2.0508551e-1</co2abs>
#		<h2o>9.95333</h2o>
#		<h2oabs>7.2898983e-2</h2oabs>
#		<h2odewpoint>6.84873</h2odewpoint>
#		<ivolt>2.3800659e1</ivolt>
#		<raw>
#			<co2>3051967</co2>
#			<co2ref>3834051</co2ref>
#			<h2o>2445989</h2o>
#			<h2oref>2419303</h2oref>
#		</raw>
#	</data>
# </li840>
class Co2Meter:
	#__uart
	__measblock_timeout_ms = int(configget('CO2Meter','measblock_timeout_ms'))			# Timeout to get a measure block
	__stab_timeout_ms = int(configget('CO2Meter','stab_timeout_ms'))			# Timeout for co2 ppm stabilization
	__sample_rate_hz = int(configget('CO2Meter','sample_rate_hz'))				# co2meter sends 2 samples per second
	__stab_nb_sample = (__sample_rate_hz * 7)	# Last 7 seconds of samples must match the stabilization criteria
	__stab_nb_sample_fast = (__sample_rate_hz * 1)	# Last 1 seconds of samples must match the stabilization criteria in fast mode
	__stab_tol_ratio = float(configget('CO2Meter','stab_tol_ratio'))			# last samples must be within +-0.5% of the mean
	__stab_tol_ppm = int(configget('CO2Meter','stab_tol_ppm'))			# last samples must be within +-10 ppm

	def __init__(self, uart_name = configget('CO2Meter','port')):
		self.__uart = serial.Serial()
		self.__uart.setPort(uart_name)
		self.__uart.setBaudrate(9600)
		self.__uart.setTimeout(0.1)
		self.__logfile = None
		# TODO: in some cases, the co2meter seems to send an incomplete block
		# Why ? are we misreading the uart ?
		# In the mean time, just don't try to find blocks with this simple regexp,
		# but use the complex "__meas_re" directly
		#self.__measblock_re = re.compile('<li840>.*?</li840>')	# '.*?' is the non-greedy version of '.*'
		self.__meas_re = re.compile('''<li840>.*?
						<data>
						.*?
						<co2>([0-9]+\.[0-9]+)(e([0-9]+))?</co2>
						.*?
						</data>
						.*?
					</li840>''', re.VERBOSE)

	def __log_open(self):
		# Create log file
		filename = 'co2meter.log'
		self.__logfile = open(filename, "a+b")

	def log(self, uart_bytes):
		# Strip "\n" to avoid "doubled" carriage returns
		stripped_uart_bytes = uart_bytes.replace('\n', '')
		self.__logfile.write(stripped_uart_bytes.encode())
					
	def open(self):
		self.__log_open()
		self.__uart.open()
		# TODO: initialize co2 meter settings
		# We use default settings for now...
		
	def close(self):
		if self.__uart.isOpen():
			self.__uart.close()

	def parse_ppm(self, measblock):
		meas = self.__meas_re.search(measblock)
		co2_ppm  =  1  * float(meas.group(1))
		if meas.group(3):
			co2_ppm *= 10 ** int(meas.group(3))
		return co2_ppm
	
	def read_ppm(self, fast_stab = False):
		text = ""
		measblock_time_start = time()
		stab_time_start = measblock_time_start
		co2_ppms = list()
		stab_nb_sample = self.__stab_nb_sample if not fast_stab else self.__stab_nb_sample_fast
		
		self.__uart.flushInput()
		if fast_stab:
			logger.info("Co2Meter RX Fast:")
		else:
			logger.info("Co2Meter RX :")
		logger.debug("-----------------------------")
		while True:
			chunk = self.__uart.read(512)
			# Do we need to decode/encode !?
			# Replace non-ascii characters because we sometimes receive - for a yet unknown reason -
			# non-ascii characters from the CO2 meter
			text += chunk.decode('ascii', 'replace')
			
			#measblock = self.__measblock_re.search(text)
			# Hack: see comment above "self.__measblock_re" declaration
			measblock = self.__meas_re.search(text)
			if measblock:
				measblock_time_start = time()
				self.log(measblock.group(0))
				co2_ppm = self.parse_ppm(measblock.group(0))
				#logger.debug("co2meter raw ppm = %.02f" % co2_ppm)
				co2_ppms.append(co2_ppm)
				text = text[measblock.end(0):]
				
				if len(co2_ppms) >= stab_nb_sample:
					#last_ppms = co2_ppms[-self.__stab_nb_sample:]
					last_ppms = co2_ppms[-stab_nb_sample:]
					#print last_ppms
					mean_ppm = sum(last_ppms) / float(len(last_ppms))
					min_ppm = min(last_ppms)
					max_ppm = max(last_ppms)
					error_abs = max_ppm - min_ppm
					error_ratio = (max_ppm - min_ppm) / mean_ppm
					logger.debug("co2meter raw ppm = %.02f, err_abs=%0.1f, err_ratio=%0.6f" % (co2_ppm, error_abs, error_ratio))
					if error_abs < self.__stab_tol_ppm:
						co2_ppm = last_ppms[-1]
						logger.info("co2meter stabilized ppm = %.02f (err_abs OK)" % co2_ppm)
						return co2_ppm
					if(error_ratio < self.__stab_tol_ratio):
						co2_ppm = last_ppms[-1]
						logger.info("co2meter stabilized ppm = %.02f (err_ratio OK)" % co2_ppm)
						return co2_ppm
				else:
					logger.debug("co2meter raw ppm = %.02f" % co2_ppm)

			now_time = time()
			measblock_elapsed_time = (now_time - measblock_time_start) * 1000
			'''if measblock_elapsed_time > self.__measblock_timeout_ms:
				raise ValueError('Co2Meter read measure block timeout')'''
			
			stab_elapsed_time = (now_time - stab_time_start) * 1000
			#logger.debug("stabilization elapsed time = %d" % stab_elapsed_time)
			if stab_elapsed_time > self.__stab_timeout_ms:
				raise ValueError('Co2Meter stabilization timeout')
				
class ITTDot:
	def __init__(self, time_ms, ppm):
		'''Describe a dot of an Injection Time Table
		   time_ms : (int) time during which the valve should be opened to reach the associated ppm
		   ppm     : ppm measured when opening the valvle for time_ms'''
		self.time_ms = time_ms
		self.ppm = ppm
		
	def __str__(self):
		return '[' + str(self.time_ms) + ' ms] -> [' + str(self.ppm) + ' ppm]'
		
		
class JigITT:
	'''Injection time table for Co2 and No2 gases of a Co2Jig.
	   This calibration table is intended to serve as a hint about how much time a valve should be opened to reach a given ppm value
	   knowing the current ppm inside the jig'''
	def __init__(self):
		self.__co2 = list()
		self.__no2 = list()
		self.air_Nb = 0
		
	def loadCo2FromRawMeasures(self, co2_ppms, co2_step_ms):
			
		self.__co2 = list()
		for index, ppm in enumerate(co2_ppms):
			inject_time = index*co2_step_ms
			dot = ITTDot(inject_time, ppm)
			self.__co2.append(dot)
		
		self.__co2 = sorted(self.__co2, key = lambda a:a.ppm)
		logger.debug("Loaded %d Co2 ITT values from raw measures" % len(self.__co2))
		
	def loadNo2FromRawMeasures(self, no2_ppms, no2_step_ms,air_step_ms):
			
		self.__no2 = list()
		logger.debug("In all %d Air Injection times" % self.air_Nb )
		for index, ppm in enumerate(no2_ppms):
			if index <= self.air_Nb:
				inject_time = index * air_step_ms
			else:
				inject_time = self.air_Nb * air_step_ms + (index - self.air_Nb) * no2_step_ms
			
			dot = ITTDot(inject_time, ppm)
			self.__no2.append(dot)
		#Sort in reverse order for N2 table
		self.__no2 = sorted(self.__no2, key = lambda a:a.ppm, reverse=True)
		logger.debug("Loaded %d No2 ITT values from raw measures" % len(self.__no2))
		
	def saveToFile(self, filename = 'inject_time_table.dat'):
		file = open(filename, 'wb')
		for dot in self.__co2:
			string = "co2 %d %d\n" % (dot.time_ms, dot.ppm)
			file.write(string.encode())
		for dot in self.__no2:
			string = "no2 %d %d\n" % (dot.time_ms, dot.ppm)
			file.write(string.encode())
		file.close()
		logger.info("Saved ITT calibration into file <%s>:" % filename)
		logger.info("-------------------")
		for line in str(self).splitlines():
			logger.info(line)
		logger.info("-------------------")
	
	def loadFromFile(self, filename = 'inject_time_table.dat'):
		co2 = list()
		no2 = list()
		dot_re = re.compile('^(co2|no2) ([0-9]+) ([0-9]+)$')
		file = open(filename, 'r')
		for line in file.readlines():
			res = dot_re.match(line)
			if not res:
				raise ValueError('%s: Invalid file format' % filename)
			dot = ITTDot(int(res.group(2)), int(res.group(3)))
			if res.group(1) == 'co2':
				co2.append(dot)
			elif res.group(1) == 'no2':
				no2.append(dot)
			else:
				raise ValueError('%s: Invalid file format' % filename)
		self.__co2 = co2
		self.__no2 = no2
		logger.info("Loaded ITT calibration from file <%s> (dots: co2=%d, no2=%d):"
				% (filename, len(self.__co2), len(self.__co2)))
		logger.info("-------------------")
		for line in str(self).splitlines():
			logger.info(line)
		logger.info("-------------------")
	
	def getGasInjectionTime(self, gas_name, current_ppm, target_ppm):
		pass
		
	def getCo2InjectionTime(self, current_ppm, target_ppm):
		co2 = self.__co2
		def dist_ppm(ppm_a, ppm_b):
			return abs(ppm_a - ppm_b)
			
		# Interpolation of current time
		idx_a = 0
		val_a = co2[0].ppm
		idx_b = None
		val_b = None
		for index, dot in enumerate(co2):
			if dist_ppm(dot.ppm, current_ppm) < dist_ppm(val_a, current_ppm):
				idx_a = index
				val_a = dot.ppm
		if val_a < current_ppm:
			idx_b = idx_a+1
			if idx_b >= len(co2):
				idx_b = idx_a-1
		else:
			idx_b = idx_a-1
			if (idx_b < 0):
				idx_b = idx_a+1	
		if not idx_a < idx_b:
			# Swap
			(idx_a, idx_b) = (idx_b, idx_a)
			(val_a, val_b) = (val_b, val_a)
		dot_a = co2[idx_a]
		dot_b = co2[idx_b]
		#current_time = dot_a.time_ms + (dot_b.time_ms - dot_a.time_ms) * ((current_ppm - dot_a.ppm) / (dot_b.ppm - dot_a.ppm))
		current_time = dot_a.time_ms + (current_ppm - dot_a.ppm) * (float(dot_b.time_ms - dot_a.time_ms) / float(dot_b.ppm - dot_a.ppm))
		
		# Interpolation of target time
		# TODO: hey ! this is a nice copy/paste of above...
		idx_a = 0
		val_a = co2[0].ppm
		idx_b = None
		val_b = None
		for index, dot in enumerate(co2):
			if dist_ppm(dot.ppm, target_ppm) < dist_ppm(val_a, target_ppm):
				idx_a = index
				val_a = dot.ppm
		if val_a < target_ppm:
			idx_b = idx_a+1
			if idx_b >= len(co2):
				idx_b = idx_a-1		
		else:
			idx_b = idx_a-1
			if (idx_b < 0):
				idx_b = idx_a+1	
		if not idx_a < idx_b:
			# Swap
			(idx_a, idx_b) = (idx_b, idx_a)
			(val_a, val_b) = (val_b, val_a)
		dot_a = co2[idx_a]
		dot_b = co2[idx_b]
		#target_time = dot_a.time_ms + (dot_b.time_ms - dot_a.time_ms) * ((target_ppm - dot_a.ppm) / (dot_b.ppm - dot_a.ppm))
		target_time = dot_a.time_ms + (target_ppm - dot_a.ppm) * (float(dot_b.time_ms - dot_a.time_ms) / float(dot_b.ppm - dot_a.ppm))
		delta_time = target_time - current_time
		logger.debug("Interpolate C02 injection time for %d ppm to %d ppm:"
				% (current_ppm, target_ppm))
		logger.debug("  interpolated time=%d ms to %d ms -> delta=%d ms"
				% (current_time, target_time, delta_time))
		if(delta_time < 0):
			raise ValueError("Injection time can't be negative !")
		return delta_time
	
	# TODO: this is also a big copy/paste of getCo2InjectionTime
	def getNo2InjectionTime(self, current_ppm, target_ppm):
		no2 = self.__no2
		def dist_ppm(ppm_a, ppm_b):
			return abs(ppm_a - ppm_b)
		
		# Interpolation of current time
		idx_a = 0
		val_a = no2[0].ppm
		idx_b = None
		val_b = None
		for index, dot in enumerate(no2):
			if dist_ppm(dot.ppm, current_ppm) < dist_ppm(val_a, current_ppm):
				idx_a = index
				val_a = dot.ppm
		if val_a < current_ppm:
			idx_b = idx_a-1
			if (idx_b < 0):
				idx_b = idx_a+1
		else:
			idx_b = idx_a+1
			if idx_b >= len(no2):
				idx_b = idx_a-1

		if not idx_a < idx_b:
			# Swap
			(idx_a, idx_b) = (idx_b, idx_a)
			(val_a, val_b) = (val_b, val_a)
		dot_a = no2[idx_a]
		dot_b = no2[idx_b]
		current_time = dot_a.time_ms + (current_ppm - dot_a.ppm) * (float(dot_b.time_ms - dot_a.time_ms) / float(dot_b.ppm - dot_a.ppm))	
		
		# Interpolation of target time
		# TODO: hey ! this is a nice copy/paste of above...
		idx_a = 0
		val_a = no2[0].ppm
		idx_b = None
		val_b = None
		for index, dot in enumerate(no2):
			#print index, dot.ppm
			if dist_ppm(dot.ppm, target_ppm) < dist_ppm(val_a, target_ppm):
				idx_a = index
				val_a = dot.ppm
		#print "closest idx for %d ppm: %d" % (target_ppm, idx_a)
		if val_a < target_ppm:
			idx_b = idx_a-1
			if (idx_b < 0):
				idx_b = idx_a+1

		else:
			idx_b = idx_a+1
			if idx_b >= len(no2):
				idx_b = idx_a-1

		if not idx_a < idx_b:
			# Swap
			(idx_a, idx_b) = (idx_b, idx_a)
			(val_a, val_b) = (val_b, val_a)
		dot_a = no2[idx_a]
		dot_b = no2[idx_b]
		#print "target dot indexes: %d, %d" % (idx_a, idx_b)
		#print("%d + (%d - %d) * ((%d - %d) / (%d - %d))", 
		#	dot_a.time_ms, target_ppm, dot_a.ppm, dot_b.time_ms, dot_a.time_ms, dot_b.ppm, dot_a.ppm)
		target_time = dot_a.time_ms + (target_ppm - dot_a.ppm) * (float(dot_b.time_ms - dot_a.time_ms) / float(dot_b.ppm - dot_a.ppm))
		delta_time = target_time - current_time
		logger.debug("Interpolate N02 injection time for %d ppm to %d ppm:"
				% (current_ppm, target_ppm))
		logger.debug("  interpolated time=%d ms to %d ms -> delta=%d ms"
				% (current_time, target_time, delta_time))
		if(delta_time < 0):
			raise ValueError("Injection time can't be negative !")
		return delta_time
		
	def __str__(self):
		result = "CO2 (%d dots):\n" % len(self.__co2)
		for dot in self.__co2:
			result += "  " + str(dot) + "\n"
		result += "NO2 (%d dots):\n" % len(self.__no2)
		for dot in self.__no2:
			result += "  " + str(dot) + "\n"
		return result
		
	
				
class Co2Jig:
	__gas_out_delay_ms = int(configget('Co2Jig','gas_out_delay_ms'))	# Overpressure avoidance delay
	__inject_loop_maxtry = int(configget('Co2Jig','inject_loop_maxtry'))	# Allow up to 5 gas injections before considering we can't reach the ppm target
	__valve_min_time_ms = int(configget('Co2Jig','valve_min_time_ms'))	# Minimum opening time for the valve
	__dut_stab_time_ms = int(configget('Co2Jig','dut_stab_time_ms'))	# Minimum time to wait after gas injection so that the gas concentration is stabilized inside dut sensor
	__dilution_threshold = int(configget('Co2Jig','dilution_threshold')) # threshold for decide using N2 or fresh air
	
	def __init__(self):
		self.__relayboard = RelayBoard()
		self.__relayboard.disableAllRelays()
		self.__co2meter = Co2Meter()
		self.__itt = JigITT()
	
	def injectGas(self, no2, time_ms):
		relayboard = self.__relayboard
			
		if(no2):
			relay_gas_in = relayboard.relay_gas_no2
		else:
			relay_gas_in = relayboard.relay_gas_co2
		
		#relayboard.enableRelay(relayboard.relay_gas_out)	# Open gas out
		#sleep(self.__gas_out_delay_ms / 1000.0)
		
		relayboard.enableRelay(relay_gas_in)
		sleep(time_ms / 1000.0)
		relayboard.disableRelay(relay_gas_in)
		
		#sleep(self.__gas_out_delay_ms / 1000.0) # Close gas out
		#relayboard.disableRelay(relayboard.relay_gas_out)
	
	def injectAir(self,time_ms):  # when co2ppm > threshold, use air to dilute the co2
		logger.info("Inject Air for %d ms", time_ms)
		relayboard = self.__relayboard
		relay_air_in = relayboard.relay_gas_air
		
		relayboard.enableRelay(relay_air_in)
		sleep(time_ms / 1000.0)
		relayboard.disableRelay(relay_air_in)
	
	def injectNO2(self, time_ms):
		logger.info("Inject NO2 for %d ms", time_ms)
		self.injectGas(True, time_ms)
		
	def injectCO2(self, time_ms):
		logger.info("Inject CO2 for %d ms", time_ms)
		self.injectGas(False, time_ms)

	def injectForDot(self, dot, cur_ppm = None):
		'''dot: CalDot object
		   cur_ppm: current co2 ppm level in the jig
			    This help to co2 measurement time.
		   Inject gas until the co2 level described by the CalDot is reached, or "timeout"'''
		co2meter = self.__co2meter
		itt = self.__itt
		maxtry = self.__inject_loop_maxtry
		target_ppm = dot.co2_ppm
		try_cnt = 0
		post_inject_time = time()
		logger.info("Inject gas for target=%d ppm +-%d"
				 % (dot.co2_ppm, dot.co2_ppm_tol))

		if cur_ppm == None:
			cur_ppm = co2meter.read_ppm()

		while True:
			level = dot.refCompareTol(cur_ppm)
			if level == 0:
				break

			try_cnt += 1
			if try_cnt > maxtry:
				msg = 'Fail to reach co2 ppm target=%d ppm. Jig recalibration needed.' % target_ppm
				logger.warn(msg)
				msg = 'Error 50100'
				logger.warn(msg)
				raise ValueError(msg)

			if level < 0:
				# Co2 level too low, inject some Co2
				co2_time = itt.getCo2InjectionTime(cur_ppm, target_ppm)
				if(co2_time < self.__valve_min_time_ms):
					logger.warn("Should open co2 valve for %d ms, but min_time=%d ms" % (co2_time, self.__valve_min_time_ms))
					co2_time = self.__valve_min_time_ms
				self.injectCO2(co2_time)
				post_inject_time = time()
			elif level > 0:  
				# Co2 level too high, inject some No2
				no2_time = itt.getNo2InjectionTime(cur_ppm, target_ppm)
				if(cur_ppm > self.__dilution_threshold):
					self.injectAir(no2_time)
				else:
					self.injectNO2(no2_time)
				post_inject_time = time()

			cur_ppm = co2meter.read_ppm()
		post_inject = (time()) - post_inject_time
		logger.debug("Injection Time: %d ms" % (post_inject * 1000))
		dut_stab_delay = self.__dut_stab_time_ms - post_inject * 1000
		logger.debug("(debug) Wait for DUT ppm stabilization: %d ms" % dut_stab_delay)
		if dut_stab_delay > 0:
			logger.info("Wait for DUT ppm stabilization: %d ms" % dut_stab_delay)
			sleep(dut_stab_delay / 1000)
			logger.info("Read co2 ppm after DUT stabilization delay")
			cur_ppm = co2meter.read_ppm(fast_stab=True)
		return cur_ppm

	def run_test(self, nb_dut, no_cal = False):
		relayboard = self.__relayboard
		co2meter = self.__co2meter
		itt = self.__itt	
		dutset = DutSet(nb_dut)
		
		if no_cal:
			logger.info("DUT calibration disabled")
		
		try:
			# relayboard.enableRelay(relayboard.relay_gas_no2)
			# while True:
				# pass
			
			#co2meter.open()
			relayboard.powerPump(True)
			relayboard.powerFan(True)
			
			#print "ppm=%d" % co2meter.read_ppm()
			
			itt.loadFromFile()
			relayboard.powerDutSet(True)
			logger.info('Duts power-on delay...')
			sleep(7) # Scale boot time is ~7 seconds
			dutset.open()
			relayboard.powerFan(True)

			# Disable timelimit
			dutset.sendCmd("timelimit off", 5000)
			
			# Disable debug traces
			trace = int(configget('Calib','Trace_on'))
			if  trace == 1:
				dutset.sendCmd("trace on", 5000)
			else:
				dutset.sendCmd("trace off", 5000)
			
			# Send T3 station CO2 commands to get these values after tube gluing
			dutset.sendCmd("co2 get_tr0_tp0_photo 1", 10000)
			dutset.sendCmd("co2 get_tr0_tp0_photo 0", 10000)
			
			if not no_cal:
				# Do aging of CO2 lamp for ~12 seconds with calib fast timings
				for _ in range(2):
					dutset.sendCmd("co2 calib 100 252 100 252 5 0 0 0.45 1", 30000)
				
				####### Erase calibration table
				# dutset.sendCmd("perso del_co2cal", 10000)
				# dutset.sendCmd("perso del_co2cal_fast", 10000)
				# dutset.sendCmd("perso del_co2cal_veryfast", 10000)
				####### Erase verification table
				# dutset.sendCmd("perso del_co2verif", 10000)
				# dutset.sendCmd("perso del_co2verif_fast", 10000)
				# dutset.sendCmd("perso del_co2verif_veryfast", 10000)


			
			# cal_dot_cnt = 0
			# verif_dot_cnt = 0
			# ref_ppm = None
			# for dot in CalSettings.cal_dots:
			
				# if no_cal and dot.dut_tol_coef == None:
					# logger.info("Skip calibration dot %d ppm..." % dot.co2_ppm)
					# continue
					
				# ref_ppm = self.injectForDot(dot, ref_ppm)
				# logger.info("Got %d ppm for target %d +-%d ppm" % (ref_ppm, dot.co2_ppm, dot.co2_ppm_tol))
					
				# if dot.dut_tol_coef == None:
					######### Calibration : fast
					# ref_ppm = co2meter.read_ppm(fast_stab=True)
					# logger.info("Calibrate DUT for FAST, target %d ppm (ref_ppm=%d)" % (dot.co2_ppm, ref_ppm))
					# cmd = "co2 calib 100 252 100 252 5 %d %d 0.45 1" % (
							# cal_dot_cnt,
							# ref_ppm)
					# if trace == 1:
						# dutset.sendCmdTraceon(cmd, 30000)
					# else:
						# dutset.sendCmd(cmd, 30000)
					
					# cal_dot_cnt += 1
				# else:
					######## Verification : fast
					# ref_ppm = co2meter.read_ppm(fast_stab=True)
					# logger.info("Verify DUT for FAST, target %d ppm (ref_ppm=%d)" % (dot.co2_ppm, ref_ppm))
					# cmd = "co2 verif %d %d 1" % (
							# verif_dot_cnt,
							# ref_ppm)
					# if trace == 1:
						# cmdRes = dutset.sendCmdTraceon(cmd, 30000)
					# else:
						# cmdRes = dutset.sendCmd(cmd, 30000)

					## Check verification 1 : fast
					# for res in cmdRes:
						########## The result is None if the DUT failed to reply correctly to the cmd
						########## Just ignore this DUT in this case (DUT has already been setPass() to False)
						# if res.cmd_result == None:
							# logger.info("Verif FAILED on <%s> for FAST: cmd error" % (res.dut.getName()))
							# continue

						# dut_ppm = int(res.cmd_result.data['co2_ppm_verif'])
						# if dot.dutMatchTol(ref_ppm, dut_ppm):
							# logger.info("Verif OK on <%s> for FAST: expected %d ppm, got %d ppm (err=%0.3f, max_err +-%0.3f)" % (
									# res.dut.getName(),
									# ref_ppm,
									# dut_ppm,
									# dot.dutTolError(ref_ppm, dut_ppm),
									# dot.dut_tol_coef
									# )
								# )
						# else:
							# logger.info("Verif FAILED on <%s> for FAST: expected %d ppm, got %d ppm (err=%0.3f, max_err +-%0.3f)" % (
									# res.dut.getName(),
									# ref_ppm,
									# dut_ppm,
									# dot.dutTolError(ref_ppm, dut_ppm),
									# dot.dut_tol_coef
									# )
								# )
							########### Set DUT as FAILED
							# res.dut.setPass(False, "FAST verification")					
					# verif_dot_cnt += 1
					

			# Read back the calibration tables
			cmd = "perso get_co2cal_fast"
			dutset.sendCmd(cmd, 5000)
			cmd = "perso get_co2verif_fast"
			dutset.sendCmd(cmd, 5000)
			
			
			# Set PASS for duts which are still in "beeing tested" state
			for dut in dutset.getDuts():
				dut.sendCmd("trace off")
				if dut.getPass() == None:
					dut.setPass(True)
					
				logger.info("Dut <%s> is %s" % (
						dut.getName(),
						"PASS" if dut.getPass() else "FAIL"
						)
					)
			
			logger.info("Test OK")
			
		finally:
			relayboard.disableAllRelays()
			dutset.close()
			#co2meter.close()
			self.saveFactoryReport(dutset.getDuts())
			self.saveCSVReport(dutset.getDuts())
		
	def run_calib(self):
		relayboard = self.__relayboard
		co2meter = self.__co2meter
		itt = self.__itt

		co2_step_ms = int(configget('Calib','co2_step_ms'))
		no2_step_ms = int(configget('Calib','no2_step_ms'))
		air_step_ms = int(configget('Calib','air_step_ms'))
		
		co2_ppms = list()
		no2_ppms = list()
		
		# # Debug 1 -------------------
		# #Save jig calibration
		# if False:
			# co2_ppms.append(250)
			# co2_ppms.append(500)
			# co2_ppms.append(700)
			# no2_ppms.append(4000)
			# no2_ppms.append(2000)
			# no2_ppms.append(300)
			# logger.info("Save injection time table...")
			# self.__itt.loadCo2FromRawMeasures(co2_ppms, co2_step_ms)
			# self.__itt.loadNo2FromRawMeasures(no2_ppms, no2_step_ms)
			# self.__itt.saveToFile()
		# else:
			# self.__itt.loadFromFile()
			# # self.__itt.getCo2InjectionTime(500, 1000)
			# # self.__itt.getCo2InjectionTime(1000, 2000)
			# # self.__itt.getCo2InjectionTime(0, 500)
			# # self.__itt.getNo2InjectionTime(4000, 1000)
			# # self.__itt.getNo2InjectionTime(3000, 2000)
			# self.__itt.getNo2InjectionTime(1000, 0)
		
		# raise ValueError("end of debug")
		# # End Debug 1 ------------------


		co2meter.open()
		relayboard.powerPump(True)
		relayboard.powerFan(True)
		try:
			# Idea 1: record 2 curves :
			# 1 curve for co2 injection,
			# 1 curve for no2 injection
			# Do co2 1st:
			# - inject no2 until co2 ~=0 ppm
			# - inject co2 by steps of 100 ms, until we reach the highest possible concentration in the cal_dots table
			# Then do no2:
			# - inject no2 by steps of 1s until we reach co2 ~= 0 ppm.
			#   1s is okay because no2 is 'cheaper' than co2 and is very concentrated
			#   with respected to co2.
			#
			# Then, with these 2 curve, we can 'guess' how much co2 (or no2) we should inject to reach
			# the next point in cal_dots table, whatever the actual co2 concentration is.

			cal_dots = CalSettings.cal_dots
			cal_dot_0ppm = CalSettings.cal_dots[0]
			cal_dot_maxppm = cal_dot_0ppm
			for dot in cal_dots:
				if dot.co2_ppm > cal_dot_maxppm.co2_ppm:
					cal_dot_maxppm = dot
			
			ppm_lower_target = cal_dot_0ppm.co2_ppm + (cal_dot_0ppm.co2_ppm_tol / 2)
			#ppm_upper_target = cal_dot_maxppm.co2_ppm + (cal_dot_maxppm.co2_ppm_tol * 2)
			ppm_upper_target = cal_dot_maxppm.co2_ppm * 2
			logger.info("Calibrate jig from 0ppm (<%dppm) to %dppm (actually %dppm)"
					% (ppm_lower_target, cal_dot_maxppm.co2_ppm, ppm_upper_target) )
			logger.info("Co2 precision = %d ms ; NO2 precision = %d ms"
					% (co2_step_ms, no2_step_ms) )
			
			skip_0ppm_init = False
			if not skip_0ppm_init:
				#Initial situation : 0 ppm
				logger.info("Settle 0ppm...")
				ppm = 0
				while True:
					#self.injectNO2(30000)
					self.injectNO2(20000)
					ppm = co2meter.read_ppm()
					if cal_dot_0ppm.refCompareTol(ppm) <= 0:
						break
			else:
				ppm = co2meter.read_ppm()
			
			#Skip the 0ppm
			#ppm = co2meter.read_ppm()
			
			# One measure every co2_step_ms, until we reach the highest
			# ppm we want to calibrate
			logger.info("Calibrate CO2 injection time...")
			co2_ppms.append(ppm)
			ppm = 0
			while ppm < ppm_upper_target:
				self.injectCO2(co2_step_ms)
				ppm = co2meter.read_ppm()
				co2_ppms.append(ppm)
				
			# One measure every no2_step_ms, until we reach back ~0ppm
			logger.info("Calibrate NO2 injection time...")
			no2_ppms.append(ppm)
			print("ppm=%d, ppm_lower_target=%d" % (ppm, ppm_lower_target))
			while ppm > ppm_lower_target:
				if(ppm > self.__dilution_threshold):
					self.injectAir(air_step_ms)
					self.__itt.air_Nb += 1
				else:
					self.injectNO2(no2_step_ms)
					
				ppm = co2meter.read_ppm()
				no2_ppms.append(ppm)
				print("ppm=%d, ppm_lower_target=%d" % (ppm, ppm_lower_target))
			
			# Save jig calibration
			logger.info("Save injection time table...")
			itt.loadCo2FromRawMeasures(co2_ppms, co2_step_ms)
			itt.loadNo2FromRawMeasures(no2_ppms, no2_step_ms,air_step_ms)
			itt.saveToFile()
			
			logger.info("Jig calibration OK")
		
		finally:
			relayboard.disableAllRelays()
			co2meter.close()
						
			# # Idead 2:
			# # Assume all dots are ordered by co2_ppm (the lowest to the highest concentration.
			# # 0ppm : assume this is the 1st point.
			# #        first, calibrate it roughly because the actual concentration may be quite different
			# #        from the concentration
			# #        Will will recalibrate it after the end of co2 calibration (the co2 concentration will be the maximum)
			# # >0ppm : inject co2 by steps of 100ms and record the total injection time for each dot
					
					
	def saveCSVReport(self, duts):
		global time_begin
		
		stationName = configget("TEST_STATION","STATION_NAME")
		stationNumber = configget("TEST_STATION","STATION_NUMBER")
		full_station_name = stationName+ "_" + stationNumber
		shift= configget("TEST_STATION","SHIFT")		
		factory = configget("INFO","FACTORY")
		project = configget("INFO","PROJECT")  #PROJECT values: 'ws30','ws50','hwa01'
		day = strftime("20%y%m%d",localtime())
		
		#csv file name: project_factory_log_stationname_stationnumber_date_shift.csv 
		csv_path =  project+ '_' +factory +'_log_' + full_station_name + '_' + day + '_' + shift + ".csv"
		
		#get local time
		time_end = time()
		time_total = round((time_end - time_begin),2)
		time_begin_= strftime("20%y%m%dt%H%M%S",localtime(time_begin))
		
		# Headers of the CSV LOG
		csv_title = "model,station,station number,program version,start time,duration,mac,secret,mfgid,bl version,fw version,station result"
		csv_title_ = csv_title.split(',')
		
		for i in range(61):			# CO2 Cal table and Verif table in all have 61 items
			csv_title_.append('Item to Test')
			csv_title_.append('Value')
		
		# One batch share the same info
		test_version = configget("TEST_STATION","TEST_VERSION")
		csvinfo = [project, stationName, stationNumber, test_version, time_begin_, str(time_total)]

		
		# Create csv report, if exist, then append mode
		if os.path.isfile(csv_path) == False:
			f = open(csv_path, "w",newline = '')
			csv_log = csv.writer(f, delimiter=',',quoting=csv.QUOTE_MINIMAL)
			csv_log.writerow(csv_title_)
		
		else:
			f = open(csv_path, "a",newline = '')
			csv_log = csv.writer(f, delimiter=',',quoting=csv.QUOTE_MINIMAL)
				
		for dut in duts:
			# Probe info
			dutProbe = dut.getInfo()
			csvline = [dutProbe['mac_label'],dutProbe['dut_secret'],dutProbe['dut_mfgid'],
				dutProbe['dut_bootloader_version'], dutProbe['dut_firmware_version']]
			toWrite = csvinfo + csvline
			
			toRecord = []
			# Result PASS or FAIL
			station_result = 'Fail'
			if dut.getPass():
				station_result = 'PASS'
				toRecord.append(station_result)    # Add station result
				toRecord.append("perso get_co2cal_fast")   # Add "get_co2cal_fast" cmd
				cal,verif = dut.get_cal_verif()
				print(len(verif),len(cal))
				
				toRecord.append(cal[-1][1]) # put rc = 0 as the value next to "get_co2cal_fast" cmd
				
				# Adding all the items in the cal table
				for i in range(len(cal)-1):
					toRecord.append(cal[i][0])
					toRecord.append(cal[i][1])
				# same thing for the verif table, in order like "cmd, result, item, value, item, value, item, value........"
				toRecord.append("perso get_co2verif_fast")
				toRecord.append(verif[-1][1]) # rc = 0			
				for i in range(len(verif)-1):
					toRecord.append(verif[i][0])
					toRecord.append(verif[i][1])
				
			else:
				toRecord.append(station_result)
			# Other Item
			toWrite += toRecord
			csv_log.writerow(toWrite)
		
		
		f.close()
	def saveCSVReportFromLog(self, duts):
		global time_begin
		
		stationName = configget("TEST_STATION","STATION_NAME")
		stationNumber = configget("TEST_STATION","STATION_NUMBER")
		full_station_name = stationName+ "_" + stationNumber
		shift= configget("TEST_STATION","SHIFT")		
		factory = configget("INFO","FACTORY")
		project = configget("INFO","PROJECT")  #PROJECT values: 'ws30','ws50','hwa01'
		day = strftime("20%y%m%d",localtime())
		
		#csv file name: project_factory_log_stationname_stationnumber_date_shift.csv 
		csv_path =  project+ '_' +factory +'_log_' + full_station_name + '_' + day + '_' + shift + ".csv"
		
		#get local time
		time_end = time()
		time_total = round((time_end - time_begin),2)
		time_begin_= strftime("20%y%m%dt%H%M%S",localtime(time_begin))
		
		# Headers of the CSV LOG
		csv_title = "model,station,station number,program version,start time,duration,mac,secret,mfgid,bl version,fw version,station result"
		csv_title_ = csv_title.split(',')
		
		for i in range(61):			# CO2 Cal table and Verif table in all have 61 items
			csv_title_.append('Item to Test')
			csv_title_.append('Value')
		
		# One batch share the same info
		test_version = configget("TEST_STATION","TEST_VERSION")
		csvinfo = [project, stationName, stationNumber, test_version, time_begin_, str(time_total)]

		
		# Create csv report, if exist, then append mode
		if os.path.isfile(csv_path) == False:
			f = open(csv_path, "w",newline = '')
			csv_log = csv.writer(f, delimiter=',',quoting=csv.QUOTE_MINIMAL)
			csv_log.writerow(csv_title_)
		
		else:
			f = open(csv_path, "a",newline = '')
			csv_log = csv.writer(f, delimiter=',',quoting=csv.QUOTE_MINIMAL)
		
		re_cmd = re.compile( 	'''shell>.*?rc=0''', 
						re.MULTILINE| re.DOTALL)
						
		for dut in duts:
			# toWrite = []
			dutProbe = dut.getInfo()
			csvline = [dutProbe['mac_label'],dutProbe['dut_secret'],dutProbe['dut_mfgid'],
			dutProbe['dut_bootloader_version'], dutProbe['dut_firmware_version']]
			toWrite = csvinfo + csvline
			
			station_result = 'Fail'
			if dut.getPass():
				station_result = 'PASS'
			
			toWrite.append(station_result)
			
			
			logpath = dut.getLog()
			res = []
			file = open(logpath, 'r')
			tt = file.read()#.replace('\n','')
			pos = 0
			while re_cmd.search(tt) != None:
				block = re_cmd.search(tt)
				pos = block.end()
				tt = tt[pos:]
				res.append(block.group(0))
			allres = []
			for part in res:
				cmdkey = []
				for o in part.splitlines():
					keyvalues = o.split("=",1)
					if len(keyvalues) < 2:
						cmdkey.append(['Command name',keyvalues[0][6:-1]])
					else:
						cmdkey.append(keyvalues)
						
				if len(cmdkey) >2:
					allres.append([[cmdkey[0][1],cmdkey[-1][1]]] + cmdkey[1:-1]) 
				else:
					allres.append([[cmdkey[0][1],cmdkey[1][1]]])
			
			allres = allres[17:]
			for item in allres:
				for i in item:
					toWrite.append(i[0])
					toWrite.append(i[1])
			
			
			csv_log.writerow(toWrite)
		
		f.close()
		
		
	def saveFactoryReport(self, duts):					
		#################################################################
		#######             Create report file     		#################
	
		file = open('MAC_CO2_RESULTS.txt', "w", newline='')	# csv module requires "binary" format (Not any more, for Python3.x ,)
		report = csv.writer(file, delimiter=',', quoting=csv.QUOTE_MINIMAL)

		# Write report
		for dut in duts:
			# 1st Col: Mac (or slot id)
			mac = dut.getMac()
			if not mac:
				mac = dut.getName()
			# 2nd Col: Pass/Fail/Untested
			state = 'Fail'			
			if dut.getPass() == None:
				state = 'Untested'
			elif dut.getPass():
				state = 'Pass'
			# Append in report
			report.writerow([mac,state])
					
		file.close()
			

def usage():
	print("Usage:\n")
	print("run_test <nb_duts> [<-nocal>]\n"	\
		"Run DUT test (calibration, and verification)\n" \
		"	nb_duts: number of DUTs plugged into the jig, fro; left to right (1->16)" \
		"	-nocal: disable dut calibration\n")
	print("run_calib\n" \
		"	Calibrate the JIG for calve operture times\n")
	print("relay <list|set|reset> <relay_name>" \
		"	Control relays (debug)")
	sys.exit(-1)

def init_logger(log_name):
	global logger
	# Initialize log system
	# set up logging to file - see previous section for more details
	logging.basicConfig(level=logging.DEBUG,
			    format='%(asctime)s %(name)-8s %(levelname)-8s %(message)s',
			    #datefmt='%m-%d %H:%M',
			    filename=log_name,
			    filemode='a')
	# define a Handler which writes INFO messages or higher to the sys.stderr
	console = logging.StreamHandler()
	console.setLevel(logging.DEBUG)
	# set a format which is simpler for console use
	formatter = logging.Formatter('%(asctime)s %(name)-12s: %(levelname)-8s %(message)s')
	# tell the handler to use this format
	console.setFormatter(formatter)
	# add the handler to the root logger
	logging.getLogger().addHandler(console)
	console = None
	formatter = None
	logger = logging.getLogger('co2')



def main():	
	global time_begin 
	argv = sys.argv
	if len(argv) < 2:
		usage()
	
	time_begin = time()
	
	init_logger('co2_calib_cal.log')

	try:
		logger.info(software_version)
		if argv[1] == 'run_test':
			skipcal = False
			nb_dut = int(argv[2])
			if len(argv) >= 4:
				print("argv[2] = <%s>" % argv[2])
				if argv[3] == '-nocal':
					skipcal = True
				else:
					print("Invalid argument dor 'run_test': %s" % argv[2])
					usage()
			jig = Co2Jig()
			jig.run_test(nb_dut, skipcal)
			
		elif argv[1] == 'run_calib':
			jig = Co2Jig()
			jig.run_calib()
			
		elif argv[1] == 'relay':
			if argv[2] == 'list':
				for relay in RelayBoard.relays:
					print('%s (id %d)' % (relay.name, relay.id))
			elif argv[2] == 'set':
				for relay in RelayBoard.relays:
					if relay.name == argv[3]:
						RelayBoard().setRelay(relay, True)
						return
				logger.info("Relay <%s> does not exist" % argv[3])
			elif argv[2] == 'reset':
				for relay in RelayBoard.relays:
					if relay.name == argv[3]:
						RelayBoard().setRelay(relay, False)
						return
				logger.info("Relay <%s> does not exist" % argv[3])
			else:
				usage()
			sys.exit(0)
			
		elif argv[1] == 'co2':
			co2meter = Co2Meter()
			co2meter.open()
			logger.info("co2_ppm=%0.2f" % co2meter.read_ppm())
			co2meter.close()
		else:
			usage()
	except:
		logger.exception('Got exception in main')
		raise

if __name__ == "__main__":
	rc = main()
	sys.exit(rc)
