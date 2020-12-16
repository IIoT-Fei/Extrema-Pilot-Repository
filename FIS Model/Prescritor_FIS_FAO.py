#!/usr/bin/python
# -*- coding: utf-8 -*-
#Elaborado por Gilberto Souza em Novembro/2019 - atualizado em Novemmbro/2020
#Aquisicao de dados dos sensores de umidade, temperatura do ar e do solo
#Aquisicao dos dados das estações PWS FEI, SBC, Extrema
#Aquisicao da previsao de chuvas para d+5
#Aquisicao de dados do sensor de iluminancia e GPS
#Gravacao em arquivo .CSV e no banco de dados
#Determinação da irrigação diária e turno de irrigação (FAO e Fuzzy)

tempo = 1
import sys
import requests
import math
import serial
import Adafruit_DHT
import RPi.GPIO as GPIO
import time
import smbus
import Adafruit_ADS1x15

from w1thermsensor import W1ThermSensor
from datetime import datetime, timedelta, date
from datetime import timedelta, date
import os
import glob
import requests

import gps

import csv
import json

import I2C_LCD_driver
import socket
import fcntl
import struct

import pymysql

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

from Adafruit_HTU21D.HTU21D import HTU21D
sensor_UHT = HTU21D()

#GPIO.setmode(GPIO.BCM)
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

#Botoes de comando
GPIO.setup(37,GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
GPIO.setup(35,GPIO.IN)
GPIO.setup(33,GPIO.IN)

#Valvulas
GPIO.setup(21,GPIO.OUT)
GPIO.setup(23,GPIO.OUT)

#Pinos NOK na Rasp
GPIO.setup(32,GPIO.OUT)
GPIO.setup(36,GPIO.OUT)
GPIO.setup(38,GPIO.OUT)
GPIO.setup(40,GPIO.OUT)

#Sinotico LEDs
GPIO.setup(29,GPIO.OUT)
GPIO.setup(31,GPIO.OUT)
GPIO.setup(16,GPIO.OUT)
GPIO.setup(19,GPIO.OUT)
GPIO.setup(15,GPIO.OUT)

#Retorno do Conversor logico
GPIO.setup(26,GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(24,GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

GPIO.setup(31,GPIO.OUT)

#Retorno do Fluxometro
GPIO.setup(13,GPIO.IN, pull_up_down = GPIO.PUD_UP)

GPIO.output(29,0)
GPIO.output(31,0)
GPIO.output(16,0)
GPIO.output(19,0)
GPIO.output(15,0)
GPIO.output(23,0)
GPIO.output(21,0)

constante = 0.10
tempo_novo = time.time()
global contador_pulsos
contador_pulsos = 0
global count

lcdi2c = I2C_LCD_driver.lcd()

global bt_valv_A
global bt_valv_B
global bt_modo
global i_auto

bt_valv_A = GPIO.input(33)
bt_valv_B = GPIO.input(37)
bt_modo = GPIO.input(35)

#Exibe informacoes iniciais
print("Inicializando")
lcdi2c.lcd_display_string("Inicializando", 1,0)
time.sleep(2)
lcdi2c.lcd_clear()

os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

base_dir = '/sys/bus/w1/devices/'

ser = serial.Serial(port='/dev/ttyS0', baudrate = 9600, timeout = 2)

sensor = Adafruit_DHT.AM2302
adc1= Adafruit_ADS1x15.ADS1115(address=0x48)
adc = Adafruit_ADS1x15.ADS1115(address=0x49)

sensor_temp_solo = W1ThermSensor()

pino_sensor = 17

lat = ''
lon = ''

Ganho_ADC1 = 1
Ganho_ADC = 1

#Variáveis temporárias

# Variáveis para cálculo da ETo
dia_corr = 0
n = 0
tmax = 0
tmin = 0
tmed = 0
rhmax = 0
hora_base_dia1 = 0
hora_base_dia2 = 0
bloqueio = 0
umid_zmedia_dia = 0
umid_zmedia_dia_1 = 0

# Define some constants from the datasheet
DEVICE     = 0x23 # Default device I2C address
POWER_DOWN = 0x00 # No active state
POWER_ON   = 0x01 # Power on
RESET      = 0x07 # Reset data register value
ONE_TIME_HIGH_RES_MODE = 0x20
 
bus = smbus.SMBus(1)  # Rev 2 Pi uses 1

port = "/dev/serial0"

def contador(channel):
    global count
    global contador_pulsos
    contador_pulsos = ((contador_pulsos + 1)) # / (60 * 7.5))


def convertToNumber(data):
  return ((data[1] + (256 * data[0])) / 1.2)
 
def readLight(addr=DEVICE):
  data = bus.read_i2c_block_data(addr,ONE_TIME_HIGH_RES_MODE)
  return convertToNumber(data)

flag_media_diaria = 0
minuto_ultimo = 0
temp_ar_ant = 20
umid_ant = 76
chave_man = 0

def evapotranspiracao(dia_corr, tmax, tmin, tmed, rhmax, n, v_med, alt_v, rhmed):
  p = 92.183188
  elev = 801
  phi = -0.408756110817
  print("RHmed: %s" % rhmed)
  print("Tmax: %s" % tmax)
  print("Tmin: %s" % tmin)
  dr = 1 + (0.033*math.cos(0.017214206321*dia_corr))
  print("dr: %s" % dr)
  delt = 0.409*math.sin((0.017214206321*dia_corr)-1.39)
  print("delta: %s" % delt)
  y = 0.060130
  e0_tmax = 0.6108*math.pow(math.e,((17.27*tmax)/(tmax+237.3)))
  print("eo tmax: %s" % e0_tmax)
  e0_tmin = 0.6108*math.pow(math.e,((17.27*tmin)/(tmin+237.3)))
  print("eo tmin: %s" % e0_tmin)
  es = (e0_tmax + e0_tmin)/2
  print("es: %s" % es)
  D = (4098*(0.6108*math.pow(math.e,((17.27*tmed)/(tmed+237.3)))))/math.pow((tmed+237.3),2)
  print("D: %s" % D)
  ea = es*rhmed/100
  print("ea: %s" % ea)
  ws = math.acos(-math.tan(phi)*math.tan(delt))
  print("ws: %s" % ws)
  ra = 37.5860314*dr*((ws*math.sin(phi)*math.sin(delt)) + (math.cos(phi)*math.cos(delt)*math.sin(ws)))
  print("ra: %s" % ra)
  rs = (0.25 + (0.5 * (n/(7.6394 * ws))))*ra #*0.408
  print("rs: %s" % rs)
  rns = 0.77*rs
  print("rns: %s" % rns)
  rso = (0.75*ra)
  print("rso: %s" % rso)
  f_rs_rso = rs/rso
  if f_rs_rso > 1:
    f_rs_rso = 1
  rnl = (4.903*math.pow(10,-9)) * ((math.pow((tmax+273.16),4) + math.pow((tmin+273.16),4))/2) * (0.34+(-0.14*math.sqrt(ea))) *  ((1.35*(f_rs_rso))-0.35)
  print("rnl: %s" % rnl)
  r_n = rns - rnl
  print("r_n: %s" % r_n)
  g = 0
  uz = v_med*1000/3600
  u2 = uz*(4.87/(math.log(67.8*alt_v - 5.42)))
  et_o = ((0.408*D*(r_n-g)+y*(900/(tmed+273))*u2*(es-ea))/(D+y*(1+0.34*u2)))
  return et_o

def dados_hist_ws(data_base_dia, ws_id):
  conexao = pymysql.connect(db='SWAMP_caseiro', user='root', passwd='123rasp')
  cursor = conexao.cursor()
  sql_consulta_x = 'select ws_cod, round(p_chuva,1), round(c_chuva,1), round(vento,1) from historico_ws where data = ' + '\'' + data_base_dia + '\'' + ' and ws_cod = \'' + ws_id + '\''
  cursor.execute(sql_consulta_x)
  d_hist = str(cursor.fetchone())
  d_hist = str(d_hist[1:(len(d_hist) - 1)])               
  d_hist = d_hist.split(",")
  vento_med = float(d_hist[3])
  return vento_med


def info_estacao_met(ws_cod):
  conexao = pymysql.connect(db='SWAMP_caseiro', user='root', passwd='123rasp')
  cursor = conexao.cursor()
  sql_consulta_y = 'select altiude_vento from estacao_met where alias = \'' + ws_cod + '\''
  cursor.execute(sql_consulta_y)
  altit_vento = str(cursor.fetchone())
  altit_vento = str(altit_vento[1:(len(altit_vento) - 1)])
  altit_vento = altit_vento.split(",")
  altit_vento = int(altit_vento[0])
  return altit_vento

def check_localiz(id_probe):
  conexao = pymysql.connect(db='SWAMP_caseiro', user='root', passwd='123rasp')
  cursor = conexao.cursor()
  sql_consulta = 'SELECT latitude, longitude, probe_id from geo_refer WHERE probe_id = %s ORDER BY probe_id ASC'
  cursor.execute(sql_consulta, id_probe)
  localizacao_atual = str(cursor.fetchone())
  return localizacao_atual

def atualiza_localiz(latit, longit, id_probe):
  conexao = pymysql.connect(db='SWAMP_caseiro', user='root', passwd='123rasp')
  cursor = conexao.cursor()
  cursor.execute (""" UPDATE geo_refer SET latitude=%s, longitude=%s WHERE probe_id=%s """, (latit, longit, id_probe))
  conexao.commit()
  resultado = "Atualização realizada"
  return resultado

def grava_fao_fuzzy_na_evolucao(ws_cod, hoje, arm_fao, irrig_ml_fao, irrig_ml_fuzzy, cod_cultura, ident_probe, ontem, dap, eto, etc, pdc_fuzzy, irrig_mm_fuzzy, irrig_mm_fao, t_irrig, arm_fao_dispon):
  conexao = pymysql.connect(db='SWAMP_caseiro', user='root', passwd='123rasp')
  cursor = conexao.cursor()
  dap = dap -1
  sql_consulta = 'select r_chuva from historico_ws where ws_cod = \'' + ws_cod + '\'' + ' and data = ' + '\'' + ontem + '\''
  cursor.execute(sql_consulta)
  res = str(cursor.fetchone())
  res = str(res[1:(len(res) - 1)])
  res = res.split(",")
  chuva_real = float(res[0])
  irrig_ml_fuzzy = float(irrig_ml_fuzzy)
  cursor.execute(""" INSERT INTO evolucao (arm_fao_max, irrig_ml_fao, irrig_ml_fuzzy, cultura_cod, probe_id, data, dap, eto, etc, chuva_real, pdc, irrig_mm_fao, irrig_mm_fuzzy, t_irrig_fao, arm_fao_dispon) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s) """, (arm_fao, irrig_ml_fao, irrig_ml_fuzzy, cod_cultura, probe_id, ontem, dap, eto, etc, chuva_real, pdc_fuzzy, irrig_mm_fao, np.float(irrig_mm_fuzzy), t_irrig, arm_fao_dispon))
  conexao.commit()
  conexao.close()                
  resultado = "Evolução gravada"
  return resultado

def grava_dados_no_banco(ident_probe, time_stamp, umid, temp, ilumi_env, temp_solo_1, temp_solo_2, obs, umid_solo_z1, umid_solo_z1_perc, umid_solo_z2, umid_solo_z2_perc, umid_solo_z3, umid_solo_z3_perc, umid_solo_z4, umid_solo_z4_perc, umid_solo_z5, umid_solo_z5_perc, umid_solo_z6, umid_solo_z6_perc):
  conexao = pymysql.connect(db='SWAMP_caseiro', user='root', passwd='123rasp')
  cursor = conexao.cursor()
  cursor.execute(""" INSERT INTO sensores (probe_id, time_stamp, umid_ar, temp_ar, ilumi_amb, temp_solo_1, temp_solo_2, obs, umid_solo_z1_int, umid_solo_z1_perc, umid_solo_z2_int, umid_solo_z2_perc, umid_solo_z3_int, umid_solo_z3_perc, umid_solo_z4_int, umid_solo_z4_perc, umid_solo_z5_int, umid_solo_z5_perc, umid_solo_z6_int, umid_solo_z6_perc) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s) """, (ident_probe, time_stamp, umid, temp, ilumi_env, temp_solo_1, temp_solo_2, obs, umid_solo_z1, umid_solo_z1_perc, umid_solo_z2, umid_solo_z2_perc, umid_solo_z3, umid_solo_z3_perc, umid_solo_z4, umid_solo_z4_perc, umid_solo_z5, umid_solo_z5_perc, umid_solo_z6, umid_solo_z6_perc))
  conexao.commit()
  conexao.close()                
  resultado = "sensores gravado"
  return resultado

def determina_dia_ant(dia_atual, mes_atual, ano_atual):
  conexao = pymysql.connect(db='SWAMP_caseiro', user='root', passwd='123rasp')
  cursor = conexao.cursor()
  if (dia_atual < 10):
    dia_atual = '0' + str(dia_atual)
  data_base_dia = '\''+ str(dia_atual) + '/' + str(mes_atual) + '/' + str(ano_atual) + '\''
  sql_consulta3 = 'select data_ant from calendario where data = ' + data_base_dia               
  cursor.execute(sql_consulta3)
  res3 = str(cursor.fetchone())
  res3 = str(res3[1:13])
  return res3

def extrai_dados_astron(ontem_dia, ontem_mes):
  conexao = pymysql.connect(db='SWAMP_caseiro', user='root', passwd='123rasp')
  cursor = conexao.cursor()
  sql_consulta3 = 'select dia_abs, round(hora_sol,1), round(nascer_sol,1) from calendario where data_dia = ' + ontem_dia + ' and data_mes = ' + ontem_mes              
  cursor.execute(sql_consulta3)
  res3 = str(cursor.fetchone())
  res3 = str(res3[1:(len(res3) - 1)])
  res3 = res3.split(",")
  dia_corr = int(res3[0])
  n = float(res3[1])
  nascer_sol = float(res3[2])
  return dia_corr, n, nascer_sol

def determina_datas_base(dia_atual, mes_atual, ano_atual, hora_nascer_sol, hora_por_sol):
  data_base = determina_dia_ant(dia_atual, mes_atual, ano_atual)
  data_base1 = str(data_base[0:11]) + ' ' + hora_nascer_sol + '\''
  data_base2 = str(data_base[0:11]) + ' ' + hora_por_sol + '\''
  data_base_geral1 = str(data_base[0:11]) + ' ' + '00:00' + '\''
  data_base_geral2 = str(data_base[0:11]) + ' ' + '23:51' + '\''
  return data_base1, data_base2, data_base_geral1, data_base_geral2 

def determina_nasc_por_sol(nascer_sol, n):
  h_nascer_sol = math.floor(nascer_sol)
  m_nascer_sol  = (nascer_sol- h_nascer_sol) * 60
  hora_nascer_sol = '0' + str(int(h_nascer_sol))+':'+str(int(m_nascer_sol))
  dur_dia = n + nascer_sol
  h_por_sol = math.floor(dur_dia)
  m_por_sol = (dur_dia - h_por_sol)* 60
  hora_por_sol = str(int(h_por_sol))+':'+str(int(m_por_sol))
  return hora_nascer_sol, hora_por_sol

def extrai_dados_ar_probe(data_base1, data_base2):
  conexao = pymysql.connect(db='SWAMP_caseiro', user='root', passwd='123rasp')
  cursor = conexao.cursor()
  sql_consulta = 'select round(min(temp_ar),1), round(max(temp_ar),1), round(avg(temp_ar),1), round(max(umid_ar),1), round(avg(umid_ar),1) from sensores where time_stamp between ' + data_base1 + ' and ' + data_base2  
  #print(sql_consulta)
  cursor.execute(sql_consulta)
  res = str(cursor.fetchone())
  res = str(res[1:(len(res) - 1)])
  res = res.split(",")
  tmin = float(res[0])
  tmax = float(res[1])
  tmed = float(res[2])
  rhmax = float(res[3])
  rhmed = float(res[4])
  return tmin, tmax, tmed, rhmax, rhmed

def extrai_dados_solo_probe(data_base1, data_base2):
  conexao = pymysql.connect(db='SWAMP_caseiro', user='root', passwd='123rasp')
  cursor = conexao.cursor()
  sql_consulta1 = 'select umid_solo_z1_perc, umid_solo_z2_perc, umid_solo_z4_perc, umid_solo_z5_perc from sensores where time_stamp between ' + data_base1 + ' and ' + data_base2 + ' order by time_stamp desc limit 1'
  cursor.execute(sql_consulta1)
  res1 = str(cursor.fetchone())
  res1 = str(res1[1:(len(res1) - 1)])
  res1 = res1.split(",")
  umid_z1_dia = float(res1[0])
  umid_z2_dia = float(res1[1])
  umid_z4_dia = float(res1[2])
  umid_z5_dia = float(res1[3])
  umid_zmedia_dia = umid_z1_dia
  umid_zmedia_dia_1 = ((umid_z1_dia * 0.3) + (umid_z5_dia * 0.7))
  return umid_z1_dia, umid_z2_dia, umid_zmedia_dia, umid_zmedia_dia_1

def determina_dap_atual(hoje, probe_ide, cod_cultura):
  conexao = pymysql.connect(db='SWAMP_caseiro', user='root', passwd='123rasp')
  cursor = conexao.cursor()
  sql_consulta = 'select data, nome from culturas where cultura_cod = ' + str(cod_cultura) + ' and probe_id = ' + str(probe_ide)
  cursor.execute(sql_consulta)
  res = str(cursor.fetchone())
  res = str(res[1:(len(res) - 1)])
  res = res.split(",")
  data_inicial = str(res[0])
  data_inicial = str(data_inicial[1:(len(data_inicial) - 1)])
  nome_cultura = str(res[1])
  hoje = date.today()
  data_inicial_f = datetime.strptime(data_inicial, '%d/%m/%Y')
  intervalo = hoje - data_inicial_f.date()
  dap = intervalo.days
  return dap, nome_cultura


def determina_z_rad_kc(dap, cod_cultura):
  conexao = pymysql.connect(db='SWAMP_caseiro', user='root', passwd='123rasp')
  cursor = conexao.cursor()
  sql_consulta = 'select kc, z_rad from parametros_cultura where dap = ' + str(dap) + ' and cultura_cod = ' + str(cod_cultura)
  cursor.execute(sql_consulta)
  res = str(cursor.fetchone())
  res = str(res[1:(len(res) - 1)])
  res = res.split(",")                
  kc = float(res[0])
  z_rad = float(res[1])
  return kc, z_rad

def extrai_previsao_tempo(data_base_dia, ws_cod):
  conexao = pymysql.connect(db='SWAMP_caseiro', user='root', passwd='123rasp')
  cursor = conexao.cursor()
  sql_consulta = 'select p_chuva, c_chuva from historico_ws where data = ' + '\'' + data_base_dia + '\'' + ' and ws_cod = %s'
  cursor.execute(sql_consulta, ws_cod)
  res = str(cursor.fetchone())
  res = str(res[1:(len(res) - 1)])
  res = res.split(",")
  p_chuva = float(res[0])
  c_chuva = float(res[1])
  return p_chuva, c_chuva

def grava_variaveis_fuzzy(cod_cultura, probe_id, data, dap, p_chuva, c_chuva, pdc, etc, eto):
  conexao = pymysql.connect(db='SWAMP_caseiro', user='root', passwd='123rasp')
  cursor = conexao.cursor()
  cursor.execute(""" INSERT INTO fuzzy (cultura_cod, probe_id, data, dap, p_chuva, c_chuva, pdc, etc, eto) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s) """, (cod_cultura, probe_id, data, dap, p_chuva, c_chuva, pdc, etc, eto))
  conexao.commit()
  conexao.close()                
  resultado = "Fuzzy gravado"
  return resultado
  
def escreve_no_display(linha1, linha2, tempo):
  lcdi2c.lcd_display_string(linha1, 1,1)
  lcdi2c.lcd_display_string(linha2, 2,0)
  time.sleep(tempo)
  lcdi2c.lcd_clear()
  return


def inferencia_fuzzy(et_c, cod_cultura, hoje, ontem, probe_id, mes):
  conexao = pymysql.connect(db='SWAMP_caseiro', user='root', passwd='123rasp')
  cursor = conexao.cursor()
  sql_consulta = 'select dap, p_chuva, c_chuva, etc, pdc from fuzzy where data = ' + '\'' + hoje + '\'' + ' and probe_id = %s and cultura_cod = ' + str(cod_cultura)
  cursor.execute(sql_consulta, probe_id)
  res = str(cursor.fetchone())
  res = str(res[1:(len(res) - 1)])
  res = res.split(",")
  dap = int(res[0])
  p_chuva = float(res[1])
  c_chuva = float(res[2])
  etc = float(res[3])
  pdc = float(res[4])
  sql_consulta1 = 'select media_chuva_dia from serie_hist_chuva where mes = ' + str(mes)
  cursor.execute(sql_consulta1)
  res = str(cursor.fetchone())
  res = str(res[1:(len(res) - 1)])
  res = res.split(",")
  media_chuva_mes_corr = float(res[0])

  #print("Inferencia E2")
  eto_arr = round((et_c),0)
  sql_consulta2 = 'select f_dep from fator_dep where etm = ' + str(eto_arr) + ' and cultura_id = ' + str(cod_cultura)
  cursor.execute(sql_consulta2)
  res = str(cursor.fetchone())
  res = str(res[1:(len(res) - 1)])
  res = res.split(",")
  fator_dep = float(res[0])

  #print("Inferencia E3")
  #Níveis do fator de depleção para a entrada do fuzzificador
  cc = 39
  fd = cc - round(fator_dep * 10,0)
  fd = round(fd,0)
  fd_inf = fd - 5
  fd_sup = fd + 4

  #print("Inferencia E4")

  dap_dias = ctrl.Antecedent(np.arange(0,110,1), 'DAP_Dias')
  prev_chuva = ctrl.Antecedent(np.arange(0, 11, 1), 'Prev_Chuva')
  conf_chuva = ctrl.Antecedent(np.arange(0, 100, 1), 'Conf_Chuva')
  pont_deplec = ctrl.Antecedent(np.arange(18, 36, 1), 'PDC')
  demand_evapo = ctrl.Antecedent(np.arange(0, 10, 1), 'ETm')
  irrig = ctrl.Consequent(np.arange(0, 31, 1), 'Irrigacao')

  #Auto-membership function population is possible with .automf(3, 5, or 7)
  #prev_chuva.automf(3)
  #pont_deplec.automf(5)

  conf_chuva['minimo'] = fuzz.trimf(conf_chuva.universe, [0, 0, 25])
  conf_chuva['baixo'] = fuzz.trimf(conf_chuva.universe, [0, 25, 50])
  conf_chuva['medio'] = fuzz.trimf(conf_chuva.universe, [25, 50, 75])
  conf_chuva['alto'] = fuzz.trimf(conf_chuva.universe, [50, 75, 100])
  conf_chuva['elevado'] = fuzz.trimf(conf_chuva.universe, [75, 100, 3000])
  
  dap_dias['emergencia'] = fuzz.trimf(dap_dias.universe, [0, 0, 15])
  dap_dias['normal'] = fuzz.trimf(dap_dias.universe, [0, 15, 65])
  dap_dias['embonecamento'] = fuzz.trimf(dap_dias.universe, [60, 65, 70])
  dap_dias['final'] = fuzz.trimf(dap_dias.universe, [65, 70, 3000])

  demand_evapo['baixo'] = fuzz.trimf(demand_evapo.universe, [0, 0, 5])
  demand_evapo['moderado'] = fuzz.trimf(demand_evapo.universe, [3, 5, 7])
  demand_evapo['alto'] = fuzz.trimf(demand_evapo.universe, [5, 7, 9])
  demand_evapo['elevado'] = fuzz.trimf(demand_evapo.universe, [7, 9, 3000])
  
  prev_chuva['baixo'] = fuzz.trimf(prev_chuva.universe, [0, 0, 3])
  prev_chuva['ideal'] = fuzz.trimf(prev_chuva.universe, [0, 3, 8])
  prev_chuva['alto'] = fuzz.trimf(prev_chuva.universe, [3, 8, 3000])
  
  pont_deplec['critico'] = fuzz.trimf(pont_deplec.universe, [15, 15, 21])
  pont_deplec['baixo'] = fuzz.trimf(pont_deplec.universe, [15, 20, 26])
  pont_deplec['ideal'] = fuzz.trimf(pont_deplec.universe, [20, 26, 33])
  pont_deplec['alto'] = fuzz.trimf(pont_deplec.universe, [26, 33, 39])
  pont_deplec['elevado'] = fuzz.trimf(pont_deplec.universe, [33, 39, 1000])

  #print("Inferencia E5")

  #Custom membership functions can be built interactively with a familiar,
  #Pythonic API

  #print("Inferencia E6")
  irrig['minimo'] = fuzz.trimf(irrig.universe, [0, 0, 2])
  irrig['baixo'] = fuzz.trimf(irrig.universe, [0, 2, 4])
  irrig['ideal'] = fuzz.trimf(irrig.universe, [2, 4, 6])
  irrig['alto'] = fuzz.trimf(irrig.universe, [4, 6, 8])
  irrig['elevado'] = fuzz.trimf(irrig.universe, [6, 8, 20])

  #Para ver os gráficos das funções de pertinência ativar as linhas abaixo, com .view()
  #demand_evapo.view()
  #prev_chuva.view()
  #conf_chuva.view()
  #pont_deplec.view()
  #irrig.view()
  #dap_dias.view()

  regra_1 = ctrl.Rule(pont_deplec['critico'], irrig['elevado'])
  regra_2 = ctrl.Rule(pont_deplec['elevado'] & (dap_dias['final'] | dap_dias['normal']), irrig['minimo'])
  regra_3 = ctrl.Rule(pont_deplec['baixo'] & (prev_chuva['baixo'] | prev_chuva['ideal']) & conf_chuva['baixo'] & (demand_evapo['alto'] | demand_evapo['elevado']), irrig['elevado'])
  regra_31 = ctrl.Rule(pont_deplec['baixo'] & (prev_chuva['baixo'] | prev_chuva['ideal']) & conf_chuva['baixo'] & (demand_evapo['moderado'] | demand_evapo['alto']), irrig['ideal'])
  regra_4 = ctrl.Rule(pont_deplec['baixo'] & (prev_chuva['ideal'] | prev_chuva['alto']) & (conf_chuva['medio'] | conf_chuva['alto']) & (demand_evapo['alto'] | demand_evapo['moderado']), irrig['ideal'])
  regra_5 = ctrl.Rule(pont_deplec['baixo'] & prev_chuva['alto'] & (conf_chuva['medio'] | conf_chuva['alto'] | conf_chuva['elevado']) & (demand_evapo['baixo'] | demand_evapo['moderado']), irrig['baixo'])
  regra_6 = ctrl.Rule(pont_deplec['ideal'] & (prev_chuva['baixo'] | prev_chuva['ideal']) & (conf_chuva['baixo'] | conf_chuva['medio']) & (demand_evapo['elevado'] | demand_evapo['alto'] | demand_evapo['moderado']), irrig['baixo'])
  regra_7 = ctrl.Rule(pont_deplec['ideal'] & (prev_chuva['ideal'] | prev_chuva['alto']) & (conf_chuva['medio'] | conf_chuva['alto'] | conf_chuva['elevado']) & (demand_evapo['baixo'] | demand_evapo['moderado']), irrig['baixo'])
  regra_8 = ctrl.Rule(pont_deplec['alto'] & prev_chuva['baixo'] & (conf_chuva['baixo'] | conf_chuva['medio'] | conf_chuva['alto']), irrig['ideal'])
  regra_9 = ctrl.Rule(pont_deplec['alto'] & (prev_chuva['ideal'] | prev_chuva['alto']) & (conf_chuva['minimo'] | conf_chuva['baixo'] | conf_chuva['medio'] | conf_chuva['alto'] | conf_chuva['elevado']), irrig['minimo'])
  regra_10 = ctrl.Rule((dap_dias['emergencia'] | dap_dias['embonecamento']) & (pont_deplec['baixo'] | pont_deplec['ideal']), irrig['alto'])
  regra_11 = ctrl.Rule(pont_deplec['elevado'], irrig['minimo'])
  regra_12 = ctrl.Rule(pont_deplec['ideal'], irrig['ideal'])
  regra_13 = ctrl.Rule(pont_deplec['alto'], irrig['baixo'])
  regra_14 = ctrl.Rule(pont_deplec['alto'] & dap_dias['final'], irrig['baixo']) #Esta regra é válida para a cultura do pimentão
  #regra_10.view()
  contr_saida = ctrl.ControlSystem([regra_31, regra_14, regra_12, regra_13, regra_1, regra_2, regra_3, regra_4, regra_5, regra_6, regra_7, regra_8, regra_9, regra_10, regra_11])

  saidas = ctrl.ControlSystemSimulation(contr_saida)
  saidas.input['DAP_Dias'] = dap
  saidas.input['Prev_Chuva'] = p_chuva
  saidas.input['Conf_Chuva'] = c_chuva
  saidas.input['PDC'] = pdc
  saidas.input['ETm'] = etc
  #Crunch the numbers
  saidas.compute()
  irrig_dia = (round((saidas.output['Irrigacao']),1))
  sql = 'UPDATE fuzzy SET irrig = %s WHERE data = ' + '\'' + hoje + '\'' + ' and probe_id = %s and cultura_cod = %s'
  valores = (np.float(irrig_dia), probe_id, cod_cultura)
  cursor.execute (sql, valores)
  conexao.commit()
  return irrig_dia

def informacoes_modelo_fao_irrig(probe_id, cod_cultura):
  intervalo = timedelta(2)
  hoje = date.today()
  ante_ontem = hoje - intervalo
  conexao = pymysql.connect(db='SWAMP_caseiro', user='root', passwd='123rasp')
  cursor = conexao.cursor()
  sql_consulta = 'select atd_mm from Parametros_solo where solo_id = (select textura from culturas where probe_id = %s and cultura_cod = ' + str(cod_cultura) + ' order by cultura_id desc limit 1)'
  cursor.execute(sql_consulta, probe_id)
  res = str(cursor.fetchone())
  res = str(res[1:(len(res) - 1)])
  res = res.split(",")
  res = float(res[0])/100
  return res

def extrai_fpdc_fao(et_c, cod_cultura):
  conexao = pymysql.connect(db='SWAMP_caseiro', user='root', passwd='123rasp')
  cursor = conexao.cursor()
  if (et_c < 2):
    et_c = 2
  sql_consulta = 'select f_dep from fator_dep where etm = %s and cultura_id = ' + str(cod_cultura)
  cursor.execute(sql_consulta, et_c)
  res = str(cursor.fetchone())
  res = str(res[1:(len(res) - 1)])
  res = res.split(",")
  f_pdc = float(res[0])
  return f_pdc

def grava_variaveis_fao(cod_cultura, probe_id, data, dap, t_irrig, l_liq_fao, etc, eto):
  conexao = pymysql.connect(db='SWAMP_caseiro', user='root', passwd='123rasp')
  cursor = conexao.cursor()
  cursor.execute(""" INSERT INTO fao (cultura_cod, probe_id, data, dap, etc, irrig_mm, t_irrig, eto) VALUES (%s, %s, %s, %s, %s, %s, %s, %s) """, (cod_cultura, probe_id, data, dap, etc, l_liq_fao, t_irrig, eto))
  conexao.commit()
  conexao.close()                
  resultado = "FAO gravado"
  return resultado

def balanco_arm_fao(cod_cultura, arm_fao, ws_id):
  conexao = pymysql.connect(db='SWAMP_caseiro', user='root', passwd='123rasp')
  cursor = conexao.cursor()
  intervalo_evoluc_1 = timedelta(1)
  intervalo_evoluc_2 = timedelta(2) 
  hoje_evoluc = date.today()
  data_ontem_evoluc = hoje_evoluc - intervalo_evoluc_1
  data_ante_ontem_evoluc = hoje_evoluc - intervalo_evoluc_2
  sql_consulta = 'select arm_fao_dispon from evolucao where probe_id = 4 and data =  ' + '\'' + data_ante_ontem_evoluc.strftime("%d/%m/%Y") + '\'' + ' and cultura_cod = ' + str(cod_cultura)
  cursor.execute(sql_consulta)
  res = str(cursor.fetchone())
  res = str(res[1:(len(res) - 1)])
  res = res.split(",")
  arm_fao_dispon = float(res[0])
  
  sql_consulta = 'select r_chuva from historico_ws where data =  ' + '\'' + data_ontem_evoluc.strftime("%d/%m/%Y") + '\'' + ' and ws_cod = %s '
  cursor.execute(sql_consulta, ws_id)
  res = str(cursor.fetchone())
  res = str(res[1:(len(res) - 1)])
  res = res.split(",")
  chuva_real = float(res[0])

  sql_consulta = 'select etc, irrig_mm from fao where probe_id = 4 and data =  ' + '\'' + data_ontem_evoluc.strftime("%d/%m/%Y") + '\'' + ' and cultura_cod = ' + str(cod_cultura)
  cursor.execute(sql_consulta)
  res = str(cursor.fetchone())
  res = str(res[1:(len(res) - 1)])
  res = res.split(",")
  etc = float(res[0])
  irrig_fao = float(res[1])

  arm_fao_hoje = arm_fao_dispon + irrig_fao + chuva_real - etc
  if (arm_fao_hoje > arm_fao):
    arm_fao_hoje = arm_fao  
  return arm_fao_hoje, irrig_fao

def conversao_irrig(l_liq_fao, irrigacao_dia):
  irrig_fao_ml = round((0.0529 * l_liq_fao * 1000),0)
  irrig_fuzzy_ml = round((0.0529 * irrigacao_dia * 1000),0)
  return irrig_fao_ml, irrig_fuzzy_ml

def determina_probab_chuva(p1,p2):
    #print("p1 = %s, p2= %s" %(p1,p2))
    if (p1 >= p2):
        maior = p1
    else:
        maior = p2
    return maior

def parse_dados_pws(key, pws_id):
    print("Entrou na function parse dados pws")
    sucesso = 0
    c = 0
    v_res = sucesso
    while (c < 3 and sucesso == 0):
        try:
            json_data = requests.get("https://api.weather.com/v2/pws/observations/current?stationId=" + pws_id + "&format=json&units=m&apiKey=%s&numericPrecision=decimal" % key, timeout=1.000)
            json = json_data.json()
            sucesso = 1
            v_res = sucesso
        except requests.exceptions.ConnectionError as e:
            print (type(e))
            print ("Falha na extracao dos dados da PWS! %s" % pws_id)
            c = c + 1
    return json, v_res    

def parse_dados_forecast(key, geo_cod):    
    try:            
        json_data = requests.get("https://api.weather.com/v3/wx/forecast/daily/5day?geocode=" + geo_cod + "&format=json&units=m&language=pb-BR&apiKey=%s" % key, timeout=1.500)
        json = json_data.json()
        return json    
    except requests.exceptions.ConnectionError as e:
        print (type(e))
        print ("Check your network connection!")

def aquisic_dados_pws(t_stamp):
  lista_pws = ['IEXTRE1', '-22.78,-46.28', 'ISOBER3', '-23.73,-46.58', 'ISOBERNA3', '-23.71,-46.55']
  time_stamp = t_stamp
  w = 0
  while (w < 6): #and (cont_falha < 2):
    print("Variavel w = %s" % w)
    try:
      dados, v_ret = parse_dados_pws(key, lista_pws[w])
      pws = dados['observations'][0]['stationID']
      local_pws = dados['observations'][0]['neighborhood']
      umid_ar = dados['observations'][0]['humidity']
      chuva_acc = dados['observations'][0]['metric']['precipTotal']
      vel_vent = dados['observations'][0]['metric']['windSpeed']
      temp = dados['observations'][0]['metric']['temp']
      print(time_stamp, pws, vel_vent, temp, umid_ar , chuva_acc)  
      conexao = pymysql.connect(db='SWAMP_caseiro', user='root', passwd='123rasp')
      cursor = conexao.cursor()
      cursor.execute(""" INSERT INTO pws_dados (time_stamp, estacao_id, vel_vento, temp_ar, umid_ar, precip_acc) VALUES (%s, %s, %s, %s, %s, %s) """, (time_stamp,pws, vel_vent, temp, umid_ar, chuva_acc))
      conexao.commit()
      conexao.close()
    except Exception:
      print("Falha na aquisição dos dados da estação %s" % lista_pws[w])
    w = w + 2
  return v_ret

def aquisic_dados_prev_pws(dia):
  print("Entrou Aquisicao dados da PWS")  
  lista_pws = ['ISOBERNA3', '-23.71,-46.55', 'ISOBER3', '-23.73,-46.58', 'IEXTRE1', '-22.78,-46.28']
  hoje = date.today()
  interv1 = timedelta(1)
  interv2 = timedelta(2)
  interv3 = timedelta(3)
  interv4 = timedelta(4)
  interv5 = timedelta(5)
  interv6 = timedelta(6)
  d1 = hoje + interv1
  d1 = d1.strftime("%d/%m/%Y")
  d2 = hoje + interv2
  d2 = d2.strftime("%d/%m/%Y")
  d3 = hoje + interv3
  d3 = d3.strftime("%d/%m/%Y")
  d4 = hoje + interv4
  d4 = d4.strftime("%d/%m/%Y")
  d5 = hoje + interv5
  d5 = d5.strftime("%d/%m/%Y")
  d6 = hoje + interv6
  d6 = d6.strftime("%d/%m/%Y")
  w = 0
  while (w < 6):
      print("Entrou no laço externo")
      dados1 = parse_dados_forecast(key, lista_pws[w+1])
      print("Voltou do parse forecast")
      c_prev = dados1['daypart'][0]['precipChance']
      perc_ch_d0, perc_ch_d1, perc_ch_d2, perc_ch_d3, perc_ch_d4, perc_ch_d5, j = 0, 0, 0, 0, 0, 0, 0
      while (j<11):
          if (c_prev[j] == None):
              c_prev[j] = 0
          if (c_prev[j+1] == None):
              c_prev[j+1] = 0             
          prev_d = determina_probab_chuva(c_prev[j],c_prev[j+1])
          if (j == 0):
              perc_ch_d0 = prev_d
          if (j == 2):
              perc_ch_d1 = prev_d
          if (j == 4):
              perc_ch_d2 = prev_d
          if (j == 6):
              perc_ch_d3 = prev_d
          if (j == 8):
              perc_ch_d4 = prev_d
          if (j == 10):
              perc_ch_d5 = prev_d
          j = j + 2

      prev_ch = dados1['qpf']
      mm_ch_d0 = prev_ch[0]
      mm_ch_d1 = prev_ch[1]
      mm_ch_d2 = prev_ch[2]
      mm_ch_d3 = prev_ch[3]
      mm_ch_d4 = prev_ch[4]
      mm_ch_d5 = prev_ch[5]
      print("Vai atualizar o banco")  

      conexao = pymysql.connect(db='SWAMP_caseiro', user='root', passwd='123rasp')
      cursor = conexao.cursor()
      sql = 'UPDATE historico_ws SET p_chuva = %s, c_chuva = %s WHERE data = ' + '\'' + d1 + '\'' + ' and ws_cod = %s'
      valores = (mm_ch_d1, perc_ch_d1, lista_pws[w])
      cursor.execute (sql, valores)
      print("Atualizou D1")  
      sql = 'UPDATE historico_ws SET p_chuva = %s, c_chuva = %s WHERE data = ' + '\'' + d2 + '\'' + ' and ws_cod = %s'
      valores = (mm_ch_d2, perc_ch_d2, lista_pws[w])
      cursor.execute (sql, valores)
      print("Atualizou D2")  
      sql = 'UPDATE historico_ws SET p_chuva = %s, c_chuva = %s WHERE data = ' + '\'' + d3 + '\'' + ' and ws_cod = %s'
      valores = (mm_ch_d3, perc_ch_d3, lista_pws[w])
      cursor.execute (sql, valores)
      print("Atualizou D3")  
      sql = 'UPDATE historico_ws SET p_chuva = %s, c_chuva = %s WHERE data = ' + '\'' + d4 + '\'' + ' and ws_cod = %s'
      valores = (mm_ch_d4, perc_ch_d4, lista_pws[w])
      cursor.execute (sql, valores)
      print("Atualizou D4")  
      sql = 'UPDATE historico_ws SET p_chuva = %s, c_chuva = %s WHERE data = ' + '\'' + d5 + '\'' + ' and ws_cod = %s'
      valores = (mm_ch_d5, perc_ch_d5, lista_pws[w])
      cursor.execute (sql, valores)
      print("Atualizou D5")  
      cursor.execute(""" INSERT INTO historico_ws (data, p_chuva, c_chuva, r_chuva, vento, ws_cod) VALUES (%s, %s, %s, %s, %s, %s) """, (d6, 0, 0, 0, 0, lista_pws[w]))
      conexao.commit()
      w = w + 2
  return

def chama_previsao(hoje):
  conexao = pymysql.connect(db='SWAMP_caseiro', user='root', passwd='123rasp')
  cursor = conexao.cursor()
  lista_ws = ['ISOBERNA3', 'ISOBER3', 'IEXTRE1']
  c = 0
  while (c < 3):
    hoje = date.today()
    interv1 = timedelta(1)
    ws_cod = lista_ws[c]
    sql_consulta = 'select round(avg(vel_vento),1), max(precip_acc) from pws_dados where time_stamp like ' + '\'' + hoje.strftime("%d/%m/%Y") + '%' + '\''  + ' and estacao_id = ' + '\'' + ws_cod + '\''
    cursor.execute(sql_consulta)
    res = str(cursor.fetchone())
    res = str(res[1:(len(res) - 1)])
    res = res.split(",")
    try:
      if (res[0] == 'None' or res[0] == ' None'):
        res[0] = 0
    except Exception:
      res[0] = 0
    try:
      if (res[1] == 'None' or res[1] == ' None'):
        res[1] = 0
    except Exception:
      res[1] = 0
    vel_med_vento = float(res[0])
    chuv_acc = float(res[1])
    hoje = hoje.strftime("%d/%m/%Y")
    sql = 'UPDATE historico_ws SET r_chuva = %s, vento = %s WHERE data = ' + '\'' + hoje + '\'' + ' and ws_cod = ' + '\'' + ws_cod + '\''
    valores = (chuv_acc, vel_med_vento)
    cursor.execute (sql, valores)
    conexao.commit()
    c = c + 1
  hoje = date.today()
  try:
    aquisic_dados_prev_pws(hoje.strftime("%d/%m/%Y"))
  except Exception:
    print("Falha ao gravar dados da previsão de chuva")

  return

def irrigador(alvo_irrigacao, canal, tempo_irrig):
    if (GPIO.input(35) == GPIO.HIGH) and (canal == 1):
        print("Entrou no modo Manual de irrigacao")
        letra_modo = 'M'
        display_L1 = ""
        display_L2 = "Modo Manual"
        v_display = escreve_no_display(display_L1, display_L2, 5)
    else:
        if canal == 2:
            print("Irrigando em modo Auto")
            letra_modo = 'A'
            display_L1 = "Irrig Auto FIS"
            display_L2 = "Volume : %s" % alvo_irrigacao
            v_display = escreve_no_display(display_L1, display_L2, 10)
        else:
            print("Irrigando em modo Auto")
            letra_modo = 'A'
            display_L1 = "Irrig Auto FAO"
            display_L2 = "Volume : %s" % alvo_irrigacao
            v_display = escreve_no_display(display_L1, display_L2, 10)
            
    global contador_pulsos
    global i_auto
    v_teste = 1
    print("i auto : %s" % i_auto)
    contador_pulsos = 0
    agora = datetime.now()
    tempo_ref = agora.minute
    tempo_total = tempo_ref + tempo_irrig
    tempo_decorri = 0
    while (contador_pulsos < alvo_irrigacao and tempo_decorri < tempo_total) and ((GPIO.input(35) == GPIO.HIGH) or (i_auto == 1)): #(v_teste == 1):
        agora = datetime.now()
        tempo_decorri = agora.minute
        try:
            if (GPIO.input(21) == GPIO.HIGH) and (GPIO.input(35) == GPIO.HIGH):
                i_canal = 'FIS'
                mmH2O = round(contador_pulsos/3600,1)
                mmH2O = round(mmH2O * 0.2115,1)
                lcdi2c.lcd_display_string("%s" %letra_modo, 1,15)
                lcdi2c.lcd_display_string("Irr %s" %i_canal, 1,0)
                lcdi2c.lcd_display_string("Volume: %s" %contador_pulsos, 2,0)
                lcdi2c.lcd_display_string("%s mm" %mmH2O, 1,9)
                print("%s pulsos - %s mm H2O" % (contador_pulsos, mmH2O))
            if (GPIO.input(23) == GPIO.HIGH) and (GPIO.input(35) == GPIO.HIGH):
                i_canal = 'FAO'
                mmH2O = round(contador_pulsos/3600,1)
                mmH2O = round(mmH2O * 0.2115,1)
                lcdi2c.lcd_display_string("%s" %letra_modo, 1,15)
                lcdi2c.lcd_display_string("Irr %s" %i_canal, 1,0)
                lcdi2c.lcd_display_string("Volume: %s" %contador_pulsos, 2,0)
                lcdi2c.lcd_display_string("%s mm" %mmH2O, 1,9)
                print("%s pulsos - %s mm H2O" % (contador_pulsos, mmH2O))
        except Exception:
            lcdi2c.lcd_clear()
            print("Saiu Man sem irrigar")
            
        if (GPIO.input(33) == True) or (canal == 2):
          GPIO.output(21,1)
          GPIO.output(29,1)
          mmH2O = round(contador_pulsos/3600,1)
          mmH2O = round(mmH2O * 0.2115,1)
          lcdi2c.lcd_display_string("%s" %letra_modo, 1,15)
          lcdi2c.lcd_display_string("Irr %s" %canal, 1,0)
          lcdi2c.lcd_display_string("Volume: %s" %contador_pulsos, 2,0)
          lcdi2c.lcd_display_string("%s mm" %mmH2O, 1,9)
          print("%s mm H2O - %s pulsos para alvo em %s. Tempo decorrido = %s para limite em %s" % (mmH2O, contador_pulsos, alvo_irrigacao, tempo_decorri, tempo_total))
        else:
          GPIO.output(21,0)
          GPIO.output(29,0)
                
        if (GPIO.input(37) == True) or (canal == 3):
          GPIO.output(23,1)
          GPIO.output(31,1)
          mmH2O = round(contador_pulsos/3600,1)
          mmH2O = round(mmH2O * 0.2115,1)
          lcdi2c.lcd_display_string("%s" %letra_modo, 1,15)
          lcdi2c.lcd_display_string("Irr %s" %canal, 1,0)
          lcdi2c.lcd_display_string("Volume: %s" %contador_pulsos, 2,0)
          lcdi2c.lcd_display_string("%s mm" %mmH2O, 1,9)
          print("%s mm H2O - %s pulsos para alvo em %s. Tempo decorrido = %s para limite em %s" % (mmH2O, contador_pulsos, alvo_irrigacao, tempo_decorri, tempo_total))
          
        else:
          GPIO.output(23,0)
          GPIO.output(31,0)
        if (GPIO.input(37) == True or GPIO.input(37) == True):
            chave_man = 1

        if (GPIO.input(33) == False and GPIO.input(37) == False) and (canal == 1): #and (chave_man == 1):
            time.sleep(3)
            print("Ciclo manual interrompido")
            display_L1 = "Saiu Manual"
            display_L2 = "%d pulsos" % (contador_pulsos)
            v_display = escreve_no_display(display_L1, display_L2, t_display)
            contador_pulsos = 0

    GPIO.output(23,0)
    GPIO.output(31,0)
    GPIO.output(21,0)
    GPIO.output(29,0)
    print("Saindo")

    return

def prescr_irrig_dia(hoje, cod_cultura, ident_probe):
  conexao = pymysql.connect(db='SWAMP_caseiro', user='root', passwd='123rasp')
  cursor = conexao.cursor()
  hoje_evoluc = date.today()
  sql_consulta = 'select irrig from fuzzy where probe_id = ' + str(ident_probe) + ' and data =  ' + '\'' + hoje_evoluc.strftime("%d/%m/%Y") + '\'' + ' and cultura_cod = ' + str(cod_cultura)
  cursor.execute(sql_consulta)
  res = str(cursor.fetchone())
  res = str(res[1:(len(res) - 1)])
  res = res.split(",")
  lamina_FIS = res[0]
  lamina_FIS = float(res[0])
  print("lamina FIS em %s: %s mm" % (hoje_evoluc, lamina_FIS))
  
  sql_consulta = 'select irrig_mm from fao where probe_id = ' + str(ident_probe) + ' and data =  ' + '\'' + hoje_evoluc.strftime("%d/%m/%Y") + '\'' + ' and cultura_cod = ' + str(cod_cultura)
  cursor.execute(sql_consulta)
  res = str(cursor.fetchone())
  res = str(res[1:(len(res) - 1)])
  res = res.split(",")
  lamina_FAO = res[0]
  lamina_FAO = float(res[0])
  print("lamina FAO em %s: %s mm" % (hoje_evoluc, lamina_FAO))
  
  return lamina_FIS, lamina_FAO

def dados_ar_pws(ws_id):
  conexao = pymysql.connect(db='SWAMP_caseiro', user='root', passwd='123rasp')
  cursor = conexao.cursor()
  sql_consulta = 'select temp_ar, umid_ar from pws_dados where estacao_id = ' + '\'' + ws_id +'\'' + ' order by idpws_dados desc limit 1'
  cursor.execute(sql_consulta)
  res = str(cursor.fetchone())
  res = str(res[1:(len(res) - 1)])
  res = res.split(",")
  temp_ar = res[0]
  umid_ar = res[1]
  return temp_ar, umid_ar

def chuva_dia(ws_id, hoje, cod_cultura, ident_probe):
  conexao = pymysql.connect(db='SWAMP_caseiro', user='root', passwd='123rasp')
  cursor = conexao.cursor()
  sql_consulta = 'select precip_acc from pws_dados where estacao_id = ' + '\'' + ws_id +'\'' + ' and time_stamp like ' + '\'' + hoje.strftime("%d/%m/%Y") + '%' + '\'' + 'order by time_stamp desc limit 1'
  cursor.execute(sql_consulta)
  res = str(cursor.fetchone())
  res = str(res[1:(len(res) - 1)])
  res = res.split(",")
  chuva_hj = res[0]

  sql_consulta = 'select irrig from fuzzy where probe_id = ' + str(ident_probe) + ' and data =  ' + '\'' + hoje.strftime("%d/%m/%Y") + '\'' + ' and cultura_cod = ' + str(cod_cultura)
  cursor.execute(sql_consulta)
  res = str(cursor.fetchone())
  res = str(res[1:(len(res) - 1)])
  res = res.split(",")
  lamina_FIS = res[0]
  lamina_FIS = float(res[0])
  
  sql_consulta = 'select irrig_mm from fao where probe_id = ' + str(ident_probe) + ' and data =  ' + '\'' + hoje.strftime("%d/%m/%Y") + '\'' + ' and cultura_cod = ' + str(cod_cultura)
  cursor.execute(sql_consulta)
  res = str(cursor.fetchone())
  res = str(res[1:(len(res) - 1)])
  res = res.split(",")
  lamina_FAO = res[0]
  lamina_FAO = float(res[0])

  return chuva_hj, lamina_FIS, lamina_FAO


def zera_fluxo(channel):
    escreve_no_display("", "", 1)


#Dados da cultura
cod_cultura = 123
ident_probe = 4
ws_id = 'IEXTRE1'
pws_SBC = 'ISOBERNA3'
pws_FEI = 'ISOBER3'
pws_EXT = 'IEXTRE1'
geo_cod_SBC = '-23.71,-46.55'
geo_cod_FEI = '-23.73,-46.58'
geo_cod_EXT = '-22.78,-46.28'

t_display = 3
display_L1 = "Giba"
gravou = 0
modo_teste = 0
alvo_irrigacao = 50000
tempo_irrig_FAO = 0
tempo_irrig_FIS = 0
global i_auto
i_auto = 0
gravou_1 = 0
gravou_2 = 0
v_gravacao_hora_1 = 0
v_gravacao_hora_2 = 0
gravou_previsao = 0
prev_tempo_man = 1
v_teste_irrig_auto = 0
irrigou = 0
GPIO.add_event_detect(13, GPIO.FALLING, callback=contador)

while(1):
    agora = datetime.now()
    dia_atual = agora.day
    mes_atual = agora.month
    ano_atual = agora.year
    hora_atual = agora.hour
    minuto_atual = agora.minute
    data_base_dia = '\''+ str(dia_atual - 1) + '/' + str(mes_atual) + '/' + str(ano_atual) + '\''
    hoje = date.today()
    intervalo = timedelta(1)
    ontem = hoje - intervalo
    amanha = hoje + intervalo
    bt_modo = GPIO.input(35)

    if (bt_modo == True):
      GPIO.output(16,1)
      GPIO.output(19,0)
      print("Modo Manual")
      lcdi2c.lcd_clear()
      lcdi2c.lcd_display_string("M", 1,15)
      alvo_irrigacao = 50000
      irrigador(alvo_irrigacao, 1, 7)
      display_L1 = "Irrigacao OK"
      display_L2 = "%s Litros" % contador_pulsos
      time.sleep(5)
      v_display = escreve_no_display(display_L1, display_L2, 5)
    else:
      GPIO.output(16,0)
      GPIO.output(19,1)
      print("Modo Automatico")
      time.sleep(5)
      lcdi2c.lcd_clear()
      lcdi2c.lcd_display_string("A", 1,15)


    if (minuto_ultimo != minuto_atual) and (modo_teste == 0):
        if hora_atual < 2:
            flag_media_diaria = 0
        print("Mudou o minuto da hora");
        minuto_ultimo = minuto_atual
        carimbo = datetime.now()
        time_stamp = carimbo.strftime("%d/%m/%Y %H:%M")
        try:
            umid = sensor_UHT.read_humidity()
            temp = sensor_UHT.read_temperature()
            orv = sensor_UHT.read_dewpoint()
            print("Leu o HTU21D");
            print("Ponto de orvalho : %s" %round(orv,1))
            if temp is None or umid is None:
                print("HTU21D com falha na leitura")
                temp, umid  = dados_ar_pws(ws_id)
                print(temp)
                print(umid)
        except Exception:
            print("Falha de HW no sensor HTU21D")
            temp, umid  = dados_ar_pws(ws_id)
                        
        try:
            carimbo = datetime.now()
            umid = round(float(umid),1)
            temp = round(float(temp),1)
            if (umid > 100):
              umid = umid_ant
              temp = temp_ar_ant
            else:
              umid_ant = umid
              temp_ar_ant = temp
              
            umid_solo_z1 = adc.read_adc(2, gain=Ganho_ADC);
            umid_solo_z2 = adc.read_adc(0, gain=Ganho_ADC);
            umid_solo_z2 = umid_solo_z2 - 2400
            umid_solo_z3 = adc.read_adc(1, gain=Ganho_ADC);
            umid_solo_z4 = adc1.read_adc(0, gain=Ganho_ADC1);
            umid_solo_z4 = umid_solo_z4 - 1800
            umid_solo_z5 = adc1.read_adc(1, gain=Ganho_ADC1);
            umid_solo_z6 = adc1.read_adc(2, gain=Ganho_ADC1);
            umid_solo_z6 = umid_solo_z6 - 1400
            umid_solo_z1_perc = (-0.002489*umid_solo_z1) + 77.291620
            umid_solo_z2_perc = (-0.002489*umid_solo_z2) + 77.291620
            umid_solo_z3_perc = (-0.002489*umid_solo_z3) + 77.291620
            umid_solo_z4_perc = (-0.002489*umid_solo_z4) + 77.291620
            umid_solo_z5_perc = (-0.002489*umid_solo_z5) + 77.291620
            umid_solo_z6_perc = (-0.002489*umid_solo_z6) + 77.291620
            sensor_1 = W1ThermSensor(W1ThermSensor.THERM_SENSOR_DS18B20, "0114501923aa")
            sensor_2 = W1ThermSensor(W1ThermSensor.THERM_SENSOR_DS18B20, "011450228caa")
            temp_solo = sensor_temp_solo.get_temperature();
            temp_solo_1 = sensor_1.get_temperature();
            temp_solo_2 = sensor_2.get_temperature();
            line = ser.readline()
            time_stamp = carimbo.strftime("%d/%m/%Y %H:%M")
            try:
                lux = round(readLight(),0)
            except Exception:
                lux = 0
            print(line)

            if line.startswith(b'$GNGGA'): #$GPGGA
                lat, _, lon = line.strip().split(b',')[2:5]
                lat = str(lat) + ' S'
                lon = str(lon) + ' W'

            #Matriz com os valores das medicoes
            vetor = []
            vetor.append(time_stamp)
            vetor.append(round(umid,1))
            vetor.append(round(temp,1))
            vetor.append(round(umid_solo_z1,1))
            vetor.append(round(umid_solo_z1_perc,1))
            vetor.append(round(umid_solo_z2,1))
            vetor.append(round(umid_solo_z2_perc,1))
            vetor.append(round(umid_solo_z3,1))
            vetor.append(round(umid_solo_z3_perc,1))
            vetor.append(round(umid_solo_z4,1))
            vetor.append(round(umid_solo_z4_perc,1))
            vetor.append(round(umid_solo_z5,1))
            vetor.append(round(umid_solo_z5_perc,1))
            vetor.append(round(umid_solo_z6,1))
            vetor.append(round(umid_solo_z6_perc,1))
            vetor.append(round(temp_solo_1,1))
            vetor.append(round(temp_solo_2,1))
            vetor.append(lux)
            vetor.append(lat)
            vetor.append(lon)
            resto_div_reg = minuto_atual % 4
            if resto_div_reg == 0:
                for i in range (1):
                    with open('/home/pi/registro.csv', 'a') as arquivo:
                        file_writer = csv.writer(arquivo)
                        file_writer.writerow(vetor)
                        print("Gravou .csv");
                print('Vai verificar a chuva hoje')
                chuva_hoje, mm_FIS, mm_FAO = chuva_dia(ws_id, date.today(), cod_cultura, ident_probe)
                print('A chuva hoje = %s mm' % chuva_hoje)
                sld_irrig_FIS = mm_FIS - float(chuva_hoje)
                if sld_irrig_FIS < 0:
                    sld_irrig_FIS = 0
                print('O saldo da lamina de irrigacao FIS hoje sera %s mm' % sld_irrig_FIS)
                sld_irrig_FAO = mm_FAO - float(chuva_hoje)
                if sld_irrig_FAO < 0:
                    sld_irrig_FAO = 0
                print('O saldo da lamina de irrigacao FAO hoje sera %s mm' % sld_irrig_FAO)                  

            if umid is not None and temp is not None:
                umid = round(umid,1)
                temp = round(temp,1)
                ilumi_env = lux
                umid_solo_z1 = round(umid_solo_z1,1)
                umid_solo_z1_perc = round(umid_solo_z1_perc,1)
                umid_solo_z2 = round(umid_solo_z2,1)
                umid_solo_z2_perc = round(umid_solo_z2_perc,1)
                umid_solo_z3 = round(umid_solo_z3,1)
                umid_solo_z3_perc = round(umid_solo_z3_perc,1)
                umid_solo_z4 = round(umid_solo_z4,1)
                umid_solo_z4_perc = round(umid_solo_z4_perc,1)
                umid_solo_z5 = round(umid_solo_z5,1)
                umid_solo_z5_perc = round(umid_solo_z5_perc,1)
                umid_solo_z6 = round(umid_solo_z6,1)
                umid_solo_z6_perc = round(umid_solo_z6_perc,1)
                temp_solo_1 = round(temp_solo_1,1)
                temp_solo_2 = round(temp_solo_2,1)
                print("Data/hora da medicao : %s " % time_stamp);
                display_L1 = "Data Hora"
                display_L2 = time_stamp
                v_display = escreve_no_display(display_L1, display_L2, t_display)

                print("Temperatura do ar : {0:0.1f}C Umidade do ar : {1:0.1f}%".format(temp,umid));
                display_L1 = "Temp ar: %d%s C" % (temp, chr(223))
                display_L2 = "Umid ar: %d %%" % (umid)
                v_display = escreve_no_display(display_L1, display_L2, t_display)

                print("Iluminancia do ambiente : " + str(lux) + " Lux");
                display_L1 = "Iluminancia amb"
                display_L2 = "%d Lux" % (ilumi_env)
                v_display = escreve_no_display(display_L1, display_L2, t_display)

                print("Umidade do solo Z1 :  %s  - %s %%" % (umid_solo_z1, umid_solo_z1_perc));
                display_L1 = "Umid solo Z1"
                display_L2 = "%d unid" % (umid_solo_z1)
                v_display = escreve_no_display(display_L1, display_L2, t_display)

                display_L1 = "Umid solo Z1"
                display_L2 = "%d %%" % (umid_solo_z1_perc)
                v_display = escreve_no_display(display_L1, display_L2, t_display)

                print("Umidade do solo Z2 :  %s  - %s %%" % (umid_solo_z2, umid_solo_z2_perc));
                display_L1 = "Umid solo Z2"
                display_L2 = "%d unid" % (umid_solo_z2)
                v_display = escreve_no_display(display_L1, display_L2, t_display)

                display_L1 = "Umid solo Z2"
                display_L2 = "%d %%" % (umid_solo_z2_perc)
                v_display = escreve_no_display(display_L1, display_L2, t_display)

                print("Umidade do solo Z3 :  %s  - %s %%" % (umid_solo_z3, umid_solo_z3_perc));
                display_L1 = "Umid solo Z3"
                display_L2 = "%d unid" % (umid_solo_z3)
                v_display = escreve_no_display(display_L1, display_L2, t_display)

                display_L1 = "Umid solo Z3"
                display_L2 = "%d %%" % (umid_solo_z3_perc)
                v_display = escreve_no_display(display_L1, display_L2, t_display)

                print("Umidade do solo Z4 :  %s  - %s %%" % (umid_solo_z4, umid_solo_z4_perc));
                display_L1 = "Umid solo Z4"
                display_L2 = "%d unid" % (umid_solo_z4)
                v_display = escreve_no_display(display_L1, display_L2, t_display)

                display_L1 = "Umid solo Z4"
                display_L2 = "%d %%" % (umid_solo_z4_perc)
                v_display = escreve_no_display(display_L1, display_L2, t_display)   

                print("Umidade do solo Z5 :  %s  - %s %%" % (umid_solo_z5, umid_solo_z5_perc));
                display_L1 = "Umid solo Z5"
                display_L2 = "%d unid" % (umid_solo_z5)
                v_display = escreve_no_display(display_L1, display_L2, t_display)

                display_L1 = "Umid solo Z5"
                display_L2 = "%d %%" % (umid_solo_z5_perc)
                v_display = escreve_no_display(display_L1, display_L2, t_display)   

                print("Umidade do solo Z6 :  %s  - %s %%" % (umid_solo_z6, umid_solo_z6_perc));
                display_L1 = "Umid solo Z6"
                display_L2 = "%d unid" % (umid_solo_z6)
                v_display = escreve_no_display(display_L1, display_L2, t_display)

                display_L1 = "Umid solo Z6"
                display_L2 = "%d %%" % (umid_solo_z6_perc)
                v_display = escreve_no_display(display_L1, display_L2, t_display)   

                print("Temperatura do solo 1: %s C" % temp_solo_1);
                display_L1 = "Temp solo 1:"
                display_L2 = "%d%s C" % (temp_solo_1, chr(223))
                v_display = escreve_no_display(display_L1, display_L2, t_display)

                print("Temperatura do solo 2: %s C" % temp_solo_2);
                display_L1 = "Temp solo 2:"
                display_L2 = "%d%s C" % (temp_solo_2, chr(223))
                v_display = escreve_no_display(display_L1, display_L2, t_display)

                print("Local da base de sensores =  lat %s, lon %s " % (lat, lon));
                latitude = lat[2:6] + ' S'
                longitude = lon[2:7] + ' W'
                display_L1 = "Latitude"
                display_L2 = "%s" % (latitude)
                v_display = escreve_no_display(display_L1, display_L2, t_display)

                display_L1 = "Longitude"
                display_L2 = "%s" % (longitude)
                v_display = escreve_no_display(display_L1, display_L2, t_display)
                obs = 'Sem anormalidades'
                localizacao_atual = check_localiz(ident_probe)
                latitude_atual = localizacao_atual[4:8]
                longitude_atual = localizacao_atual[14:19]
                probe_id = localizacao_atual[22:23]
                if (latitude != ' S' and latitude != latitude_atual and len(longitude) > 6):
                    print("Vai atualizar a localizacao do probe")
                    latitude_atual = latitude
                    longitude_atual = longitude
                    resultado = atualiza_localiz(latitude, longitude, ident_probe)
                    print(resultado)
                else:
                    v1 = 0    

                resto_div_1 = minuto_atual % 28
                resto_div_2 = minuto_atual % 29
                resto_div_3 = minuto_atual % 30
                resto_div_4 = minuto_atual % 56
                resto_div_5 = minuto_atual % 57
                resto_div_6 = minuto_atual % 58
                if ((minuto_atual > 19 and minuto_atual < 26) or (minuto_atual > 49 and minuto_atual < 56)) and (gravou == 0):
                    gravou = 1
                    sensores_id = 0
                    print("Vai gravar no banco")
                    resultado = grava_dados_no_banco(ident_probe, time_stamp, umid, temp, ilumi_env, temp_solo_1, temp_solo_2, obs, umid_solo_z1, umid_solo_z1_perc, umid_solo_z2, umid_solo_z2_perc, umid_solo_z3, umid_solo_z3_perc, umid_solo_z4, umid_solo_z4_perc, umid_solo_z5, umid_solo_z5_perc, umid_solo_z6, umid_solo_z6_perc)
                    print(resultado)

                if ((minuto_atual > 28 and minuto_atual < 38) or (minuto_atual < 15)) and (gravou == 1):
                    gravou = 0
                if minuto_atual > 0 and minuto_atual < 15:
                    gravou_1 = 0
                    gravou_2 = 0
                    v_gravacao_hora_1 = 0
                    v_gravacao_hora_2 = 0
                    if hora_atual <= 23 and minuto_atual < 50:
                        gravou_previsao = 0
                if minuto_atual > 20 and gravou_1 == 0:
                    v_gravacao_hora_1 = 1
                if minuto_atual > 50 and gravou_2 == 0:
                    v_gravacao_hora_2 = 1
                if ((v_gravacao_hora_1 == 1) or (v_gravacao_hora_2 == 1)) and (gravou_1 == 0 or gravou_2 == 0):

                  try:
                    print("Vai pegar dados da PWS")
                    dados_pws = aquisic_dados_pws(time_stamp)
                    if minuto_atual < 50 and dados_pws == 1:
                        v_gravacao_hora_1 = 0
                        gravou_1 = 1
                    if minuto_atual >= 50 and minuto_atual < 59 and dados_pws == 1:
                        v_gravacao_hora_1 = 0
                        v_gravacao_hora_2 = 0
                        gravou_2 = 1
                  except Exception:
                    print("Falha ao carregar / gravar dados da pws")
            else:
                print("Falha ao carregar dados do sensor")
        except Exception:
            resultado = "Erro na medicao do minuto: " + str(minuto_atual) 
            print("Falha na medicao do minuto %s " % minuto_atual);
        display_L1 = "Fim ciclo"
        display_L2 = "Proxima medicao"
        v_display = escreve_no_display(display_L1, display_L2, t_display)
        teste = 0
        if (minuto_atual > 1 and minuto_atual < 13 and hora_atual > 25):
            v_teste_irrig_auto = 1
        else:
            v_teste_irrig_auto = 0

        if (hora_atual == 20 and minuto_atual > 5 and irrigou == 0) or (v_teste_irrig_auto == 1):
            print('Vai verificar a chuva hoje')
            chuva_hoje, mm_FIS, mm_FAO = chuva_dia(ws_id, date.today(), cod_cultura, ident_probe)
            print('A chuva hoje = %s mm' % chuva_hoje)
            sld_irrig_FIS = mm_FIS - float(chuva_hoje)
            if sld_irrig_FIS < 0:
                sld_irrig_FIS = 0
            print('O saldo da lamina de irrigacao FIS hoje sera %s mm' % sld_irrig_FIS)
            sld_irrig_FAO = mm_FAO - float(chuva_hoje)
            if sld_irrig_FAO < 0:
                sld_irrig_FAO = 0
            print('O saldo da lamina de irrigacao FAO hoje sera %s mm' % sld_irrig_FAO)                  
            irrig_FAO = sld_irrig_FAO            
            irrig_FIS = sld_irrig_FIS            
            tempo_irrig_FAO = round((0.26 * irrig_FAO) + 7,0)
            if irrig_FAO > 0:
                irrig_pulsos = round((irrig_FAO * 12500),0) + 18750
            else:
                irrig_pulsos = round((irrig_FAO * 12500),0)
            tempo_irrig_FIS = round((0.26 * irrig_FIS) + 10,0)

            i_auto = 1
            print("Iniciando processo de irrigação FAO com %s mm" % irrig_FAO)
            print("Enviando %s pulsos para o FAO" % irrig_pulsos)
            irrigador(irrig_pulsos, 3, tempo_irrig_FAO)
            contador_pulsos = 0
            time.sleep(7)
            lcdi2c.lcd_clear()
            time.sleep(23)
            if irrig_FIS > 0:
                irrig_pulsos = round((irrig_FIS * 12500),0) + 18750
                irrig_pulsos = round((1.5 * irrig_pulsos),0)
            else:
                irrig_pulsos = round((irrig_FIS * 12500),0)            
            print("Iniciando processo de irrigação FIS com %s mm" % irrig_FIS)
            print("Enviando %s pulsos para o FIS" % irrig_pulsos)
            irrigador(irrig_pulsos, 2, tempo_irrig_FIS)
            contador_pulsos = 0            
            i_auto = 0
            irrigou = 1
            
        if (hora_atual == 23 and minuto_atual > 50 and gravou_previsao == 0) or (teste == 1):
            print("Vai atualizar a previsão do tempo para os proximos dias")
            chama_previsao(hoje)
            gravou_previsao = 1

        try:
            if (hora_atual == 2 and minuto_atual > 38 and flag_media_diaria == 0) or (teste == 1):
                print("Etapa 1")
                dia_corr, n, nascer_sol = extrai_dados_astron(ontem.strftime("%d"), ontem.strftime("%m"))
                print("Etapa 2")
                hora_nascer_sol, hora_por_sol = determina_nasc_por_sol(nascer_sol, n)

                data_base1 = '\'' + ontem.strftime("%d/%m/%Y") + ' ' + hora_nascer_sol + '\''
                data_base2 = '\'' + ontem.strftime("%d/%m/%Y") + ' ' + hora_por_sol + '\''
                data_base_geral1 = '\'' + ontem.strftime("%d/%m/%Y") + ' ' + '00:00' + '\''
                data_base_geral2 = '\'' + ontem.strftime("%d/%m/%Y") + ' ' + '23:59' + '\''
                flag_media_diaria = 1
                irrigou = 0

                print("Etapa 3")
                tmin, tmax, tmed, rhmax, rhmed = extrai_dados_ar_probe(data_base1, data_base2)
                print("Etapa 4")
                umid_z1_dia, umid_z2_dia, umid_zmedia_dia, umid_zmedia_dia_12 = extrai_dados_solo_probe(data_base_geral1, data_base_geral2)
                print("Etapa 5")
                vento_med = dados_hist_ws(ontem.strftime("%d/%m/%Y"), ws_id)
                print("Etapa 6")
                altit_vento = info_estacao_met(ws_id)
                print("Etapa 7")
                eto = evapotranspiracao(dia_corr, tmax, tmin, tmed, rhmax, n, vento_med, altit_vento, rhmed)
                print("Evapotranspiração de referencia do dia %s foi de %s mm" % (ontem.strftime("%d/%m/%Y"), str(round(eto,1))))
                dap, nome_cultura = determina_dap_atual(hoje.strftime("%d/%m/%Y"), ident_probe, cod_cultura)
                print("Etapa 8")
                kc, z_rad = determina_z_rad_kc(dap, cod_cultura)                
                print("Etapa 9")
                et_c = eto * kc
                print("Evapotranspiração da cultura dia %s foi de %s mm" % (ontem.strftime("%d/%m/%Y"), str(round(et_c,1))))
                if (z_rad < 10):
                  v_pdc = umid_zmedia_dia
                else:
                  v_pdc = umid_zmedia_dia_12
                print("Etapa 10")  
                p_chuva, c_chuva = extrai_previsao_tempo(hoje.strftime("%d/%m/%Y"), ws_id)
                print("Extraiu a previsão do tempo")
                res = grava_variaveis_fuzzy(cod_cultura, ident_probe, hoje.strftime("%d/%m/%Y"), dap, p_chuva, c_chuva, v_pdc, et_c, eto)
                print(res)
                irrigacao_dia = inferencia_fuzzy(et_c, cod_cultura, hoje.strftime("%d/%m/%Y"), ontem.strftime("%d/%m/%Y"), ident_probe, mes_atual)
                print("A irrigação pelo prescritor Fuzzy para %s é de %s mm de água" % (hoje.strftime("%d/%m/%Y"), round(irrigacao_dia,0)))

                atd_fao = informacoes_modelo_fao_irrig(ident_probe, cod_cultura)
                f_pdc = extrai_fpdc_fao(round(et_c,0), cod_cultura)
                arm_fao = atd_fao*f_pdc*z_rad
                arm_fao_dispon, irrig_fao_ontem = balanco_arm_fao(cod_cultura, arm_fao, ws_id)
                print("A lâmina de água disponível pelo FAO para a planta hoje é %s" % arm_fao_dispon)

                t_irrig = arm_fao_dispon / et_c

                if (t_irrig >= 2):
                  l_liq_fao = 0
                else:
                  l_liq_fao = et_c * 1.0
                  t_irrig = 1
                res = grava_variaveis_fao(cod_cultura, probe_id, hoje.strftime("%d/%m/%Y"), dap, t_irrig, l_liq_fao, et_c, eto)
                print("A irrigação pelo modelo FAO para %s é de %s mm" % (hoje.strftime("%d/%m/%Y"), round(l_liq_fao,2)))

                irrig_fao_ml, irrig_fuzzy_ml = conversao_irrig(round(l_liq_fao,1), round(irrigacao_dia,1))

                res = grava_fao_fuzzy_na_evolucao(ws_id, hoje.strftime("%d/%m/%Y"), arm_fao, irrig_fao_ml, irrig_fuzzy_ml, cod_cultura, ident_probe, ontem.strftime("%d/%m/%Y"), dap, eto, et_c, v_pdc, irrigacao_dia, irrig_fao_ontem, t_irrig, arm_fao_dispon)
                print(res)
        except Exception:
            print("Falha na extração das médias")
        print("Fim da medição para o horário %s" % time_stamp)
        print(" ");
