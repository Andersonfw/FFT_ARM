#  Copyright (c) 2023. Vieira Filho Tecnológia

"""
Created on abril 11 14:37:44 2023
@author: vieirafilho
"""
import numpy as np
import matplotlib.pyplot as plt
import serial
import serial.tools.list_ports
import sys

"""
VARIAVEIS GLOBAIS
"""

PORT = '/dev/ttyACM0'
BAUD_RATE = '115200'
TIMEOUT = '3'
PARITY = 'N'
STOPBITS = '1'
BYTESIZE = '8'

# Variaveis de estado
xh1 = 0
xh2 = 0
yh1 = 0
yh2 = 0

# ports = serial.tools.list_ports.comports()
# print('Portas seriais disponiveis:')
# for p in ports:
#     print(p.device)
# print(len(ports), 'Portas localizadas.')
#
# PORT = input("\n\nDigite o nome da porta desejada.\r\nPORTA: ")
# for p in ports:
#     if (PORT == p.device):
#         print(f' Porta serial {PORT} OK')
#     else:
#         sys.exit(f'Porta {PORT} serial não encontrada!!!')

def filtervariables(fs, fc):
    global FS, FC, TS, C, b0, b1, b2, a0, a1, a2
    FS = fs
    FC = fc
    TS = 1 / FS
    zeta = 0.707
    C = 1 / (np.tan(np.pi * FC / FS))
    b0 = 1 / (1 + 2 * zeta * C + C * C)
    b1 = 2 * b0
    b2 = b0
    a0 = 1
    a1 = 2 * b0 * (1 - C * C)
    a2 = b0 * (1 - 2 * zeta * C + C * C)
def conectserial():
    comport = serial.Serial(PORT,
                            int(BAUD_RATE),
                            timeout=int(TIMEOUT),
                            bytesize=int(BYTESIZE),
                            stopbits=int(STOPBITS),
                            parity=PARITY)
    # Alem das opcoes rtscts=BOOL, xonxoff=BOOL, e dsrdtr=BOOL
    print('\n###############################################\n')
    print('\nStatus Porta: %s ' % (comport.isOpen()))
    print('Device conectado: %s ' % comport.name)
    # print ('Dump da configuracao:\n %s ' % (comport))
    print('prescione CRTR+C para parar o programa.')
    print('\n###############################################\n')

    return comport


""" main """
if __name__ == '__main__':
    port = conectserial()
    if not port.isOpen():
        sys.exit('ERRO. rro ao abrir a porta serial')
    else:
        print("porta serial Aberta")

    print("Enviando dado para recebimento do sinal de entrada")
    comwrite = "I"
    port.write(comwrite.encode())
    signal_in = port.readline().decode().strip()
    adc_value = np.array(signal_in.split(','), dtype=float)

    print("Enviando dado para recebimento da frequência de amostragem")
    comwrite = "S"
    port.write(comwrite.encode())
    fs = port.readline().decode().strip()

    print("Enviando dado para recebimento da frequência de corte")
    comwrite = "C"
    port.write(comwrite.encode())
    fc = port.readline().decode().strip()

    print("Frequência de amostragem em:", fs, "Hz")
    print("Frequência de corte em:", fc, "Hz")
    filtervariables(int(fs),int(fc))

    print("Enviando dado para recebimento da FFT calculada no ARM")
    comwrite = "O"
    port.write(comwrite.encode())
    signal_out = port.readline().decode().strip()
    fft_ARM = m = np.array(signal_out.split(','), dtype=float)
    port.close()  # Fecha conexão serial

    print("Montando plots")
    n = len(adc_value)
    t = np.arange(0, n * TS, TS)  # vetor de tempo

    fig, ax = plt.subplots(4, 1)
    ax[0].plot(t * 1e3, adc_value, label='Sinal Discretizado pelo ADC ARM')
    ax[0].set_xlabel('Tempo ms')
    ax[0].set_ylabel('Amplitude')
    ax[0].legend()

    fft_np = np.fft.fft(adc_value) / n
    k = np.arange(len(fft_np))
    frq_res = k * FS / len(fft_np)

    fft_np = fft_np[range(int(n / 2))]
    frq_res = frq_res[range(int(n / 2))]

    ax[1].plot(frq_res / 1e3, 2 * abs(fft_np), 'r', label='FFT com numpy')
    ax[1].set_xlabel('Frequência kHz')
    ax[1].set_ylabel('Amplitude')
    ax[1].legend()

    # fft_ARM = fft_ARM[range(int(512))]
    # frq_res = frq_res[range(int(512))]
    ax[2].plot(frq_res / 1e3, 2 * fft_ARM / n, 'g', label='FFT ARM')
    ax[2].set_xlabel('Frequência kHz')
    ax[2].set_ylabel('Amplitude')
    ax[2].legend()

    y = np.zeros(len(adc_value))

    for n in range(0, len(adc_value)):
        y[n] = b0 * adc_value[n] + b1 * xh1 + b2 * xh2 - a1 * yh1 - a2 * yh2

        yh2 = yh1
        yh1 = y[n]

        xh2 = xh1
        xh1 = adc_value[n]

    n = len(y)  # tamanho do sinal
    k = np.arange(n)  # vetor em k
    T = n / FS
    frq = k / T  # os dois lados do vetor de frequencia
    frq = frq[range(int(n / 2))]  # apenas um lado

    Y = np.fft.fft(y) / n  # calculo da fft e normalização por n
    Y = Y[range(int(n / 2))]

    ax[3].plot(frq / 1e3, 2 * abs(Y), 'y', label='Filtro sallen key')
    ax[3].set_xlabel('Frequência kHz')
    ax[3].set_ylabel('Amplitude')
    ax[3].legend()

    plt.tight_layout()
    plt.show()
    sys.exit("Finalizado \r\n")  # sair
