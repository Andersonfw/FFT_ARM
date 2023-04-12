#  Copyright (c) 2023. Vieira Filho Tecnologia

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
VARIÁVEIS GLOBAIS
"""

PORT = '/dev/ttyACM0'
BAUD_RATE = '115200'
TIMEOUT = '3'
PARITY = 'N'
STOPBITS = '1'
BYTESIZE = '8'

# Variáveis de estado
xh1 = 0
xh2 = 0
yh1 = 0
yh2 = 0


# ports = serial.tools.list_ports.comports()
# print('Portas seriais disponíveis: ')
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

def filtervariables(f_s, f_c):
    global FS, TS, FC, C, b0, b1, b2, a0, a1, a2
    FS = f_s
    FC = f_c
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
    # Além das opções structs=BOOL, xonxoff=BOOL, e dsrdtr=BOOL
    print('\n###############################################\n')
    print('\nStatus Porta: %s ' % (comport.isOpen()))
    print('Device conectado: %s ' % comport.name)
    # print ('Dump da configuracão:\n %s ' % (comport))
    print('pressione CRTR+C para parar o programa.')
    print('\n###############################################\n')

    return comport


""" main """
if __name__ == '__main__':
    port = conectserial()
    if not port.isOpen():
        sys.exit('ERRO. rro ao abrir a porta serial')
    else:
        print("porta serial Aberta")

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
    filtervariables(int(fs), int(fc))

    print("Enviando dado para verificação de configuração da FFT, SSB ou DSB")
    comwrite = "M"
    port.write(comwrite.encode())
    model = port.readline().decode().strip()
    model = int(model)
    if model == 1:
        print("O tipo de FFT é DSB")
    else:
        print("O tipo de FFT é SSB")

    print("Enviando dado para recebimento do sinal de entrada")
    comwrite = "I"
    port.write(comwrite.encode())
    signal_in = port.readline().decode().strip()
    adc_value = np.array(signal_in.split(','), dtype=float)

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
    N_BINS = len(fft_np)
    F_RES = FS / N_BINS
    frq_res = np.arange(0, N_BINS * F_RES, F_RES)

    if model == 0:
        fft_np = fft_np[range(int(n / 2))]
        frq_res = frq_res[range(int(n / 2))]
        fft_np = 2 * abs(fft_np)
    else:
        fft_np = abs(fft_np)

    ax[1].plot(frq_res / 1e3, fft_np, 'r', label='FFT com numpy')
    ax[1].set_xlabel('Frequência kHz')
    ax[1].set_ylabel('Amplitude')
    ax[1].legend()

    # fft_ARM = fft_ARM[range(int(512))]
    # frq_res = frq_res[range(int(len(fft_ARM)))]
    N_BINS = len(fft_ARM)
    if model == 0:
        F_RES = FS / 2 / N_BINS
        fft_ARM = 2 * fft_ARM
    else:
        F_RES = FS / N_BINS
    frq_res = np.arange(0, N_BINS * F_RES, F_RES)
    # ax[2].plot(frq_res / 1e3, 2 * abs(fft_ARM/len(fft_ARM)), 'g', label='FFT ARM')
    ax[2].plot(frq_res / 1e3, fft_ARM, 'g', label='FFT ARM')
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

    scale = 2.469  # fator de scala do filtro para ajustar a 3.3V
    ax[3].plot(frq / 1e3, 2 * abs(Y) * scale, 'y', label='Filtro sallen key')
    ax[3].set_xlabel('Frequência kHz')
    ax[3].set_ylabel('Amplitude')
    ax[3].legend()

    plt.tight_layout()

    max_np = max(fft_np)
    max_fft_np = np.argmax(fft_np) * F_RES / 2
    max_fft_arm = np.max(fft_ARM)
    max_freq_arm = np.argmax(fft_ARM) * F_RES
    filter_max = 2 * scale * max(abs(Y))
    print("Amplitude máxima da FFT no numpy é", max_np, " na frequência de ", max_fft_np)
    print("Amplitude máxima da FFT no ARM é", max_fft_arm, " na frequência de ", max_freq_arm)
    print("Amplitude máxima de saída do filtro é", filter_max)

    plt.show()

    sys.exit("Finalizado \r\n")  # sair
