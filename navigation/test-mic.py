import pyaudio

#Giraff aoudio conflict
name_device = ["ORBBEC", "ASTRA"]

def audio_devices():
    device_info = []

    p = pyaudio.PyAudio()
    host_api = p.get_host_api_info_by_type(pyaudio.paALSA)
    for i in range(host_api.get('deviceCount')):
        device = p.get_device_info_by_host_api_device_index(host_api.get('index'), i)
        if (device.get('maxInputChannels') > 0):
            if any([name in device.get('name') for name in name_device]):
                device_info.append(device)

    p.terminate()

    return device_info

if __name__ == '__main__':
    #mic_number()
    RATE = 16000
    CHUNK = 8000
    
    print audio_devices()