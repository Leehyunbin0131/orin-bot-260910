"""Local Whisper.cpp speech recognition for browser-supplied PCM WAV audio."""
import io
import json
import math
from pathlib import Path
import subprocess
import tempfile
import threading
import wave
import numpy as np


class LocalSTT:
    def __init__(self):
        self.binary=Path.home()/'.local/opt/whisper.cpp-v1.8.4/build/bin/whisper-cli'
        self.model=Path.home()/'.local/share/orinbot/stt/ggml-small-q5_1.bin'
        self.lock=threading.Lock()

    def ready(self): return self.binary.is_file() and self.model.is_file()

    def transcribe(self,audio,language='ko'):
        if not self.ready(): raise ValueError('로컬 STT가 설치되지 않았습니다. setup_guide_stt.sh를 실행하세요.')
        if not self.lock.acquire(blocking=False): raise ValueError('음성을 인식 중입니다. 잠시 기다려 주세요.')
        try:
            with wave.open(io.BytesIO(audio),'rb') as wav:
                if wav.getframerate()!=16000 or wav.getnchannels()!=1 or wav.getsampwidth()!=2:
                    raise ValueError('16kHz 모노 PCM16 WAV만 지원합니다.')
                duration=wav.getnframes()/16000
                if not .3<=duration<=20: raise ValueError('음성 길이는 0.3~20초여야 합니다.')
                samples=np.frombuffer(wav.readframes(wav.getnframes()),dtype='<i2').astype(float)/32768
                if len(samples)!=round(duration*16000) or math.sqrt(float(np.mean(samples*samples)))<.003:
                    raise ValueError('목소리가 들리지 않습니다. 마이크를 확인하고 다시 말씀해 주세요.')
            with tempfile.TemporaryDirectory(prefix='orinbot_stt_') as temp:
                folder=Path(temp);path=folder/'speech.wav';path.write_bytes(audio);output=folder/'result'
                r=subprocess.run(['nice','-n','10',str(self.binary),'-m',str(self.model),'-f',str(path),
                    '-l',language,'-t','3','-bs','1','-bo','1','-nf','-ojf','-of',str(output)],
                    stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=120)
                if r.returncode: raise RuntimeError('로컬 STT 처리에 실패했습니다.')
                result=json.loads(output.with_suffix('.json').read_text())
                text=' '.join(segment['text'].strip() for segment in result['transcription']).strip()
                if not text: raise ValueError('음성을 인식하지 못했습니다. 다시 말씀해 주세요.')
                return text
        except (wave.Error,EOFError): raise ValueError('오디오 파일 형식이 올바르지 않습니다.')
        finally:self.lock.release()
