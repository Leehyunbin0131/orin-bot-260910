"""Local Whisper.cpp speech recognition for browser-supplied PCM WAV audio."""
import io
import json
import math
import os
from pathlib import Path
import re
import subprocess
import tempfile
import threading
import wave
import numpy as np


def cuda_backend_used(log):
    """A GPU-enabled binary may fall back to CPU; require actual CUDA selection."""
    return (re.search(r'whisper_backend_init_gpu: using CUDA\d+ backend', log) is not None
            and re.search(r'whisper_backend_init_gpu: (?:failed to initialize|no GPU found)', log) is None)


class LocalSTT:
    def __init__(self):
        self.binary=Path.home()/'.local/opt/whisper.cpp-v1.8.4/build-cuda/bin/whisper-cli'
        self.model=Path.home()/'.local/share/orinbot/stt/ggml-small-q5_1.bin'
        self.runtime=Path.home()/'.local/share/orinbot/stt/cuda.json'
        self.lock=threading.Lock()

    def ready(self): return self.binary.is_file() and self.model.is_file() and self.runtime.is_file()

    def environment(self):
        settings=json.loads(self.runtime.read_text())
        library=Path(settings['library_path'])
        if not library.is_dir():raise RuntimeError('CUDA 라이브러리를 찾지 못했습니다. setup_guide_stt.sh를 다시 실행하세요.')
        env=os.environ.copy()
        env['LD_LIBRARY_PATH']=str(library)+(':'+env['LD_LIBRARY_PATH'] if env.get('LD_LIBRARY_PATH') else '')
        return env

    def transcribe(self,audio,language='ko'):
        if not self.ready(): raise ValueError('GPU STT가 설치되지 않았습니다. setup_guide_stt.sh를 실행하세요.')
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
                # Full JSON includes byte-level tokens that may split a Korean
                # UTF-8 character. Only complete segment text is needed here.
                r=subprocess.run(['nice','-n','10',str(self.binary),'-m',str(self.model),'-f',str(path),
                    '-l',language,'-t','3','-dev','0','-bs','1','-bo','1','-nf','-oj','-of',str(output)],
                    stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=120,env=self.environment())
                if r.returncode: raise RuntimeError('GPU STT 처리에 실패했습니다. CUDA 설치와 GPU 메모리 상태를 확인하세요.')
                if not cuda_backend_used(r.stderr.decode(errors='replace')):
                    raise RuntimeError('STT가 CUDA GPU를 사용하지 못했습니다. CPU 결과로 안내를 시작하지 않습니다.')
                try:
                    result=json.loads(output.with_suffix('.json').read_text(encoding='utf-8'))
                except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                    raise RuntimeError('음성 인식 결과 파일이 올바르지 않습니다. 다시 녹음해 주세요.') from exc
                text=' '.join(segment['text'].strip() for segment in result['transcription']).strip()
                if not text: raise ValueError('음성을 인식하지 못했습니다. 다시 말씀해 주세요.')
                return text
        except (wave.Error,EOFError): raise ValueError('오디오 파일 형식이 올바르지 않습니다.')
        finally:self.lock.release()
