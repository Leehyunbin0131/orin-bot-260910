import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import wave

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from guide_stt import LocalSTT, cuda_backend_used


class CudaSTTTests(unittest.TestCase):
    def test_cuda_selection_required_not_just_gpu_flag(self):
        self.assertTrue(cuda_backend_used('whisper_backend_init_gpu: using CUDA0 backend\n'))
        for log in ('use gpu = 1\nCUDA = 1', 'whisper_backend_init_gpu: no GPU found',
                    'whisper_backend_init_gpu: using CPU backend',
                    'whisper_backend_init_gpu: using CUDA0 backend\nwhisper_backend_init_gpu: failed to initialize CUDA0 backend'):
            self.assertFalse(cuda_backend_used(log))

    def test_inference_selects_gpu_and_cpu_fallback_is_rejected(self):
        audio=io.BytesIO()
        with wave.open(audio,'wb') as writer:
            writer.setnchannels(1);writer.setsampwidth(2);writer.setframerate(16000)
            writer.writeframes(b'\x00\x10'*16000)
        with tempfile.TemporaryDirectory() as temp:
            directory=Path(temp);stt=LocalSTT()
            self.assertIn('build-cuda',str(stt.binary))
            stt.binary=directory/'whisper-cli';stt.binary.touch()
            stt.model=directory/'model.bin';stt.model.touch()
            stt.runtime=directory/'cuda.json'
            self.assertFalse(stt.ready())
            stt.runtime.write_text(json.dumps({'library_path':str(directory)}))
            self.assertTrue(stt.ready())
            def run(args,**kwargs):
                self.assertEqual(args[args.index('-dev')+1],'0')
                self.assertNotIn('-ng',args)
                self.assertTrue(kwargs['env']['LD_LIBRARY_PATH'].startswith(str(directory)))
                result={'transcription':[{'text':'101호로 가줘'}]}
                # whisper.cpp full JSON can split a Hangul character between
                # byte tokens even when the complete segment is valid UTF-8.
                if '-ojf' in args:
                    result['transcription'][0]['tokens']=[{'text':'PART1'},{'text':'PART2'}]
                encoded=json.dumps(result,ensure_ascii=False).encode('utf-8')
                encoded=encoded.replace(b'PART1',b'\xed\x98').replace(b'PART2',b'\xb8')
                Path(args[args.index('-of')+1]).with_suffix('.json').write_bytes(encoded)
                return SimpleNamespace(returncode=0,stderr=b'whisper_backend_init_gpu: using CUDA0 backend\n\xed\x98')
            with patch('guide_stt.subprocess.run',side_effect=run):
                self.assertEqual(stt.transcribe(audio.getvalue()),'101호로 가줘')
            def malformed_segment(args,**kwargs):
                result=run(args,**kwargs)
                Path(args[args.index('-of')+1]).with_suffix('.json').write_bytes(
                    b'{"transcription":[{"text":"101\xed\x98"}]}')
                return result
            with patch('guide_stt.subprocess.run',side_effect=malformed_segment):
                with self.assertRaisesRegex(RuntimeError,'결과 파일'):
                    stt.transcribe(audio.getvalue())
            self.assertFalse(stt.lock.locked())
            for result in (SimpleNamespace(returncode=0,stderr=b'whisper_backend_init_gpu: no GPU found'),
                           SimpleNamespace(returncode=1,stderr=b'CUDA out of memory')):
                with patch('guide_stt.subprocess.run',return_value=result),self.assertRaises(RuntimeError):
                    stt.transcribe(audio.getvalue())
                self.assertFalse(stt.lock.locked())


if __name__=='__main__':unittest.main()
