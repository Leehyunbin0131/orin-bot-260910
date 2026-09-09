#!/usr/bin/env bash
set -euo pipefail
stt_source="$HOME/.local/opt/whisper.cpp-v1.8.4"
mkdir -p "$HOME/.local/opt" "$HOME/.local/share/orinbot/stt"
if [[ ! -d "$stt_source" ]]; then
  git clone --depth 1 --branch v1.8.4 https://github.com/ggml-org/whisper.cpp.git "$stt_source"
fi
# Prefer an explicitly selected/system toolkit. This Jetson has the CUDA driver
# but no toolkit; extract matching NVIDIA APT packages into a user-owned prefix.
cuda_root=${ORINBOT_CUDA_ROOT:-}
if [[ -z "$cuda_root" && -x /usr/local/cuda/bin/nvcc ]]; then
  cuda_root=/usr/local/cuda
fi
if [[ -z "$cuda_root" ]]; then
  cuda_root="$HOME/.local/opt/orinbot-cuda-13.2/usr/local/cuda-13.2"
  if [[ ! -f "$HOME/.local/opt/orinbot-cuda-13.2/complete.json" ]]; then
    /usr/bin/python3 - <<'PY'
import apt, ctypes, hashlib, json, platform, subprocess
from pathlib import Path
if platform.machine() != 'aarch64':
 raise SystemExit('Set ORINBOT_CUDA_ROOT to an installed CUDA toolkit on this platform.')
cuda=ctypes.CDLL('libcuda.so.1'); version=ctypes.c_int()
if cuda.cuInit(0) or cuda.cuDriverGetVersion(ctypes.byref(version)) or version.value < 13020:
 raise SystemExit('The automatic CUDA 13.2 setup requires the matching Jetson CUDA driver. Set ORINBOT_CUDA_ROOT for another toolkit.')
names=['cuda-nvcc-13-2','cuda-cudart-dev-13-2','cuda-cudart-13-2','cuda-cccl-13-2',
 'cuda-culibos-dev-13-2','cuda-driver-dev-13-2','cuda-crt-13-2','libnvvm-13-2',
 'libnvptxcompiler-13-2','libcublas-13-2','libcublas-dev-13-2','cuda-toolkit-13-2-config-common']
cache=apt.Cache(); selected=[]
for name in names:
 if name not in cache or cache[name].candidate is None:
  raise SystemExit(f'NVIDIA Jetson APT package missing: {name}. Check the configured Jetson repository.')
 package=cache[name].candidate
 if not any(origin.trusted and origin.site=='repo.download.nvidia.com' for origin in package.origins):
  raise SystemExit(f'Expected a trusted NVIDIA Jetson package: {name}')
 selected.append(package)
downloads=Path.home()/'.cache/orinbot/cuda-13.2-debs'; downloads.mkdir(parents=True,exist_ok=True)
target=Path.home()/'.local/opt/orinbot-cuda-13.2'; target.mkdir(parents=True,exist_ok=True)
subprocess.run(['apt-get','download',*[p.package.name+'='+p.version for p in selected]],cwd=downloads,check=True)
manifest=[]
for package in selected:
 archive=downloads/Path(package.filename).name
 with archive.open('rb') as source: digest=hashlib.file_digest(source,'sha256').hexdigest()
 if digest!=package.sha256:raise SystemExit(f'CUDA package checksum mismatch: {archive.name}')
 subprocess.run(['dpkg-deb','--extract',str(archive),str(target)],check=True)
 manifest.append({'package':package.package.name,'version':package.version,'sha256':digest})
(target/'complete.json').write_text(json.dumps(manifest,indent=2)+'\n')
print('CUDA toolkit extracted:',target/'usr/local/cuda-13.2')
PY
  fi
fi
[[ -x "$cuda_root/bin/nvcc" ]] || { echo "CUDA compiler missing: $cuda_root/bin/nvcc" >&2; exit 1; }
cuda_library="$cuda_root/lib64"
[[ -d "$cuda_library" ]] || cuda_library="$cuda_root/targets/aarch64-linux/lib"
cmake -S "$stt_source" -B "$stt_source/build-cuda" -DCMAKE_BUILD_TYPE=Release \
  -DGGML_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=87 -DGGML_CUDA_FA_ALL_QUANTS=OFF \
  -DCMAKE_CUDA_COMPILER="$cuda_root/bin/nvcc" -DCUDAToolkit_ROOT="$cuda_root" \
  -DCMAKE_BUILD_RPATH="$cuda_library" -DCMAKE_EXE_LINKER_FLAGS="-Wl,-rpath-link,$cuda_library" \
  -DWHISPER_BUILD_TESTS=OFF
stt_build_jobs=${ORINBOT_STT_BUILD_JOBS:-4}
[[ "$stt_build_jobs" =~ ^[1-4]$ ]] || { echo 'ORINBOT_STT_BUILD_JOBS must be 1..4' >&2; exit 1; }
cmake --build "$stt_source/build-cuda" --target whisper-cli -j "$stt_build_jobs"
python3 - <<'PY'
from pathlib import Path
import urllib.request,os
p=Path.home()/'.local/share/orinbot/stt/ggml-small-q5_1.bin'
if not p.is_file():
 urllib.request.urlretrieve('https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-small-q5_1.bin',str(p)+'.part')
 os.replace(str(p)+'.part',p)
print('Local STT ready:',p)
PY
# Verify actual CUDA inference before publishing the runtime configuration.
/usr/bin/python3 - "$cuda_root" <<'PY'
import json,os,re,subprocess,sys
from pathlib import Path
root=Path(sys.argv[1]).resolve()
library=root/'targets/aarch64-linux/lib'
if not library.is_dir():library=root/'lib64'
source=Path.home()/'.local/opt/whisper.cpp-v1.8.4'
model=Path.home()/'.local/share/orinbot/stt/ggml-small-q5_1.bin'
environment=os.environ.copy()
environment['LD_LIBRARY_PATH']=str(library)+(':'+environment['LD_LIBRARY_PATH'] if environment.get('LD_LIBRARY_PATH') else '')
result=subprocess.run([str(source/'build-cuda/bin/whisper-cli'),'-m',str(model),'-f',str(source/'samples/jfk.wav'),
 '-l','en','-t','3','-dev','0','-bs','1','-bo','1','-nf'],env=environment,capture_output=True,timeout=120)
log=result.stderr.decode(errors='replace')
(model.parent/'cuda-check.log').write_text(log)
if (result.returncode or not re.search(r'whisper_backend_init_gpu: using CUDA\d+ backend',log)
    or re.search(r'whisper_backend_init_gpu: (?:failed to initialize|no GPU found)',log)):
 raise SystemExit('CUDA inference verification failed. See ~/.local/share/orinbot/stt/cuda-check.log')
target=Path.home()/'.local/share/orinbot/stt/cuda.json'
temporary=target.with_suffix('.json.tmp')
temporary.write_text(json.dumps({'cuda_root':str(root),'library_path':str(library),'backend':'CUDA','device':0},indent=2)+'\n')
temporary.replace(target)
print('CUDA STT ready; device 0, Orin sm_87. Restart the guide GUI to apply.')
PY
