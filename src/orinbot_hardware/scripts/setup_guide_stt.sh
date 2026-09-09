#!/usr/bin/env bash
set -euo pipefail
stt_source="$HOME/.local/opt/whisper.cpp-v1.8.4"
mkdir -p "$HOME/.local/opt" "$HOME/.local/share/orinbot/stt"
if [[ ! -d "$stt_source" ]]; then
  git clone --depth 1 --branch v1.8.4 https://github.com/ggml-org/whisper.cpp.git "$stt_source"
fi
cmake -S "$stt_source" -B "$stt_source/build" -DCMAKE_BUILD_TYPE=Release -DGGML_CUDA=OFF -DWHISPER_BUILD_TESTS=OFF
cmake --build "$stt_source/build" --target whisper-cli -j 2
python3 - <<'PY'
from pathlib import Path
import urllib.request,os
p=Path.home()/'.local/share/orinbot/stt/ggml-small-q5_1.bin'
if not p.is_file():
 urllib.request.urlretrieve('https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-small-q5_1.bin',str(p)+'.part')
 os.replace(str(p)+'.part',p)
print('Local STT ready:',p)
PY
