/* Same-origin JPEG preview; never queue frames or keep a stale picture visible. */
(() => {
  const panel = document.getElementById('cameraPreview');
  if (!panel) return;
  panel.innerHTML = '<h3>로봇 카메라</h3><button type="button" id="cameraToggle">영상 보기</button>' +
    '<img id="cameraImage" alt="로봇 전방 카메라 영상" hidden style="width:100%;height:auto;border-radius:8px;margin-top:10px">' +
    '<p id="cameraStatus" class="muted" role="status">영상 보기를 누르면 로봇 카메라 화면이 표시됩니다.</p>';
  const button = document.getElementById('cameraToggle');
  const image = document.getElementById('cameraImage');
  const status = document.getElementById('cameraStatus');
  let enabled = false, generation = 0, timer = null, request = null, objectURL = null;
  function clearImage() {
    image.hidden = true; image.removeAttribute('src');
    if (objectURL) URL.revokeObjectURL(objectURL);
    objectURL = null;
  }
  function cancel() {
    generation++; clearTimeout(timer);
    if (request) request.abort();
    request = null; clearImage();
  }
  async function update(token) {
    if (!enabled || document.hidden || token !== generation) return;
    const controller = new AbortController(); request = controller;
    const timeout = setTimeout(() => controller.abort(), 1500);
    let retry = 200;
    try {
      const response = await fetch('/api/camera.jpg', {cache: 'no-store', signal: controller.signal});
      if (!response.ok) {
        const error = await response.json();
        throw Error(error.message || '카메라 영상을 받을 수 없습니다.');
      }
      const blob = await response.blob();
      if (token !== generation) return;
      const previous = objectURL; objectURL = URL.createObjectURL(blob);
      image.src = objectURL;
      if (previous) URL.revokeObjectURL(previous);
      await image.decode();
      if (token !== generation) return;
      image.hidden = false; status.textContent = '로봇 카메라 · 실시간 보기 (최대 5fps)';
    } catch (error) {
      if (token !== generation) return;
      clearImage(); retry = 1000;
      status.textContent = error.name === 'AbortError' ? '영상 연결이 지연되고 있습니다. 재연결 중…' : error.message;
    } finally {
      clearTimeout(timeout);
      if (request === controller) request = null;
      if (enabled && !document.hidden && token === generation) timer = setTimeout(() => update(token), retry);
    }
  }
  button.onclick = () => {
    enabled = !enabled; cancel();
    button.textContent = enabled ? '영상 숨기기' : '영상 보기';
    status.textContent = enabled ? '카메라 연결 중…' : '영상 표시를 중지했습니다.';
    if (enabled) update(generation);
  };
  document.addEventListener('visibilitychange', () => {
    cancel();
    if (enabled && !document.hidden) update(generation);
  });
  window.addEventListener('pagehide', cancel);
})();
