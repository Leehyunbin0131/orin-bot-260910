"""Coordinate, room intent, persistence, and speech input checks without motor access."""
import io,json,math,sys,tempfile,unittest,wave
from pathlib import Path
import numpy as np
from PIL import Image
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from guide_core import SavedMap,resolve_room,korean_number
from guide_stt import LocalSTT


class GuideTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.d=Path(self.temp.name)
        pixels=np.full((200,200),254,dtype='uint8');pixels[:4]=0;pixels[-4:]=0;pixels[:,:4]=0;pixels[:,-4:]=0;pixels[70:80,70:80]=205
        Image.fromarray(pixels).save(self.d/'map.pgm')
        (self.d/'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm','resolution':.03,'origin':[-3.,-3.,0.],'free_thresh':.196,'occupied_thresh':.65,'negate':0}))
        self.m=SavedMap(self.d/'map.yaml')
    def tearDown(self):self.temp.cleanup()
    def test_pixel_world_roundtrip_and_y_flip(self):
        self.assertEqual(self.m.pixel_to_world(100,100),(0,0))
        self.assertGreater(self.m.pixel_to_world(100,90)[1],0)
        self.m.origin[2]=math.pi/2
        for px,py in [(20,80),(150,50)]:
            x,y=self.m.pixel_to_world(px,py);a,b=self.m.world_to_pixel(x,y)
            self.assertAlmostEqual(a,px);self.assertAlmostEqual(b,py)
    def test_wall_unknown_outside_and_nan_rejected(self):
        for p in [(2.9,0,0),(-.75,.75,0),(10,10,0),(math.nan,0,0)]:
            with self.assertRaises(ValueError):self.m.validate_pose(*p)
    def test_room_persistence_alias_collision_and_backup(self):
        self.m.save_room('101호',0,0,0,['상담실'])
        self.assertEqual(SavedMap(self.d/'map.yaml').rooms[0]['name'],'101호')
        with self.assertRaises(ValueError):self.m.save_room('101호실',.3,0,0,[])
        with self.assertRaises(ValueError):self.m.save_room('102호',.3,0,0,['상담실'])
        self.m.save_room('101호',.2,0,0,['상담실']);self.assertEqual(len(list(self.d.glob('rooms.*.bak.json'))),1)
    def test_changed_map_rejects_old_room_coordinates(self):
        self.m.save_room('101호',0,0,0,[])
        (self.d/'map.pgm').write_bytes((self.d/'map.pgm').read_bytes()+b' ')
        with self.assertRaises(ValueError):SavedMap(self.d/'map.yaml')
    def test_korean_room_resolution_and_no_substring_guessing(self):
        rooms=[{'name':'101호','aliases':['상담실']},{'name':'1101호','aliases':[]},{'name':'102호','aliases':[]}]
        for text in ['101호로 가줘','백일호실로 안내해 줘','일공일호로 가줘','상담실로 가줘']:
            self.assertEqual(resolve_room(text,rooms)['room']['name'],'101호')
        self.assertEqual(resolve_room('1101호로 가줘',rooms)['room']['name'],'1101호')
        for text in ['999호로 가줘','101호 또는 102호','안녕하세요']:
            with self.assertRaises(ValueError):resolve_room(text,rooms)
        self.assertEqual(resolve_room('101호로 가지마',rooms)['intent'],'stop')
        self.assertEqual(korean_number('천백일'),1101)
    def test_silence_rejected_before_model_inference(self):
        stream=io.BytesIO()
        with wave.open(stream,'wb') as f:f.setnchannels(1);f.setsampwidth(2);f.setframerate(16000);f.writeframes(bytes(32000))
        stt=LocalSTT();stt.ready=lambda:True
        with self.assertRaisesRegex(ValueError,'목소리'):stt.transcribe(stream.getvalue())


class VoiceCancellationTests(unittest.TestCase):
    def test_stale_and_cancelled_inflight_voice_never_dispatches_goal(self):
        import threading,urllib.request,urllib.error
        from types import SimpleNamespace
        from guide_service import start_http
        called=[]
        node=SimpleNamespace(generation=3,drive=True,last_voice='',closing=threading.Event(),lock=threading.RLock())
        node.command_text=lambda text:called.append(text) or {'intent':'navigate'}
        def transcribe(audio):node.generation+=1;return '101호로 가줘'
        node.stt=SimpleNamespace(transcribe=transcribe)
        server=start_http(node,0)
        try:
            for epoch in (2,3):
                request=urllib.request.Request(f'http://127.0.0.1:{server.server_port}/api/speech',b'audio',method='POST',headers={'X-Orinbot-UI':'1','X-Guide-Epoch':str(epoch)})
                with self.assertRaises(urllib.error.HTTPError) as error:urllib.request.urlopen(request,timeout=3)
                self.assertEqual(error.exception.code,400)
            self.assertEqual(called,[])
        finally:server.shutdown();server.server_close()

if __name__=='__main__':unittest.main()
