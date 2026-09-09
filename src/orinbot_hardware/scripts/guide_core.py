"""Map coordinates, persistent room labels, and deterministic Korean destination matching."""
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import unicodedata
from datetime import datetime
import numpy as np
from PIL import Image
import yaml


def atomic_json(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name+'.tmp')
    with tmp.open('w') as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False); f.flush(); os.fsync(f.fileno())
    os.replace(tmp, path)


def normalized(text):
    return re.sub(r'[^0-9a-z가-힣]', '', unicodedata.normalize('NFKC', str(text)).lower())


def korean_number(text):
    digits = dict(zip('영공일이삼사오육칠팔구', [0,0,1,2,3,4,5,6,7,8,9]))
    units = {'십':10,'백':100,'천':1000}
    if not text or any(c not in digits and c not in units for c in text): return None
    if not any(c in units for c in text): return int(''.join(str(digits[c]) for c in text))
    result, current, previous = 0, 0, 10000
    for c in text:
        if c in digits: current = digits[c]
        else:
            if units[c] >= previous: return None
            result += (current or 1)*units[c]; current=0; previous=units[c]
    return result+current


def room_keys(room):
    return {normalized(v) for v in [room['name'], *room.get('aliases', [])] if normalized(v)}


def resolve_room(text, rooms):
    phrase = normalized(text)
    if any(v in phrase for v in ['취소','정지','멈춰','멈추','가지마']): return {'intent':'stop','text':text}
    # Convert Sino-Korean and spoken individual digits only when followed by a room suffix.
    def number(m):
        value = korean_number(m[1]); return str(value)+'호' if value is not None else m[0]
    phrase = re.sub(r'([영공일이삼사오육칠팔구십백천]+)호', number, phrase)
    matches=[]
    for room in rooms:
        for key in room_keys(room):
            key = re.sub(r'(호실|호)$','',key)
            if key.isdigit(): found = re.search(r'(?<!\d)'+re.escape(key)+r'(?!\d)',phrase)
            else: found = len(key)>=2 and key in phrase
            if found: matches.append(room); break
    if len(matches)!=1:
        raise ValueError('호실을 찾지 못했습니다.' if not matches else '여러 호실이 들렸습니다. 목적지 하나만 말씀해 주세요.')
    return {'intent':'navigate','text':text,'room':matches[0]}


class SavedMap:
    def __init__(self, path):
        self.path=Path(path).resolve(); config=yaml.safe_load(self.path.read_text())
        if config.get('mode','trinary')!='trinary': raise ValueError('trinary 지도 형식만 지원합니다.')
        image=Path(config['image']); self.image_path=image if image.is_absolute() else self.path.parent/image
        self.resolution=float(config['resolution']);self.origin=[float(v) for v in config['origin']]
        if len(self.origin)!=3 or not all(math.isfinite(v) for v in [self.resolution,*self.origin]) or self.resolution<=0:
            raise ValueError('지도 좌표 설정이 올바르지 않습니다.')
        source=Image.open(self.image_path).convert('L'); self.width,self.height=source.size
        if self.width*self.height>25_000_000: raise ValueError('지도 이미지가 너무 큽니다.')
        pixels=np.asarray(source,dtype=float)/255.
        occupancy=pixels if config.get('negate',0) else 1-pixels
        self.free=occupancy<float(config.get('free_thresh',.25))
        occupied=occupancy>float(config.get('occupied_thresh',.65))
        display=np.where(self.free,254,np.where(occupied,0,205)).astype('uint8')
        data=io.BytesIO();Image.fromarray(display).save(data,format='PNG');self.png=data.getvalue()
        self.fingerprint=hashlib.sha256(self.path.read_bytes()+self.image_path.read_bytes()).hexdigest()
        self.rooms_path=self.path.parent/'rooms.json'
        self.rooms=[]
        if self.rooms_path.exists():
            store=json.loads(self.rooms_path.read_text())
            if store.get('map_fingerprint')!=self.fingerprint: raise ValueError('지도와 호실 데이터가 다릅니다. 기존 rooms.json을 확인하세요.')
            self.rooms=store['rooms']

    def pixel_to_world(self, px, py):
        x,y=float(px)*self.resolution,(self.height-float(py))*self.resolution
        c,s=math.cos(self.origin[2]),math.sin(self.origin[2])
        return self.origin[0]+c*x-s*y,self.origin[1]+s*x+c*y

    def world_to_pixel(self,x,y):
        dx,dy=x-self.origin[0],y-self.origin[1];c,s=math.cos(self.origin[2]),math.sin(self.origin[2])
        return (c*dx+s*dy)/self.resolution,self.height-(-s*dx+c*dy)/self.resolution

    def validate_pose(self,x,y,yaw,clearance=.46):
        if not all(math.isfinite(float(v)) for v in (x,y,yaw)): raise ValueError('좌표와 방향은 유한한 숫자여야 합니다.')
        px,py=self.world_to_pixel(float(x),float(y));radius=clearance/self.resolution
        if px-radius<0 or py-radius<0 or px+radius>=self.width or py+radius>=self.height:
            raise ValueError('지도 가장자리와 너무 가깝습니다.')
        x0,x1=int(px-radius),math.ceil(px+radius);y0,y1=int(py-radius),math.ceil(py+radius)
        yy,xx=np.mgrid[y0:y1+1,x0:x1+1];mask=(xx+.5-px)**2+(yy+.5-py)**2<=radius**2
        if np.any(~self.free[y0:y1+1,x0:x1+1][mask]):
            raise ValueError('벽·미확인 영역과 너무 가깝습니다. 복도 안쪽의 빈 공간을 선택하세요.')

    def save_room(self,name,x,y,yaw,aliases):
        name=str(name).strip()
        if not 1<=len(name)<=24 or not normalized(name): raise ValueError('호실 이름을 1~24자로 입력하세요.')
        aliases=[str(v).strip() for v in aliases if str(v).strip()]
        if len(aliases)>8 or any(len(v)>24 for v in aliases): raise ValueError('별칭은 24자 이내, 최대 8개입니다.')
        self.validate_pose(x,y,yaw)
        room={'name':name,'x':float(x),'y':float(y),'yaw':math.atan2(math.sin(yaw),math.cos(yaw)),'aliases':aliases}
        canonical=lambda k:re.sub(r'(호실|호)$','',k)
        newkeys={canonical(k) for k in room_keys(room)}
        for old in self.rooms:
            if old['name']!=name and newkeys & {canonical(k) for k in room_keys(old)}: raise ValueError('다른 호실과 이름·별칭이 겹칩니다.')
        updated=[old for old in self.rooms if old['name']!=name]+[room]
        self.persist(updated);return room

    def persist(self,rooms):
        if self.rooms_path.exists():
            backup=self.rooms_path.with_name('rooms.'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.bak.json')
            backup.write_bytes(self.rooms_path.read_bytes())
        atomic_json(self.rooms_path,{'map_fingerprint':self.fingerprint,'rooms':rooms});self.rooms=rooms

    def metadata(self):
        return {'width':self.width,'height':self.height,'resolution':self.resolution,'origin':self.origin,
                'fingerprint':self.fingerprint,'rooms':self.rooms,'path':str(self.path)}
