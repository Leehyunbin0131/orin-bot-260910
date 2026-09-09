"""Map export format and service guards, without sensors or motor access."""
from pathlib import Path
import struct
import sys
import tempfile
import threading
import unittest
import zlib
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mapping_service import map_png, verify_saved_map, MappingService
from std_srvs.srv import Trigger


class MappingTests(unittest.TestCase):
    def test_png_pixels_and_vertical_orientation(self):
        grid=SimpleNamespace(info=SimpleNamespace(width=2,height=2),data=[0,100,-1,50])
        png=map_png(grid)
        self.assertEqual(png[:8],b'\x89PNG\r\n\x1a\n')
        pos=8; chunks={}
        while pos<len(png):
            size=struct.unpack('!I',png[pos:pos+4])[0];kind=png[pos+4:pos+8];payload=png[pos+8:pos+8+size]
            self.assertEqual(struct.unpack('!I',png[pos+8+size:pos+12+size])[0],zlib.crc32(kind+payload)&0xffffffff)
            chunks[kind]=payload;pos+=size+12
        self.assertEqual(zlib.decompress(chunks[b'IDAT']),bytes([0,205,127,0,254,0]))

    def test_export_requires_raster_and_graph(self):
        with tempfile.TemporaryDirectory() as temp:
            d=Path(temp)
            with self.assertRaises(RuntimeError):verify_saved_map(d)
            (d/'map.yaml').write_text('image: map.pgm\nresolution: 0.03\n')
            (d/'map.posegraph').write_bytes(b'graph');(d/'map.data').write_bytes(b'data')
            with self.assertRaises(RuntimeError):verify_saved_map(d)
            (d/'map.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
            self.assertEqual(len(verify_saved_map(d)),4)

    def test_save_before_start_rejected(self):
        node=SimpleNamespace(operation=threading.Lock(),alive=lambda:False)
        response=MappingService.save(node,None,Trigger.Response())
        self.assertFalse(response.success);self.assertIn('Start mapping',response.message)
        self.assertFalse(node.operation.locked())

    def test_concurrent_operation_rejected(self):
        node=SimpleNamespace(operation=threading.Lock());node.operation.acquire()
        for callback in [MappingService.start,MappingService.save]:
            self.assertFalse(callback(node,None,Trigger.Response()).success)

    def test_repeated_start_preserves_map(self):
        node=SimpleNamespace(operation=threading.Lock(),closing=threading.Event(),alive=lambda:True)
        response=MappingService.start(node,None,Trigger.Response())
        self.assertFalse(response.success);self.assertIn('preserved',response.message)


if __name__=='__main__':unittest.main()
