import importlib.util,json,math,tempfile,unittest
from pathlib import Path
spec=importlib.util.spec_from_file_location('entry',Path('Scripts/analyze_sensorless_entry.py'))
entry=importlib.util.module_from_spec(spec);spec.loader.exec_module(entry)

class EntryPhaseTest(unittest.TestCase):
 def test_interpolation_removes_known_sample_delay_across_wrap(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'test.log';lines=['time_us,flags,pll,enc,age']
   omega=600*2*math.pi/60*7
   for t,s in [(1000,0),(2000,0),(3000,2000),(4000,2000),(5000,4000)]:
    lines.append(f'{1000000+t},274,{entry.wrap(3+omega*t/1e6+.3)},{entry.wrap(3+omega*s/1e6)},{(t-s)/1e6}')
   p.write_text('\n'.join(lines),encoding='utf8');out=entry.analyze(p)
   self.assertEqual(out['blend']['n'],3)
   self.assertAlmostEqual(out['blend']['aligned_error']['median'],.3,places=7)
   self.assertGreater(out['blend']['raw_error']['median'],.7)
 def test_missing_entry_is_not_reported_as_takeover(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'test.log';p.write_text('time_us,flags,pll,enc,age\n1000000,16,0,0,0\n',encoding='utf8')
   self.assertIsNone(entry.analyze(p)['entry_us'])

if __name__=='__main__':unittest.main()
