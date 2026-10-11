import math,random,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'Scripts'))
from simulate_parameter_identification import line_fit,resistance,inductance,flux
class IdentificationTests(unittest.TestCase):
 def test_constant_offset_cancels(self):
  slope,offset=line_fit([.5,.75,1],[3.75*x-.37 for x in [.5,.75,1]])
  self.assertAlmostEqual(slope,3.75);self.assertAlmostEqual(offset,-.37)
 def test_gain_error_is_not_removed(self):
  slope,_=line_fit([1.05*x+.04 for x in [.5,.75,1]],[3.75*x+.3 for x in [.5,.75,1]])
  self.assertAlmostEqual(slope,3.75/1.05)
 def test_unexcited_rejected(self):
  with self.assertRaises(ValueError):line_fit([1,1],[2,3])
 def test_voltage_command_bias(self):
  actual=inductance(random.Random(42),1000,True)
  command=inductance(random.Random(42),1000,False)
  self.assertAlmostEqual(actual,.0013,delta=.00001)
  self.assertAlmostEqual(command/actual,1.2,delta=.005)
 def test_delay_matches_complex_impedance(self):
  f=1000;dt=.00002;w=2*math.pi*f
  expected=(3.75*math.sin(w*dt)+w*.0013*math.cos(w*dt))/w
  measured=inductance(random.Random(42),f,True,dt)
  self.assertAlmostEqual(measured,expected,delta=.00001)
 def test_command_speed_bias(self):
  a,_=flux(random.Random(42),commanded=False)
  b,_=flux(random.Random(42),commanded=True)
  self.assertAlmostEqual(b/a,.85)
if __name__=='__main__':unittest.main()
