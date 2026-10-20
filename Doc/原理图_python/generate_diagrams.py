
"""Render source-grounded motor-control diagrams; no firmware writes."""
from pathlib import Path
import json, hashlib, re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
from PIL import Image, ImageOps, ImageDraw
ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
FIG=OUT/"images"
FIG.mkdir(exist_ok=True)
plt.rcParams.update({"font.sans-serif":["Microsoft YaHei","SimHei","DejaVu Sans"],
                     "axes.unicode_minus":False,"svg.fonttype":"none"})
SPECS=json.loads(r'''[["01_main","\u4e2d\u65ad\u5206\u6d3e\u4e0e\u6b63\u5e38 FOC \u63a7\u5236\u62cd",["ADC-DMA \u56de\u8c03\nMotor_DmaInt","\u6309\u4f18\u5148\u7ea7\u5206\u6d3e\n\u68c0\u6d4b \u2192 BLDC \u2192 \u5feb\u901f V/F \u2192 FOC","\u63a7\u5236\u62cd\u7b5b\u9009\n\u6709\u611f\uff1aPWM \u4e0b\u6570\u4fa7\n\u5176\u4ed6\uff1aFOC \u5206\u9891","Sense \u6362\u7b97 / \u4f20\u611f\u5668\u66f4\u65b0\n\u539f\u59cb\u7535\u6d41\u4fdd\u62a4","LONGEST_ZERO \u7535\u6d41\u91cd\u6784\n\u542f\u52a8/\u53cd\u9988\u9009\u62e9 \u2192 \u7535\u6d41\u63a7\u5236","\u6700\u7ec8\u6545\u969c\u4ef2\u88c1\n\u9006 Park / SVM / A-C-B PWM"],"App/motor/motor_it.c|App/motor/core/motor_foc_isr.c|App/motor/core/motor_foc_divider.c","\u901f\u5ea6/\u4f4d\u7f6e/\u5173\u8282\u5916\u73af\u5728 foc pid \u4efb\u52a1\uff0c\u7535\u6d41\u73af\u5728 ISR\uff1b\u7b5b\u9009\u53d1\u751f\u5728\u6b63\u5e38\u8def\u5f84\u91c7\u6837\u4e0e\u8f6f\u4ef6\u4fdd\u62a4\u4e4b\u524d\u3002"],["02_sampling","ADC \u7269\u7406\u91cf\u3001\u677f\u7ea7\u76f8\u5e8f\u4e0e\u7535\u6d41\u91cd\u6784",["HAL \u540c\u5468\u671f\u91c7\u6837\u5e27\nADC / \u76f8\u7535\u538b / Vbus","\u7535\u6d41\uff1aI=(ADC-offset)\u00d7scale\nA\u2190CURR1 B\u2190CURR3 C\u2190CURR2","\u6bcd\u7ebf\u4f4e\u901a \u03b1=0.1\n\u76f8\u7535\u538b\u6309 A-C-B \u6362\u7b97","\u4fdd\u62a4\u5148\u4f7f\u7528\u6362\u7b97\u539f\u59cb\u7535\u6d41\n\u9632\u6b62\u91cd\u6784\u63a9\u76d6\u5f02\u5e38","\u552f\u4e00\u6700\u5927 duty \u4e14\u7a97\u53e3\u4e0d\u8db3\nImax=\u2212(\u53e6\u5916\u4e24\u76f8\u7535\u6d41)","Clarke / Park\n\u4e09\u76f8 \u2192 \u9759\u6b62 \u03b1\u03b2 \u2192 \u65cb\u8f6c dq"],"App/motor/core/motor_sense.c|App/motor/core/motor_current.c|Library/Hal/port/stm32f4/hal_motor_stm32f4.c","\u6700\u5927 duty \u5e76\u5217\u65f6\u4e0d\u91cd\u6784\uff1bBM75 \u7ea6 0.0806 A/count\uff0c0.1\uff5e0.2 A \u7684\u7cbe\u786e\u8ddf\u8e2a\u65e0\u6cd5\u7531\u5355\u62cd\u8bfb\u6570\u8bc1\u660e\u3002"],["03_current","d/q \u7535\u6d41 PI \u4e0e\u4e24\u7ea7\u6297\u9971\u548c",["\u76ee\u6807\u9009\u62e9\n\u5bf9\u9f50 Id / I/F Iq / \u5916\u73af Iq","\u8bef\u5dee e=Iref\u2212Imeas\nd/q \u4f7f\u7528\u72ec\u7acb\u79ef\u5206\u72b6\u6001","z\u2190z+Ki\u00d7e\u00d7dt\nvraw=Kp\u00d7e+z","\u5355\u8f74\u9650\u5e45 \u00b1Vlim\nz\u2190vlimited\u2212Kp\u00d7e","\u5408\u6210\u77e2\u91cf\u5706\u9650\u5e45\nvd/vq \u540c\u6bd4\u7f29\u5c0f","\u82e5\u77e2\u91cf\u9971\u548c\u518d\u6b21\u56de\u5199\u79ef\u5206\nVlim=min(\u914d\u7f6e,Vbus/\u221a3)"],"App/motor/core/motor_current.c|App/motor/algo/motor_pid.c|App/motor/core/motor_control.c","\u7535\u6d41\u76ee\u6807\u5355\u4f4d A\u3001\u7535\u538b V\u3001dt \u79d2\u3002\u5bf9\u9f50\u5b9a\u89d2 Id \u5efa\u7acb\u78c1\u573a\uff0c\u6b63\u5e38 Iq \u63d0\u4f9b\u8f6c\u77e9\u3002"],["04_output","\u9006 Park \u6781\u6027\u3001\u8c03\u5236\u9009\u62e9\u4e0e PWM \u63d0\u4ea4",["dq \u8bf7\u6c42 + \u63a7\u5236\u7535\u89d2\u5ea6\nvd / vq / \u03b8e","inverse_park\nv\u03b1=\u2212(vd cos\u03b8\u2212vq sin\u03b8)\nv\u03b2=\u2212(vd sin\u03b8+vq cos\u03b8)","I/F / \u5f3a\u5236\u7ebf\u6027\u8def\u5f84\nfoc_svm_from_voltage\n\u8f93\u5165\u4e3a \u2212v\u03b1, \u2212v\u03b2","\u5176\u4ed6\u8def\u5f84\uff08\u542b\u5feb\u901f V/F\uff09\nsvm_hqd(v\u03b1,v\u03b2)","\u8f6f\u4ef6 ABC \u6bd4\u8f83\u503c\nHAL \u63d0\u4ea4 A-C-B","PWM Commit\nSAFE \u8bf7\u6c42\u63d0\u4ea4 ARR/2"],"App/motor/core/motor_output.c|App/motor/algo/motor_foc.c","\u56fe\u5185\u4e24\u4e2a\u8c03\u5236\u8282\u70b9\u662f\u6761\u4ef6\u5206\u652f\uff1bSAFE \u4e09\u76f8\u4e2d\u70b9\u4e0d\u7b49\u4e8e\u5173\u6865\u3002\u65e0\u611f\u8bb0\u5f55\u5b9e\u9645\u8c03\u5236\u7ea6\u5b9a\u4e0b\u7684\u7535\u538b\u3002"],["05_task","\u4efb\u52a1\u3001\u7f16\u7801\u5668\u91c7\u96c6\u4e0e\u5feb\u73af\u4e4b\u95f4\u7684\u6570\u636e\u6d41",["mt6701 \u72ec\u7acb\u4efb\u52a1\n\u56fa\u5b9a\u91c7\u96c6\u5468\u671f","\u8bfb\u53d6 MT6701\n\u53d1\u5e03\u5e26\u65b0\u9c9c\u5ea6\u7684\u539f\u59cb\u5e27","FOC ISR\n\u9a8c\u6536/\u9884\u6d4b\u89d2\u5ea6\u548c\u901f\u5ea6","foc pid \u4efb\u52a1\n\u6309 speed_loop_hz \u8c03\u5ea6","\u5173\u8282\u542f\u7528\uff1aJoint_Run\n\u5426\u5219 Position_Run\n\u7136\u540e SpeedLoop_Run","\u5916\u73af\u8f93\u51fa Vq \u6216 Iq\n\u63a7\u5236 ISR \u6d88\u8d39\u8f93\u51fa"],"App/motor/motor_task.c|App/motor_sensor/motor_mt6701.c|App/motor/core/motor_control.c","PID dt \u6309\u771f\u5b9e\u7ecf\u8fc7\u65f6\u95f4\u53d6\u5f97\u5e76\u94b3\u4f4d\uff1b\u7f16\u7801\u5668\u603b\u7ebf\u4e8b\u52a1\u4e0d\u8fdb\u5165 ADC ISR\u3002"],["06_vf","\u5f53\u524d\u5feb\u901f V/F \u8bca\u65ad\uff1a\u6052 Vq \u65cb\u8f6c\u78c1\u573a",["DMA \u5206\u6d3e\u547d\u4e2d V/F\n\u5feb\u901f\u8bca\u65ad ISR","\u76ee\u6807 RPM \u4fdd\u7559\u7b26\u53f7\n\u4f4e\u975e\u96f6\u76ee\u6807\u4f7f\u7528 min_rpm \u5730\u677f","\u03b8step=RPM\u00d72\u03c0/60\u00d7p/fctrl\n\u03b8\u2190wrap(\u03b8+\u03b8step)","Vd=0\nVq=clamp(v_boost,0,Vq_limit)","Motor_OutputVoltage\nsvm_hqd \u8c03\u5236","\u65cb\u8f6c\u5b9a\u5b50\u7535\u538b\u77e2\u91cf\n\u673a\u68b0\u5b9e\u9645\u901f\u5ea6\u5e76\u975e\u76ee\u6807\u4fdd\u8bc1"],"App/motor/diagnostic/motor_fast_vf.c|App/motor/diagnostic/motor_fast_vf_isr.c","\u6b64\u5b9e\u9645\u8bca\u65ad\u8def\u5f84\u4e0d\u662f\u7ecf\u5178 V=kf \u66f2\u7ebf\uff1b\u6700\u4f4e\u8f6c\u901f\u5730\u677f\u662f\u6b64\u5386\u53f2\u8def\u5f84\u884c\u4e3a\uff0c\u4e0d\u80fd\u5957\u5230\u6b63\u5e38 I/F\u3002"],["07_start","I/F \u542f\u52a8\uff1a\u7535\u6d41\u5efa\u7acb\u3001\u4fdd\u6301\u4e0e\u62d6\u8f6c",["\u53ef\u9009 HFI \u626b\u63cf\n\u5df2\u77e5\u521d\u59cb\u7535\u89d2","CURRENT_RAMP\uff08\u542f\u7528\u5bf9\u9f50\uff09\n\u5b9a\u89d2 Id \u5e73\u6ed1\u5efa\u7acb","ANGLE_HOLD\uff08\u542f\u7528\u5bf9\u9f50\uff09\n\u8ba1\u65f6\u4ece\u722c\u5347\u5b8c\u6210\u5f00\u59cb","DRAG_RAMP\nId \u4ea4\u7ed9 Iq / Iq \u5e73\u6ed1\u5efa\u7acb","\u673a\u68b0 RPM \u6309\u659c\u5761\u63a8\u8fdb\n\u03b8ol\u2190\u03b8ol+\u03c9e\u00d7dt","\u4f20\u611f\u5668/\u65e0\u611f\u63a5\u7ba1\n\u89d2\u5ea6\u6709\u6548\u6027\u4e0e\u76ee\u6807\u5207\u6362"],"App/motor/core/motor_control.c|App/motor/start/motor_if_start.c|App/motor/start/motor_alignment.c","ol_align_enable=0 \u8df3\u8fc7\u524d\u4e24\u9636\u6bb5\u3002\u6b63\u5e38 I/F \u4e0d\u628a\u771f\u5b9e\u89d2\u901f\u5ea6\u77ac\u95f4\u62ac\u5230 ol_min_rpm\u3002"],["08_encoder","MT6701 \u7f16\u7801\u5668\uff1a\u56de\u7ed5\u3001PLL \u4e0e\u7535\u89d2",["\u7edd\u5bf9\u8ba1\u6570 + \u65b0\u9c9c\u5ea6\ncounts_per_rev","\u6700\u77ed\u8def\u5f84\u8ba1\u6570\u589e\u91cf\n\u62d2\u7edd\u4e0d\u5408\u7406\u8df3\u53d8","PLL \u9884\u6d4b\u4f4d\u7f6e\n\u8bef\u5dee\u6298\u5230 \u00b1CPR/2","\u4e34\u754c\u963b\u5c3c\u6821\u6b63\nKp=2\u00d7BW\uff0cKi=Kp\u00b2/4","\u901f\u5ea6\uff1a60\u00d7vel/CPR\n\u65e0\u65b0\u5e27\u53ea\u9884\u6d4b","\u65b9\u5411/\u96f6\u70b9/\u6781\u5bf9\u6570\n\u673a\u68b0\u89d2 \u2192 \u7535\u89d2\u5ea6"],"App/motor_sensor/motor_encoder.c|App/motor_sensor/motor_encoder_detect.c","\u8fd1\u96f6\u901f\u5ea6\u5438\u9644\u524d\u7684 raw RPM \u5355\u72ec\u4fdd\u7559\uff1b\u7535\u89d2\u96f6\u70b9\u6709\u6548\u6027\u4e0e\u901f\u5ea6\u6709\u6548\u6027\u5206\u5f00\u5904\u7406\u3002"],["09_hall","\u970d\u5c14\u89d2\u5ea6\u4e0e\u901f\u5ea6\u53cd\u9988",["3 \u4f4d\u970d\u5c14\u72b6\u6001\n\u67e5\u5df2\u6807\u5b9a\u516d\u6247\u533a\u8868","\u975e\u6cd5\u72b6\u6001 / \u8868\u5931\u6548\u68c0\u67e5\n\u5224\u65ad\u76f8\u90bb\u6247\u533a\u548c\u65b9\u5411","\u6247\u533a\u4e2d\u5fc3\u89d2\n\u968f\u901f\u5ea6\u589e\u52a0\u542f\u7528\u89d2\u5ea6\u63d2\u503c","\u4f4e\u901f\uff1a\u4e34\u754c\u963b\u5c3c PLL\n\u9ad8\u901f\uff1a\u8fb9\u6cbf\u5468\u671f\u4f30\u901f","RPM=\u65b9\u5411\u00d710/(p\u00d7Tsector)\n\u6216\u7d2f\u8ba1\u591a\u6247\u533a\u5468\u671f","\u8d85\u65f6 / \u505c\u8f6c / \u6362\u5411\u5904\u7406\n\u53d1\u5e03\u89d2\u5ea6\u3001RPM\u3001valid"],"App/motor_sensor/motor_hall_angle.c|App/motor_sensor/motor_hall.c","\u8868\u503c raw/200\u00d72\u03c0 \u5f97\u7535\u89d2\uff1b\u970d\u5c14\u7684\u89d2\u5ea6\u91cf\u5316\u4e0e\u4f4e\u901f\u8fb9\u6cbf\u7a00\u758f\u51b3\u5b9a\u53cd\u9988\u7cbe\u5ea6\u3002"],["10_speed","\u901f\u5ea6 PI\u3001\u5236\u52a8\u8bed\u4e49\u4e0e\u9971\u548c\u6cc4\u653e",["\u6709\u6548\u53cd\u9988 + I/F + \u4f7f\u80fd\nISR \u5141\u8bb8\u4efb\u52a1\u8fd0\u884c\u901f\u5ea6 PI","\u4efb\u52a1\u7d2f\u8ba1 dt \u5230\u76ee\u6807\u5468\u671f\n\u76ee\u6807\u53d6\u659c\u5761\u540e RPM","e=RPMref\u2212RPMmeas\nP=Kp\u00d7e","\u79ef\u5206\u5019\u9009 z+Ki\u00d7e\u00d7dt\n\u8f93\u51fa\u9650\u5e45 \u00b1Iq_limit","\u540c\u5411\u9876\u6ee1\u65f6\u4e0d\u7d2f\u52a0\n\u5386\u53f2\u79ef\u5206\u6309 0.100 s \u6cc4\u653e","Iq \u4ea4\u7ed9\u63a7\u5236 ISR\n\u4f4e\u901f\u53ef\u9009\u7535\u538b\u540e\u7aef"],"App/motor/loop/motor_speed.c|App/motor/core/motor_control.c","\u7981\u6b62\u5236\u52a8\u65f6\u96f6\u901f\u547d\u4ee4\u81ea\u7136\u6ed1\u505c\uff1b\u5141\u8bb8\u5236\u52a8\u65f6\u68c0\u6d4b\u8fc7\u96f6\u540e\u91ca\u653e\uff1b\u4f4d\u7f6e\u4fdd\u6301\u5177\u6709\u72ec\u7acb\u8bed\u4e49\u3002"],["11_position","\u591a\u5708\u4f4d\u7f6e\u63a7\u5236\uff1a\u5927\u8bef\u5dee\u79fb\u52a8\u4e0e\u7535\u538b\u4fdd\u6301",["\u7d2f\u8ba1\u7f16\u7801\u5668\u673a\u68b0\u8ba1\u6570\n\u76f8\u5bf9\u96f6\u70b9\u591a\u5708\u89d2\u5ea6\uff08deg\uff09","\u4f4d\u7f6e\u8bef\u5dee\nRPMraw=Kp_pos\u00d7error","\u901f\u5ea6/\u52a0\u901f\u5ea6\u9650\u5e45\n\u751f\u6210\u5e73\u6ed1 RPMref","\u5927\u8bef\u5dee\u53ef\u8d70\u901f\u5ea6\u7535\u6d41\u73af\n\u8fd1\u76ee\u6807\u5207\u5230\u7535\u538b\u4fdd\u6301","\u4fdd\u6301\u540e\u7aef\uff1a\u901f\u5ea6 PI + V/RPM \u524d\u9988\n\u6761\u4ef6\u79ef\u5206 / \u7535\u538b\u9650\u5e45","\u8f93\u51fa Vq / \u03b8encoder\n\u53cd\u9988\u5931\u6548\u8fd4\u56de SAFE"],"App/motor/loop/motor_position.c|App/motor/loop/motor_position_control.c","\u7eaf\u4f4d\u7f6e\u63a7\u5236\u6a21\u5757\u8f93\u51fa\u7535\u538b\uff1b\u5e94\u7528\u5c42\u6839\u636e\u8bef\u5dee\u6ede\u56de\u9009\u62e9\u79fb\u52a8/\u4fdd\u6301\u3002\u4e0d\u80fd\u7b80\u5316\u6210\u6240\u6709\u5de5\u51b5\u90fd\u4e09\u73af\u4e32\u8054\u3002"],["12_joint","\u5173\u8282\u963b\u6297\u4e0e\u4f4e\u7535\u6d41\u6df7\u5408\u540e\u7aef",["\u4f4d\u7f6e/\u901f\u5ea6/\u524d\u9988\u547d\u4ee4\n\u5355\u4f4d rad\u3001rad/s","u=Kp\u00d7\u4f4d\u7f6e\u8bef\u5dee\n+Kd\u00d7\u901f\u5ea6\u8bef\u5dee+\u524d\u9988","\u5c0f\u8bef\u5dee/\u4f4e\u901f\u6761\u4ef6\u79ef\u5206\n\u9971\u548c\u540c\u5411\u65f6\u51bb\u7ed3\u79ef\u5206","\u5e94\u7528\u5c42\u540e\u7aef\u9009\u62e9\n\u7535\u538b / \u7535\u6d41 / \u6df7\u5408","\u6df7\u5408\u6ede\u56de\uff1a|Iq|\u22650.25 A \u8fdb\u5165 PI\n|Iq|\u22640.20 A \u9000\u51fa\u5230 Vq","\u7f16\u7801\u5668\u7535\u89d2\u6267\u884c\n\u8d85\u65f6 / \u65e0\u6548\u53cd\u9988\u505c\u673a"],"App/motor/joint/motor_joint_control.c|App/motor/joint/motor_joint.c","\u7eaf\u7b97\u6cd5 output \u5355\u4f4d\u7531\u914d\u7f6e\u51b3\u5b9a\uff1b\u5e94\u7528\u5c42\u6620\u5c04\u4e3a V \u6216 A\u3002\u5207\u6362\u662f\u547d\u4ee4\u9700\u6c42\u9608\u503c\uff0c\u4e0d\u80fd\u7b49\u540c\u5b9e\u6d4b\u7535\u6d41\u7cbe\u5ea6\u3002"],["13_trajectory","\u68af\u5f62\u8f68\u8ff9\uff1a\u6309\u5269\u4f59\u5236\u52a8\u8ddd\u79bb\u63d0\u524d\u51cf\u901f",["\u5f53\u524d\u4f4d\u7f6e/\u901f\u5ea6\n\u76ee\u6807\u4f4d\u7f6e\u4e0e v/a \u9650\u5236","\u5269\u4f59\u8bef\u5dee \u0394x\n\u786e\u5b9a\u8fd0\u52a8\u65b9\u5411","\u53ef\u505c\u901f\u5ea6\nvbrake=\u221a(2\u00d7adec\u00d7|\u0394x|)","vdes=\u65b9\u5411\u00d7min(vmax,vbrake)\n\u6309\u52a0\u901f/\u51cf\u901f\u9650\u5236\u6539\u901f\u5ea6","\u4f4d\u7f6e\u68af\u5f62\u79ef\u5206\nxnext=x+(vold+vnew)dt/2","\u8de8\u76ee\u6807/\u5230\u4f4d\u5224\u5b9a\n\u53d1\u5e03\u4f4d\u7f6e\u3001\u901f\u5ea6\u3001\u52a0\u901f\u5ea6"],"App/motor/loop/motor_trajectory.c","\u8f68\u8ff9\u751f\u6210\u5668\u7ed9\u5916\u73af\u53c2\u8003\uff0c\u4e0d\u76f4\u63a5\u5199 PWM\uff1b\u53cd\u5411\u8fd0\u52a8\u5148\u6309\u51cf\u901f\u89c4\u5219\u5904\u7406\u3002"],["14_flux","Ortega \u78c1\u94fe\u89c2\u6d4b\u4e0e\u4e3b\u8def\u5f84 PLL",["\u4e0a\u4e00\u62cd\u5b9e\u9645 \u03b1\u03b2 \u7535\u538b\n\u5f53\u524d \u03b1\u03b2 \u7535\u6d41 + R/L/\u03c8f","\u8f6c\u5b50\u78c1\u94fe \u03c8r=x\u2212L\u00d7i\n\u5e45\u503c\u8bef\u5dee \u03b5=\u03c8f\u00b2\u2212|\u03c8r|\u00b2","x\u70b9=v\u2212R\u00d7i+(\u03b3/2)\u03c8r \u03b5\n\u6b27\u62c9\u79ef\u5206 x\u2190x+x\u70b9dt","atan2(\u03c8\u03b2,\u03c8\u03b1)\n\u52a0\u76f8\u4f4d\u504f\u7f6e \u2192 \u539f\u59cb\u89d2","PLL \u8ddf\u8e2a\u89d2\u548c \u03c9e\nRPM=\u03c9e\u00d760/(2\u03c0p)","\u9501\u5b9a\u5224\u5b9a / \u65f6\u95f4\u53bb\u6296\n\u63a7\u5236\u89d2\u5e73\u6ed1\u63a5\u7ba1"],"App/motor/observer/motor_sensorless.c|App/motor/observer/motor_flux.c","\u91cd\u8981\u5dee\u5f02\uff1a\u4e3b sensorless \u5b9e\u73b0\u5c06 \u03b5>0 \u622a\u4e3a 0\uff1b\u72ec\u7acb motor_flux \u4fdd\u7559\u53cc\u5411\u8bef\u5dee\u3002"],["15_lbg","\u72ec\u7acb\u9f99\u4f2f\u683c\u53cd\u7535\u52bf\u89c2\u6d4b\u5668",["\u8f93\u5165 v\u03b1\u03b2\u3001i\u03b1\u03b2\u3001R\u3001L\nSeed \u521d\u59cb\u5316 \u00ee \u548c \u00ea","\u7535\u6d41\u6b8b\u5dee \u03b5=i\u2212\u00ee","\u00ee\u70b9=(v\u2212R\u00ee\u2212\u00ea)/L+l1 \u03b5\n\u6b27\u62c9\u66f4\u65b0\u4f30\u8ba1\u7535\u6d41","\u00ea\u70b9=\u03c9\u4f30\u00d7J\u00ea\u2212l2 \u03b5\n\u5305\u542b\u53cd\u7535\u52bf\u65cb\u8f6c\u9884\u6d4b","\u4ece \u00ea \u53d1\u5e03\u89d2\u5ea6/\u6a21\u957f/\u901f\u5ea6\n\u5408\u6cd5\u6027\u68c0\u67e5","\u6a21\u5757\u8f93\u51fa\n\u542f\u7528\u4e0e\u63a5\u5165\u9700\u6838\u5bf9\u8c03\u7528\u65b9"],"App/motor/observer/motor_luenberger.c","\u72ec\u7acb\u7b97\u6cd5\u6a21\u5757\u4e0d\u7b49\u4e8e\u9ed8\u8ba4\u4e3b\u63a7\u5236\u5165\u53e3\uff1b\u56fe\u793a\u4e3a\u6b64\u6587\u4ef6\u7684\u5177\u4f53\u5b9e\u73b0\u3002"],["16_smo","\u72ec\u7acb SMO\uff1a\u6b8b\u5dee\u6ce8\u5165\u5230\u53cd\u7535\u52bf\u9884\u6d4b",["\u8f93\u5165\u7535\u538b\u3001\u7535\u6d41\nR/L\u3001slide_gain\u3001lpf","s=\u00ee\u2212i\nz=k\u00d7Sigmoid(s,eps)","\u00ee\u2190\u00ee+dt\u00d7(v\u2212R\u00ee\u2212\u00ea)/L\n\u7535\u6d41\u65b9\u7a0b\u4ec5\u51cf \u00ea","\u00ea\u2190Rot(\u03c9\u4f30dt)\u00ea+lpf\u00d7z\n\u9884\u6d4b\u540e\u52a0\u6b8b\u5dee\u6821\u6b63","\u4ece \u00ea \u8ba1\u7b97\u89d2\u5ea6/\u901f\u5ea6\n\u6709\u9650\u503c\u548c\u6a21\u957f\u68c0\u67e5","\u8f93\u51fa\u89d2 / \u901f\u5ea6 / \u6a21\u957f\n\u4f9b\u8c03\u7528\u65b9\u9009\u7528"],"App/motor/observer/motor_smo.c","\u5f53\u524d\u4ee3\u7801\u6ca1\u6709\u5728\u7535\u6d41\u65b9\u7a0b\u518d\u51cf\u6ee1\u5e45 z\uff0c\u4e0d\u80fd\u5957\u7528\u5e38\u89c1 textbook SMO \u6846\u56fe\u3002"],["17_ekf","\u72ec\u7acb EKF\uff1a\u56db\u72b6\u6001\u9884\u6d4b\u4e0e\u7535\u6d41\u89c2\u6d4b",["x=[i\u03b1,i\u03b2,\u03c9e,\u03b8e]\n\u91cf\u6d4b y=[i\u03b1,i\u03b2]","PMSM \u6a21\u578b\u9884\u6d4b\n\u03b8\u70b9=\u03c9e\uff0c\u03c9\u70b9=0","F\u2248I+dt\u00d7\u6a21\u578b Jacobian\nPpred=FPF\u1d40+Q","S=HPpredH\u1d40+R\nK=PpredH\u1d40S\u207b\u00b9","\u72b6\u6001\u4fee\u6b63 x\u2190x+K(y\u2212h(x))\n\u03b8 wrap","Joseph \u534f\u65b9\u5dee\u66f4\u65b0\n\u8f93\u51fa\u89d2\u548c\u901f\u5ea6"],"App/motor/observer/motor_ekf.c","\u4ee3\u7801\u6bcf\u62cd\u76f4\u63a5\u52a0\u914d\u7f6e Q\uff0c\u56fe\u672a\u628a Q \u6539\u5199\u4e3a Q\u00d7dt\uff1b\u77e9\u9635\u5f02\u5e38\u65f6\u590d\u4f4d\u5e76\u58f0\u660e\u8f93\u51fa\u4e0d\u53ef\u7528\u3002"],["18_hfi","HFI \u626b\u63cf\u8d77\u6b65\u4e0e\u4f4e\u901f\u51f8\u6781\u8ddf\u8e2a",["\u8d77\u6b65\u626b\u63cf\uff1a12 \u4e2a\u8f74\n\u03b8axis=\u03c0\u00d7axis/12","\u5404\u8f74\u4ea4\u66ff \u00b1Vd\n\u7edf\u8ba1\u6295\u5f71\u7535\u6d41\u589e\u91cf\u54cd\u5e94","\u6700\u5927\u54cd\u5e94\u8f74 + \u6781\u6027\u6bd4\u8f83\n\u5fc5\u8981\u65f6\u8865 \u03c0","\u7f6e\u4fe1\u5ea6=(max\u2212min)/max\n\u662f\u5426\u5141\u8bb8\u4f7f\u7528\u521d\u59cb\u89d2","\u72ec\u7acb\u8ddf\u8e2a HFI\n\u0394Iq\u00d7\u6ce8\u5165\u6781\u6027 \u89e3\u8c03","\u89d2\u5ea6\u6309 gain\u00d7error \u4fee\u6b63\n\u53d1\u5e03\u4ea4\u66ff Vd \u6ce8\u5165"],"App/motor/start/motor_hfi_start.c|App/motor/observer/motor_hfi.c","\u626b\u63cf\u4e0e\u8ddf\u8e2a\u662f\u4e24\u4e2a\u6a21\u5757\uff1b\u4f9d\u8d56\u51f8\u6781/\u9971\u548c\u54cd\u5e94\uff0c\u5b58\u5728\u6027\u4e0d\u4ee3\u8868\u4efb\u610f\u7535\u673a\u4e0a\u5df2\u9a8c\u6536\u3002"],["19_bldc","BLDC \u516d\u6b65\u4e0e BEMF \u9762\u79ef\u6362\u76f8",["\u5b9a\u89d2\u5bf9\u9f50 / \u5f00\u73af\u542f\u52a8\u659c\u5761\n60\u00b0 \u6362\u76f8\u5468\u671f\u4e0b\u964d","\u6bcf\u6b65\uff1a\u4e00\u76f8 PWM\n\u4e00\u76f8 LOW\uff0c\u4e00\u76f8 FLOAT","\u865a\u62df\u4e2d\u6027\u70b9 Vn=(Va+Vb+Vc)/3\nBEMF=Vfloat\u2212Vn","\u6d88\u9690/\u91c7\u6837\u7a97/\u6781\u6027\u68c0\u67e5\n\u6b63\u786e\u6781\u6027\u6263\u566a\u58f0\u540e\u79ef\u5206","\u9762\u79ef\u8fbe\u5230\u672c\u6b65\u9608\u503c\n\u5468\u671f\u9650\u5236 + \u6362\u5230\u4e0b\u4e00\u6b65","\u95ed\u73af\u5360\u7a7a\u6bd4\u5e73\u6ed1\u53d8\u5316\n\u5931\u9501/\u8d85\u65f6\u6d6e\u7a7a\u5e76\u9000\u51fa\u6062\u590d"],"App/motor/bldc/motor_bldc_sensorless.c|App/motor/bldc/motor_bldc_six_step.c|Library/Hal/port/stm32f4/hal_motor_bldc_stm32f4.c","\u91c7\u7528 BEMF \u79ef\u5206\u9762\u79ef\uff0c\u4e0d\u80fd\u753b\u6210\u5355\u7eaf\u8fc7\u96f6\u5ef6\u65f6 30\u00b0\uff1b\u524d\u534a\u6247\u533a\u907f\u514d\u94b3\u4f4d\u533a\uff0c\u540e\u534a\u6247\u533a\u5141\u8bb8\u79ef\u5206\u3002"],["20_detect","\u504f\u79fb\u3001R/L/\u78c1\u94fe\u68c0\u6d4b",["\u505c\u673a\u68c0\u6d4b\u547d\u4ee4\n\u68c0\u6d4b ISR \u4f18\u5148\u5360\u7528\u8f93\u51fa","\u7535\u6d41/\u7535\u538b\u504f\u79fb\n\u7d2f\u8ba1\u6837\u672c\u5747\u503c","R\uff1a\u591a\u70b9 V=R\u00d7I+offset\n\u6700\u5c0f\u4e8c\u4e58 / \u6b8b\u5dee\u68c0\u67e5","L\uff1a\u4e0d\u540c\u65b9\u5411\u53cc\u6781\u8109\u51b2\nL=(Veff\u2212R\u00d7Imid)/(di/dt)","\u78c1\u94fe\uff1a\u5f00\u73af\u52a0\u901f\u540e\u5173\u6865\u6ed1\u884c\n\u03c8f\u2248\u53cd\u7535\u52bf\u5e45\u503c/|\u03c9e|","\u7ed3\u679c\u4fdd\u5b58\u5728 RAM\n\u663e\u5f0f\u5e94\u7528/\u4fdd\u5b58\u4e3a\u72ec\u7acb\u52a8\u4f5c"],"App/motor/calibration/motor_detect.c|App/motor/calibration/motor_detect_offset.c|App/motor/calibration/motor_detect_isr.c","\u68c0\u6d4b\u8fc7\u7a0b\u6709\u4fdd\u62a4\u3001\u8d85\u65f6\u53ca\u9000\u51fa\u6062\u590d\uff1b\u6e90\u7801\u6587\u4ef6\u5934\u5b58\u5728\u65e7\u8bf4\u660e\uff0c\u672c\u56fe\u4ee5\u5b9e\u9645\u51fd\u6570\u4f53\u4e3a\u4f9d\u636e\u3002"],["21_protection","\u4fdd\u62a4\u53bb\u6296\u3001\u9996\u56e0\u9501\u5b58\u4e0e\u6700\u7ec8\u4ef2\u88c1",["\u539f\u59cb\u7269\u7406\u91cf\u7535\u6d41/\u6bcd\u7ebf\n\u6709\u6548\u901f\u5ea6\u53cd\u9988 + dt","\u8fc7\u6d41 / \u6b20\u538b / \u5835\u8f6c / \u5931\u901f\n\u6309\u5404\u81ea\u6761\u4ef6\u8ba1\u65f6","\u6761\u4ef6\u6301\u7eed\u5219\u7d2f\u52a0 dt\n\u6761\u4ef6\u89e3\u9664\u5219\u6e05\u96f6","\u9996\u6545\u969c\u539f\u56e0\u9501\u5b58\n\u8bb0\u5f55\u8fd0\u884c\u5feb\u7167","\u63a7\u5236\u7ed3\u679c\u6700\u7ec8\u6545\u969c\u4ef2\u88c1\n\u91cd\u7f6e\u8fd0\u884c\u79ef\u5206\u5e76\u5e94\u7528 SAFE","\u663e\u5f0f\u6e05\u6545\u969c + \u65b0\u542f\u52a8\u547d\u4ee4\n\u786c\u4ef6\u5173\u65ad\u5355\u72ec\u6838\u5bf9"],"App/motor/protect/motor_protect.c|App/motor/core/motor_foc_isr.c|App/motor/core/motor_control.c","\u65e0\u53ef\u9760\u901f\u5ea6\u53cd\u9988\u65f6\u4e0d\u8fdb\u884c\u5835\u8f6c/\u5931\u901f\u5224\u5b9a\uff1b\u8f6f\u4ef6 SAFE \u4e3a\u4e2d\u70b9\u7535\u538b\u8bed\u4e49\uff0c\u786c\u4ef6\u77ed\u8def\u4fdd\u62a4\u4e0d\u7531\u6b64\u56fe\u9a8c\u6536\u3002"],["22_storage","\u53cc\u69fd Flash\uff1a\u6700\u540e\u5199\u63d0\u4ea4\u6807\u8bb0",["\u505c\u673a\u6761\u4ef6\u68c0\u67e5\n\u6355\u83b7\u914d\u7f6e\u4e0e\u6709\u6548\u6821\u51c6","\u663e\u5f0f\u7ebf\u683c\u5f0f\u7f16\u7801\n\u53c2\u6570 + \u6297\u9f7f\u69fd\u8868","\u9009\u53e6\u4e00\u69fd\n\u5e8f\u53f7+1 / \u957f\u5ea6 / CRC32C","\u64e6\u76ee\u6807\u69fd \u2192 \u5199\u4e3b\u4f53\n\u6700\u540e\u5199 committed \u6807\u8bb0","\u56de\u8bfb\u5b8c\u6574\u5408\u6cd5\u6027\u6821\u9a8c\n\u65e7\u6709\u6548\u69fd\u4fdd\u7559","\u542f\u52a8\u9009\u6700\u65b0\u6709\u6548\u5e8f\u53f7\n\u65e0\u6709\u6548\u69fd\u56de\u9ed8\u8ba4"],"App/motor_param/flash_storage.c|App/motor_param/motor_settings.c","\u4e0d\u4fdd\u5b58\u8fd0\u884c enable\u3001\u547d\u4ee4\u76ee\u6807\u3001PI \u79ef\u5206\u6216\u6307\u9488\uff1b\u7248\u672c\u517c\u5bb9\u4ee5\u5f53\u524d\u89e3\u7801\u51fd\u6570\u4e3a\u51c6\u3002"],["23_compensation","\u5f31\u78c1\u3001\u901f\u5ea6\u524d\u9988\u62df\u5408\u4e0e\u6297\u9f7f\u69fd",["\u5f31\u78c1\uff1am=\u221a(vd\u00b2+vq\u00b2)/Vlim\n\u89e6\u53d1\u4e0e\u9000\u51fa\u6ede\u56de","\u8d1f Id \u6309 A/s \u659c\u5761\nId \u4f59\u91cf=\u221a(Ibudget\u00b2\u2212Iq\u00b2)","\u901f\u5ea6\u524d\u9988\u62df\u5408\n|Vq|=slope\u00d7|RPM|+boost","\u591a\u70b9\u56de\u5f52 / RMS \u6b8b\u5dee\n\u975e\u6cd5\u659c\u7387\u4e0e\u622a\u8ddd\u62d2\u7edd","\u6297\u9f7f\u69fd\uff1a\u6309\u673a\u68b0\u89d2\u5206\u6876\n\u8bb0\u5f55\u4fdd\u6301\u7535\u538b\u5e76\u51cf\u5747\u503c","\u8fd0\u884c\u63d2\u503c\u53d6\u8865\u507f\u7535\u538b\n\u6309\u8c03\u7528\u8def\u5f84\u5408\u6210/\u9650\u5e45"],"App/motor/loop/motor_fw.c|App/motor/loop/motor_speed_ff_detect.c|App/motor/loop/motor_anticogging.c","\u4e09\u4e2a\u8282\u70b9\u7ec4\u4ee3\u8868\u4e09\u79cd\u72ec\u7acb\u8865\u507f\uff1b\u4e0d\u8868\u793a\u5b83\u4eec\u5fc5\u987b\u4e32\u8054\u3002\u6297\u9f7f\u69fd\u8865\u507f\u5f53\u524d\u4ee5\u7535\u538b\u8868\u8868\u793a\u3002"],["24_commands","\u547d\u4ee4\u3001\u72b6\u6001\u5feb\u7167\u4e0e\u5173\u8282\u56fa\u5b9a\u5e27",["Shell / \u5173\u8282\u534f\u8bae\n\u7528\u6237\u4f7f\u80fd\u4e0e\u76ee\u6807","\u5173\u8282\u547d\u4ee4\u5934 FE EE\n\u72b6\u6001\u5934 FC EE\n\u591a\u5b57\u8282\u5b57\u6bb5\u5927\u7aef","\u5355\u4f4d/\u6709\u9650\u503c/\u8303\u56f4\u6821\u9a8c\n\u5b8c\u6574\u547d\u4ee4\u63d0\u4ea4","\u77ed\u4e34\u754c\u533a\u6574\u4f53\u4ea4\u63a5\n\u4e0d\u6df7\u5165\u65e5\u5fd7\u6216\u603b\u7ebf\u7b49\u5f85","ISR/\u4efb\u52a1\u63a8\u8fdb\u8fd0\u884c\u72b6\u6001\n\u72b6\u6001\u5feb\u7167\u4f9b\u5916\u90e8\u8bfb\u53d6","\u534f\u8bae\u8d85\u65f6/\u65e0\u6548\u6570\u636e\u5904\u7406\n\u663e\u5f0f\u505c\u673a\u4e0e\u6545\u969c\u4fdd\u6301"],"App/motor/motor.c|App/motor/motor_snapshot.c|App/motor/joint/motor_joint_protocol.c|App/motor/joint/motor_joint.c","\u547d\u4ee4\u4e00\u81f4\u6027\u4e0d\u4f9d\u9760 volatile \u4fdd\u8bc1\uff1b\u72b6\u6001\u5feb\u7167\u4e0e\u63a7\u5236\u5185\u90e8\u79ef\u5206\u5206\u79bb\u3002"]]''')
BLUE="#245b8a"; GREEN="#dff1e7"; RED="#fde8e7"; GOLD="#fff1d2"
def txt(s): return s
def finish(fig,name):
    fig.savefig(FIG/(name+".png"),dpi=160,facecolor="white")
    fig.savefig(FIG/(name+".svg"),facecolor="white")
    plt.close(fig)
def node(ax,x,y,text,col):
    text=text.replace('ᵀ', '^T').replace('⁻¹', '^(-1)')
    ax.add_patch(FancyBboxPatch((x,y),.40,.16,boxstyle="round,pad=0.008,rounding_size=0.018",
                              fc=col,ec=BLUE,lw=1.4))
    ax.text(x+.20,y+.08,text,ha="center",va="center",fontsize=13,linespacing=1.6)
def link(ax,a,b):
    ax.add_patch(FancyArrowPatch(a,b,arrowstyle="-|>",mutation_scale=17,lw=1.5,color=BLUE))
def flow(spec):
    name,title,steps,paths,note=spec
    fig,ax=plt.subplots(figsize=(14,8))
    fig.subplots_adjust(left=.025,right=.975,bottom=.02,top=.98)
    ax.set(xlim=(0,1),ylim=(0,1)); ax.axis("off")
    ax.text(.035,.95,title,fontsize=21,color=BLUE,weight="bold")
    if name == '20_detect':
        small=[(.03,.72,.94,.11,steps[0]),(.03,.51,.44,.13,steps[1]),
               (.53,.51,.44,.13,steps[2]),(.03,.30,.44,.13,steps[3]),
               (.53,.30,.44,.13,steps[4]),(.03,.11,.94,.11,steps[5])]
        for x,y,w,h,label in small:
            ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0.008',fc=GREEN,ec=BLUE,lw=1.4))
            ax.text(x+w/2,y+h/2,label,ha='center',va='center',fontsize=12)
        for x in [.25,.75]:
            link(ax,(x,.71),(x,.65))
            link(ax,(x,.29),(x,.23))
        ax.text(.50,.465,'按命令选择对应检测（四种独立方法）',ha='center',fontsize=10,color=BLUE,
                bbox=dict(facecolor='white',edgecolor='none'))
        ax.text(.035,.035,'R/L/磁链结果仅 RAM；偏移结果写入校准字段。显式保存另行执行。',fontsize=10)
        ax.text(.035,.005,'App/motor/calibration/motor_detect.c / motor_detect_offset.c',fontsize=9,color='#666')
        finish(fig,name);return
    coords=[(.05,.70),(.55,.70),(.55,.45),(.05,.45),(.05,.20),(.55,.20)]
    for i,((x,y),step) in enumerate(zip(coords,steps)):
        node(ax,x,y,step,[GREEN,GOLD,GREEN,GOLD,GREEN,RED][i])
        ax.text(x+.012,y+.145,str(i+1),fontsize=10,color=BLUE,weight="bold")
    if name == '04_output':
        link(ax,(.458,.78),(.542,.78))
        link(ax,(.65,.692),(.75,.618))
        link(ax,(.55,.70),(.25,.618))
        # Route the linear-SVM branch through the empty inter-row corridor.
        ax.plot([.75,.75,.48],[.442,.405,.405],color=BLUE,lw=1.5)
        link(ax,(.48,.405),(.40,.368))
        link(ax,(.25,.442),(.25,.368))
        link(ax,(.458,.28),(.542,.28))
    elif name in ('18_hfi','23_compensation','20_detect'):
        # Distinct methods: only connect within each method, not across methods.
        if name == '23_compensation':
            link(ax,(.458,.78),(.542,.78))
            link(ax,(.458,.53),(.542,.53))
            link(ax,(.458,.28),(.542,.28))
        elif name == '18_hfi':
            link(ax,(.458,.78),(.542,.78));link(ax,(.75,.692),(.75,.618))
            link(ax,(.542,.53),(.458,.53));link(ax,(.458,.28),(.542,.28))
        else:
            for end in [(.542,.78),(.55,.60),(.25,.618),(.25,.368)]:
                link(ax,(.45,.70),end)
            for start in [(.75,.70),(.75,.45),(.25,.45),(.45,.28)]:
                link(ax,start,(.70,.368))
    else:
        link(ax,(.458,.78),(.542,.78));link(ax,(.75,.692),(.75,.618))
        link(ax,(.542,.53),(.458,.53));link(ax,(.25,.442),(.25,.368))
        link(ax,(.458,.28),(.542,.28))
    ax.text(.035,.135,note,fontsize=11,color="#444",wrap=True)
    for i,path in enumerate(paths.split("|")):
        ax.text(.035,.09-i*.023,path,fontsize=9,color="#666")
    finish(fig,name)

SPECS[19][4] += ' 偏移/R/L/磁链为按命令分别执行的检测分支，不是必经串联。'
SPECS[23][2][1] += '\nCRC-32/ISO-HDLC 校验'
for spec in SPECS: flow(spec)

# Static alpha/beta axes and rotating d/q projections, with the actual Park signs.
fig,ax=plt.subplots(figsize=(10,8)); ax.set_aspect('equal')
theta=.6; current=np.array([.4,.9]); d=np.array([np.cos(theta),np.sin(theta)])
q=np.array([-np.sin(theta),np.cos(theta)])
for vec,label,color in [(np.array([1.,0.]),'alpha',BLUE),(np.array([0.,1.]),'beta',BLUE),
                        (d,'d','#c57822'),(q,'q','#8255a3'),(current,'I','#bd5160')]:
    ax.annotate('',xy=vec,xytext=(0,0),arrowprops=dict(arrowstyle='-|>',color=color,lw=2))
    ax.text(*(vec*1.10),label,fontsize=15,color=color)
proj_d=np.dot(current,d)*d; proj_q=np.dot(current,q)*q
ax.plot([current[0],proj_d[0]],[current[1],proj_d[1]],'--',color='#c57822')
ax.plot([current[0],proj_q[0]],[current[1],proj_q[1]],'--',color='#8255a3')
ax.set(xlim=(-1.3,1.3),ylim=(-1.3,1.3),xlabel='alpha',ylabel='beta')
ax.axhline(0,color='#ddd');ax.axvline(0,color='#ddd');ax.grid(alpha=.2)
ax.set_title('Clarke / Park: static axes and rotating projections',fontsize=18)
fig.tight_layout();finish(fig,'30_coordinate_geometry')

# DC-link voltage vector hexagon and the guaranteed linear-region circle.
fig,ax=plt.subplots(figsize=(10,8));ax.set_aspect('equal')
angles=np.arange(7)*np.pi/3; radius=2/3
ax.plot(radius*np.cos(angles),radius*np.sin(angles),color=BLUE,lw=2,label='active-vector hexagon')
a=np.linspace(0,2*np.pi,400);ax.plot(np.cos(a)/np.sqrt(3),np.sin(a)/np.sqrt(3),'--',label='Vbus/sqrt(3) circle')
for j in range(6):
    ax.plot([0,radius*np.cos(angles[j])],[0,radius*np.sin(angles[j])],color='#bbb')
    ax.text(.76*np.cos(angles[j]),.76*np.sin(angles[j]),f'V{j+1}',ha='center',va='center',fontsize=14)
target=.45*np.array([np.cos(.4),np.sin(.4)])
ax.annotate('',xy=target,xytext=(0,0),arrowprops=dict(arrowstyle='-|>',color='#bd5160',lw=2))
ax.text(-.74,-.82,'mod_alpha/beta = 1.5 * V_alpha/beta / Vbus\n|mod| <= sqrt(3)/2 * max_duty',fontsize=12)
ax.set(xlim=(-.95,.95),ylim=(-.9,.9),xlabel='Valpha / Vbus',ylabel='Vbeta / Vbus')
ax.set_title('SVPWM: adjacent active vectors and zero-vector time',fontsize=18)
ax.legend(loc='upper right');ax.grid(alpha=.2);fig.tight_layout();finish(fig,'31_svpwm_hexagon')

# Exact full three-shunt Clarke and Park sample, teaching waveform only.
t=np.linspace(0,2*np.pi,800); theta=t
ia=np.cos(t); ib=np.cos(t-2*np.pi/3); ic=np.cos(t+2*np.pi/3)
alpha=(2*ia-ib-ic)/3; beta=(ib-ic)/np.sqrt(3)
id_=alpha*np.cos(theta)+beta*np.sin(theta)
iq=-alpha*np.sin(theta)+beta*np.cos(theta)
fig,axes=plt.subplots(3,1,figsize=(13,9),sharex=True)
for ax,curves in zip(axes,[[(ia,"Ia"),(ib,"Ib"),(ic,"Ic")],[(alpha,"Ialpha"),(beta,"Ibeta")],[(id_,"Id"),(iq,"Iq")]]):
    for y,label in curves: ax.plot(t,y,label=label,lw=2)
    ax.grid(alpha=.2);ax.legend(loc="upper right",ncol=3);ax.set_ylabel("A")
axes[-1].set_xlabel("theta_e (rad)")
fig.suptitle("\u4e09\u76f8\u7535\u6d41 \u2192 Clarke \u2192 Park\uff08\u540c\u6b65\u89d2\u793a\u610f\uff09",fontsize=20)
fig.tight_layout();finish(fig,"25_transform_waveforms")
assert np.max(np.abs(id_-1))<1e-12 and np.max(np.abs(iq))<1e-12

# Geometric voltage circle (not measured response).
fig,ax=plt.subplots(figsize=(10,8));ax.set_aspect("equal")
ang=np.linspace(0,2*np.pi,400);lim=12/np.sqrt(3)
ax.plot(lim*np.cos(ang),lim*np.sin(ang),label="Vbus/sqrt(3), Vbus=12 V")
v=np.array([7.,6.]);vs=v*min(1,lim/np.linalg.norm(v))
for val,color,label in [(v,"#b74a53","raw"),(vs,BLUE,"limited")]:
    ax.annotate("",xy=val,xytext=(0,0),arrowprops=dict(arrowstyle="-|>",color=color,lw=2))
    ax.scatter(*val,color=color,label=label)
ax.axhline(0,color="#aaa");ax.axvline(0,color="#aaa")
ax.set(xlim=(-9,9),ylim=(-9,9),xlabel="Vd (V)",ylabel="Vq (V)")
ax.legend();ax.grid(alpha=.2);ax.set_title("\u7535\u538b\u77e2\u91cf\u9650\u5e45\uff1a\u4fdd\u7559\u65b9\u5411\u540c\u6bd4\u7f29\u653e",fontsize=18)
fig.tight_layout();finish(fig,"26_voltage_circle")

# Six-step table transcribed from s_forward_steps.
table=np.array([[1,-1,0],[1,0,-1],[0,1,-1],[-1,1,0],[-1,0,1],[0,-1,1]])
fig,ax=plt.subplots(figsize=(12,5))
from matplotlib.colors import ListedColormap,BoundaryNorm
cmap=ListedColormap(["#7eafe1","#eeeeee","#efb36a"])
ax.imshow(table.T,aspect="auto",cmap=cmap,norm=BoundaryNorm([-1.5,-.5,.5,1.5],3))
for j in range(3):
    for i in range(6): ax.text(i,j,{-1:"LOW",0:"FLOAT",1:"PWM"}[int(table[i,j])],ha="center",va="center",fontsize=16)
ax.set_xticks(range(6),[str(i) for i in range(6)]);ax.set_yticks(range(3),["A","B","C"])
ax.set_xlabel("forward step");ax.set_title("\u516d\u6b65\u6865\u81c2\u72b6\u6001\uff08\u4e0e\u4ee3\u7801 s_forward_steps \u4e00\u81f4\uff09",fontsize=18)
fig.tight_layout();finish(fig,"27_bldc_table")

# Conceptual BEMF integral after blanking/noise; thresholds illustrative.
time=np.linspace(0,1,400);bemf=2*(time-.3);noise=.1
valid=time>.12
area=np.cumsum(np.where(valid,np.maximum(bemf-noise,0),0))*(time[1]-time[0])
fig,axes=plt.subplots(2,1,figsize=(12,7),sharex=True)
axes[0].plot(time,bemf,label="signed BEMF");axes[0].axhline(noise,color="orange",ls="--",label="noise")
axes[0].axvspan(0,.12,color="#ddd",alpha=.5,label="blanking")
axes[1].plot(time,area,label="positive integral")
axes[1].axhline(.25,color="red",ls="--",label="illustrative threshold")
for ax in axes: ax.legend();ax.grid(alpha=.2)
axes[0].set_ylabel("V");axes[1].set_ylabel("normalized V*time");axes[1].set_xlabel("normalized sector time")
fig.suptitle("\u60ac\u7a7a\u76f8 BEMF \u9762\u79ef\u6362\u76f8\uff08\u6559\u5b66\u793a\u610f\uff0c\u975e\u5b9e\u6d4b\uff09",fontsize=18)
fig.tight_layout();finish(fig,"28_bemf_area")

# Encoder shortest-path wrap and current quantization.
fig,axes=plt.subplots(1,2,figsize=(14,5))
raw=np.array([16378,16381,16383,1,4,7,10]);unwrap=np.unwrap(raw/16384*2*np.pi)*16384/(2*np.pi)
axes[0].plot(raw,"o-",label="raw count");axes[0].plot(unwrap,"o-",label="unwrapped")
axes[0].set_xlabel("sample");axes[0].set_ylabel("count");axes[0].legend();axes[0].grid(alpha=.2)
i=np.linspace(-.4,.4,600);axes[1].plot(i,i,label="ideal");axes[1].plot(i,np.round(i/.0806)*.0806,label="0.0806 A/count")
axes[1].set_xlabel("current (A)");axes[1].set_ylabel("quantized (A)");axes[1].legend();axes[1].grid(alpha=.2)
fig.suptitle("\u56de\u7ed5\u5904\u7406\u4e0e BM75 \u4f4e\u7535\u6d41\u91cf\u5316\uff08\u793a\u610f\uff09",fontsize=18)
fig.tight_layout();finish(fig,"29_wrap_quantization")

# Reproducibility evidence: hash source files and inventory first-party implementation.
sources=sorted({p for spec in SPECS for p in spec[3].split("|")})
evidence={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources}
(OUT/"source_hashes.json").write_text(json.dumps(evidence,indent=2),encoding="utf-8")
inventory=[]
for base in ["App/motor","App/motor_sensor","App/motor_param"]:
    for p in sorted((ROOT/base).rglob("*.c")):
        rel=p.relative_to(ROOT).as_posix()
        text=p.read_text(encoding="utf-8",errors="replace")
        desc=re.search(r"@Description:\s*([^\r\n]+)",text)
        inventory.append({"path":rel,"description":desc.group(1) if desc else "", "direct_diagrams":[s[0] for s in SPECS if rel in s[3].split("|")]})
(OUT/"module_inventory.json").write_text(json.dumps(inventory,ensure_ascii=False,indent=2),encoding="utf-8")
# Contact sheet for visual review.
files=sorted(FIG.glob("*.png"));thumbs=[]
for f in files:
    im=Image.open(f).convert("RGB");im.thumbnail((360,215))
    tile=Image.new("RGB",(380,245),"white");tile.paste(im,((380-im.width)//2,0))
    ImageDraw.Draw(tile).text((10,222),f.stem,fill="black")
    thumbs.append(tile)
sheet=Image.new("RGB",(380*4,245*((len(thumbs)+3)//4)),"#dce1e6")
for n,tile in enumerate(thumbs):sheet.paste(tile,((n%4)*380,(n//4)*245))
sheet.save(OUT/"contact_sheet.jpg")
# Readable gallery: UTF-8 Chinese names stay in metadata, image paths ASCII.
lines=["# 当前代码控制原理图（Python 绘制）","","依据：2026-10-09 工作区实际源码，包含已有未提交修改。只新增此目录，未修改固件或外设基线。",
       "","## 阅读与重新生成","","安装 Python、matplotlib、numpy、Pillow 后，在本目录运行：","","```powershell","python generate_diagrams.py","```",
       "","images/ 同时包含 PNG 和可放大编辑的 SVG；source_hashes.json 记录源码 SHA-256；module_inventory.json 列出项目电机实现文件与直接对应图件。",
       "","## 范围和约定","","覆盖电机控制、启动、采样、传感器、外环、无感、BLDC、校准检测、保护、补偿及存储。第三方库内部、串口驱动、RTOS 内核及历史 motor_simple_foc 示例不作为另一套主控制实现展开。流程图表达模块原理，不逐个绘制打印/包装函数。",
       "","流程页按 1→2→3→4→5→6 阅读；明确标注的条件分支与独立补偿不是无条件串联。曲线图是给定参数的教学示意，不是仿真验收或硬件实测。",
       "","特别注意：正常 FOC 先筛选控制拍再采样/保护；速度 PI 在任务运行；快速 V/F 是恒 boost 电压诊断；逆 Park 整体负号、线性 SVM 二次取负和 A-C-B 提交必须整体理解。SAFE 是中点占空比，不能推断关桥。",
       "","独立 flux/Luenberger/SMO/EKF/HFI 模块的存在不代表默认主路径正在运行；主 sensorless 当前直接实现 Ortega/PLL。"]
for name,title,steps,paths,note in SPECS:
    lines+=["", "## "+name+" "+title, "", "!["+title+"](images/"+name+".png)", "",note,"",
            "源码："+ "、".join("["+p+"](../../"+p+")" for p in paths.split("|"))]
for name,label in [("25_transform_waveforms","Clarke/Park 波形"),("26_voltage_circle","电压矢量圆限幅"),("27_bldc_table","六步桥臂表"),("28_bemf_area","BEMF 积分面积"),("29_wrap_quantization","编码器回绕与低电流量化"),("30_coordinate_geometry","静止坐标与旋转坐标几何投影"),("31_svpwm_hexagon","SVPWM 空间矢量六边形与线性区")]:
    lines+=["","## "+label,"","!["+label+"](images/"+name+".png)"]
(OUT/"README.md").write_text("\n".join(lines)+"\n",encoding="utf-8")
readme = OUT / 'README.md'
content = readme.read_text(encoding='utf-8')
content = content.replace('# 当前代码控制原理图（Python 绘制）',
    '# 当前代码控制原理图（Python 绘制）\n\n**先看物理与控制原理：[原理图阅读.md](原理图阅读.md)。**\n\n新增 principles/ 包含磁场、转矩、负反馈、电路、时序和检测关系共 12 组原理图；运行 `python draw_principles.py` 重新生成。下面 images/ 的图件是源码结构与流程辅助说明。', 1)
readme.write_text(content, encoding='utf-8')
print(f"Generated {len(files)} PNG + {len(list(FIG.glob('*.svg')))} SVG diagrams in {OUT}")

