from cereal import car
from common.numpy_fast import interp, clip
from selfdrive.car import make_can_msg
from selfdrive.car.ford.fordcan import create_steer_command, create_speed_command, create_speed_command2, create_ds_118, create_lkas_ui, spam_cancel_button
from selfdrive.car.ford.values import CAR, CarControllerParams
from opendbc.can.packer import CANPacker
from common.op_params import opParams

op_params = opParams()
enable_angle_live = op_params.get('enable_angle_live')

MAX_STEER_DELTA = 0.2
TOGGLE_DEBUG = False
COUNTER_MAX = 7

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.packer = CANPacker(dbc_name)
    self.enable_camera = CP.enableCamera
    self.enabled_last = False
    self.main_on_last = False
    self.vehicle_model = VM
    self.generic_toggle_last = 0
    self.steer_alert_last = False
    self.lkas_action = 0
    #self.lkasToggle = 1
    self.lastAngle = 0
    self.angleReq = 0
    self.sappConfig = 0
    self.sappChime = 0
    self.chimeCounter = 0
    self.sappConfig_last = 0
    self.angleReq_last = 0
    self.apaCounter = 0
    self.sappAction = 0
    self.eightysix = 0
    self.alwaysTrue = True
  def update(self, enabled, CS, frame, actuators, visual_alert, pcm_cancel):
  
    frame_step = CarControllerParams.FRAME_STEP
    if enable_angle_live:
      ANGLE_MAX_BP = op_params.get('angle_max_bp')
      ANGLE_MAX_V = op_params.get('angle_max_v')
      ANGLE_DELTA_BP = op_params.get('angle_delta_bp')
      ANGLE_DELTA_V = op_params.get('angle_delta_v')
      ANGLE_DELTA_VU = op_params.get('angle_delta_vu')
    else:
      ANGLE_MAX_BP = [0., 11., 36.]
      ANGLE_MAX_V = [410., 25., 15.]
      ANGLE_DELTA_BP = [0., 5., 15.]
      ANGLE_DELTA_V = [5., .8, .15]     #windup
      ANGLE_DELTA_VU = [5., 3.5, 0.4] #unwind

    can_sends = []
    steer_alert = visual_alert == car.CarControl.HUDControl.VisualAlert.steerRequired
    apply_steer = actuators.steeringAngleDeg
    if self.enable_camera:
      if CS.epsAssistLimited:
        print("PSCM Assist Limited")
      if (frame % 2) == 0:
        if self.alwaysTrue == True:
          self.actlnocs = 0
          self.actlbrknocs = 0
          self.speed = 0
          self.drvstate = 6
          can_sends.append(create_ds_118(self.packer,  CS.filler1, CS.filler2, CS.filler3, CS.brakectr, CS.awdlckmax, CS.awdlckmn, self.drvstate, CS.drvtq, CS.emergbrk, CS.stoplmp, CS.angle))
          can_sends.append(create_speed_command(self.packer, enabled, frame, self.speed, CS.trlraid, self.actlnocs, CS.actlnocnt, CS.actlqf, CS.epsgear, frame_step))
          can_sends.append(create_speed_command2(self.packer, enabled, frame, self.speed, CS.lsmcdecel, self.actlbrknocs, CS.actlbrknocnt, CS.actlbrkqf, CS.brkfld, CS.stab_stat, frame_step))
        else:
          can_sends.append(create_ds_118(self.packer,  CS.filler1, CS.filler2, CS.filler3, CS.brakectr, CS.awdlckmax, CS.awdlckmn, CS.drvstate, CS.drvtq, CS.emergbrk, CS.stoplmp, CS.angle))
          can_sends.append(create_speed_command(self.packer, enabled, frame, CS.vehSpeed, CS.trlraid, CS.actlnocs, CS.actlnocnt, CS.actlqf, CS.epsgear, frame_step))
          can_sends.append(create_speed_command2(self.packer, enabled, frame, CS.vehSpeed2, CS.lsmcdecel, CS.actlbrknocs, CS.actlbrknocnt, CS.actlbrkqf, CS.brkfld, CS.stab_stat, frame_step))
      if pcm_cancel:
       #print("CANCELING!!!!")
        can_sends.append(spam_cancel_button(self.packer))
      if (frame % 1) == 0:
        self.main_on_last = CS.out.cruiseState.available
      #SAPP Config Value Handshake
      if (frame % 2) == 0:
        if not enabled:
          self.apaCounter = 0
          self.eightysix = 0
          self.angleReq = 0
          self.sappAction = 0
        if enabled:
          self.apaCounter += 1 #Increment counter 
          #Sets config to base value when init handshake
          if CS.sappHandshake == 0 and self.sappConfig_last not in [16, 86, 224] :
            self.sappConfig = 70
          #waits for the pscm to respond, and waits 8 frames as well. sets config to response
          if CS.sappHandshake == 1 and self.apaCounter > 8:
            self.sappConfig = 86
            self.eightysix += 1
          #waits 5 frames then sends the angle request
          if CS.sappHandshake == 1 and self.apaCounter > 13 and self.sappConfig_last == 86:
            self.angleReq = 1
          #when 20 frames have passed at response config, values are cleared and angle request is held
          if self.sappConfig_last == 86 and self.eightysix == 20:
            self.apaCounter = 0
            self.eightysix = 0
            self.angleReq = 1
          #pscm responds to handshake. config is set to parallel action. 
          if CS.sappHandshake == 2 and self.sappConfig_last != 16: # and self.apaCounter in range (15,16):
            self.sappConfig = 224
            self.angleReq = 1
            self.sappAction += 1
          #once action is held for 3 frames, final response is sent. pscm is handshaken
          if CS.sappHandshake == 2 and self.sappAction >= 3 and self.sappConfig_last == 224:
            self.sappConfig = 16
            self.angleReq = 1
          #if pscm faults, values reset to retry. 
          if CS.sappHandshake == 3:
            self.sappConfig = 0
            self.apaCounter = 0
            self.angleReq = 0
        self.sappConfig_last = self.sappConfig
        self.angleReq_last = self.angleReq
        print("Handshake:", CS.sappHandshake, "Config:", self.sappConfig_last, "Desired Angle:", apply_steer, "Curr Angle:", CS.out.steeringAngleDeg) # "Counter:", self.apaCounter, "AngleRequest:", self.angleReq, "fwdAction:", self.sappAction)
        self.lkas_action = 0 #6 Finished 5 NotAccessible 4 ApaCancelled 2 On 1 Off  
        angle_lim = interp(CS.out.vEgo, ANGLE_MAX_BP, ANGLE_MAX_V)
        apply_steer = clip(apply_steer, -angle_lim, angle_lim)
        if enabled:
          if self.lastAngle * apply_steer > 0. and abs(apply_steer) > abs(self.lastAngle):
            angle_rate_lim = interp(CS.out.vEgo, ANGLE_DELTA_BP, ANGLE_DELTA_V)
          else:
            angle_rate_lim = interp(CS.out.vEgo, ANGLE_DELTA_BP, ANGLE_DELTA_VU)
          print("max_bp:", ANGLE_MAX_BP, "max_v:", ANGLE_MAX_V, "delta_bp:", ANGLE_DELTA_BP, "delta_v:", ANGLE_DELTA_V, "delta_vu:", ANGLE_DELTA_VU)
          apply_steer = clip(apply_steer, self.lastAngle - angle_rate_lim, self.lastAngle + angle_rate_lim) 
        else:
          apply_steer = CS.out.steeringAngleDeg
        self.lastAngle = apply_steer
        can_sends.append(create_steer_command(self.packer, apply_steer, enabled, CS.out.steeringAngleDeg, self.lkas_action, self.angleReq_last, self.sappConfig_last, self.sappChime))
        self.generic_toggle_last = CS.out.genericToggle
      if (frame % 1) == 0 or (self.enabled_last != enabled) or (self.main_on_last != CS.out.cruiseState.available) or (self.steer_alert_last != steer_alert):
        if steer_alert:
          self.steer_chime = 1
          self.daschime = 2
        else:
          self.steer_chime = 0
          self.daschime = 0
        can_sends.append(create_lkas_ui(self.packer, CS.out.cruiseState.available, enabled, self.steer_chime, CS.ipmaHeater, CS.ahbcCommanded, CS.ahbcRamping, CS.ipmaConfig, CS.ipmaNo, CS.ipmaStats, CS.persipma, CS.dasdsply, CS.x30, self.daschime))
        self.enabled_last = enabled                         
      self.steer_alert_last = steer_alert

    return can_sends
