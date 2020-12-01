from common.numpy_fast import clip
from selfdrive.car.ford.values import MAX_ANGLE

def create_steer_command(packer, angle_cmd, enabled, lkas_state, angle_steers, curvature, lkas_action):
  """Creates a CAN message for the Ford Steer Command."""

  #angle_cmd = clip(angle_cmd * MAX_ANGLE, - MAX_ANGLE, MAX_ANGLE)
  if enabled and lkas_state in [2,3]:
    action = lkas_action
  else:
    action = 0xf

  values = {
    "Lkas_Action": action,
    "Lkas_Alert": 0xe,             # no alerts
    "Lane_Curvature": clip(curvature, -0.03, 0.03),   # is it just for debug?
    "Steer_Angle_Req": angle_cmd
  }
  return packer.make_can_msg("Lane_Keep_Assist_Control", 0, values)

def create_lkas_ui(packer, main_on, enabled, steer_alert, defog, ahbc, ahbcramping, config, noipma, stats, persipma, dasdsply, x30):
  """Creates a CAN message for the Ford Steer Ui."""
  if enabled:
    lines = 0x6
  else:
    lines = 0xc

  values = {
    "PersIndexIpma_D_Actl": persipma,
    "DasStats_D_Dsply": dasdsply,
    "Set_Me_X30": x30,
    "Lines_Hud": lines,
    "Hands_Warning_W_Chime": steer_alert,
    "CamraDefog_B_Req": defog,
    "AhbHiBeam_D_Rq": ahbc,
    "AhbcRampingV_D_Rq": ahbcramping,
    "FeatConfigIpmaActl": config,
    "FeatNoIpmaActl": noipma,
    "CamraStats_D_Dsply": stats,
  }
  return packer.make_can_msg("Lane_Keep_Assist_Ui", 0, values)

def spam_cancel_button(packer):
  values = {
    "Cancel": 1
  }
  return packer.make_can_msg("Steering_Buttons", 0, values)
