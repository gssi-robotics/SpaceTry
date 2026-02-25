{-# OPTIONS_GHC -Wno-unused-imports #-}
module CuriosityMonitor where

import qualified Copilot.Language as C
import Copilot.PtLTL

-- === Extern Signals (from mapping) ===
at_science_rock :: C.Stream Bool
at_science_rock = C.extern "at_science_rock" Nothing
battery_near_outpost :: C.Stream Bool
battery_near_outpost = C.extern "battery_near_outpost" Nothing
battery_soc :: C.Stream C.Double
battery_soc = C.extern "battery_soc" Nothing
cmd_vel_ang :: C.Stream C.Double
cmd_vel_ang = C.extern "cmd_vel_ang" Nothing
cmd_vel_lin :: C.Stream C.Double
cmd_vel_lin = C.extern "cmd_vel_lin" Nothing
delta_yaw_deg :: C.Stream C.Double
delta_yaw_deg = C.extern "delta_yaw_deg" Nothing
obstacle_front :: C.Stream Bool
obstacle_front = C.extern "obstacle_front" Nothing
obstacle_left :: C.Stream Bool
obstacle_left = C.extern "obstacle_left" Nothing
obstacle_right :: C.Stream Bool
obstacle_right = C.extern "obstacle_right" Nothing

-- === Derived Boolean Signals ===
battery_low :: C.Stream Bool
battery_low = (battery_soc C.<= C.constant 0.2)
battery_high :: C.Stream Bool
battery_high = (battery_soc C.> C.constant 0.8)
lin_le_0_5 :: C.Stream Bool
lin_le_0_5 = (cmd_vel_lin C.<= C.constant 0.5)
lin_le_1_0 :: C.Stream Bool
lin_le_1_0 = (cmd_vel_lin C.<= C.constant 1.0)
lin_ge_2_0 :: C.Stream Bool
lin_ge_2_0 = (cmd_vel_lin C.>= C.constant 2.0)
ang_le_neg0_25 :: C.Stream Bool
ang_le_neg0_25 = (cmd_vel_ang C.<= C.constant (-0.25))
ang_ge_0_25 :: C.Stream Bool
ang_ge_0_25 = (cmd_vel_ang C.>= C.constant 0.25)
yaw_pos_ge_60 :: C.Stream Bool
yaw_pos_ge_60 = (delta_yaw_deg C.>= C.constant 60.0)
yaw_neg_le_neg60 :: C.Stream Bool
yaw_neg_le_neg60 = (delta_yaw_deg C.<= C.constant (-60.0))

-- === Formalized Properties ===
-- MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY: (H (((((((battery_high & (! obstacle_front)) & (! obstacle_left)) & (! obstacle_right)) & (! at_science_rock)) & (! battery_near_outpost)) -> lin_ge_2_0)))
phi_MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY :: C.Stream Bool
phi_MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY = ptltl_H (C.not ((battery_high C.&& C.not (obstacle_front) C.&& C.not (obstacle_left) C.&& C.not (obstacle_right) C.&& C.not (at_science_rock) C.&& C.not (battery_near_outpost))) C.|| lin_ge_2_0)
viol_MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY :: C.Stream Bool
viol_MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY = C.not phi_MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY

-- MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY: (H ((O[60,60] (battery_low & (! battery_near_outpost))) -> (O[0,59] ((Z FALSE) | battery_near_outpost))))
phi_MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY :: C.Stream Bool
phi_MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY = ptltl_H (C.not (ptltl_Ow 60 60 (battery_low C.&& C.not (battery_near_outpost))) C.|| ptltl_Ow 0 59 (ptltl_Z (C.false) C.|| battery_near_outpost))
viol_MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY :: C.Stream Bool
viol_MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY = C.not phi_MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY

-- === Triggers ===
spec :: C.Spec
spec = do
  C.trigger "handlerMR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY" (viol_MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY) []
  C.trigger "handlerMR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY" (viol_MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY) []
