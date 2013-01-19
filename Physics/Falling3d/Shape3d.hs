{-# LANGUAGE MultiParamTypeClasses #-}

module Physics.Falling3d.Shape3d
(
DynamicShape3d(..)
, StaticShape3d(..)
)
where

import Data.Vect.Double.Base
import Physics.Falling.Shape.VolumetricShape
import Physics.Falling.Shape.Ball
import Physics.Falling.Shape.Plane
import Physics.Falling3d.InertiaTensor3d

data DynamicShape3d = Ball3d       Ball
data StaticShape3d  = StaticBall3d Ball
                      | Plane3d    (Plane Vec3)

instance VolumetricShape DynamicShape3d InertiaTensor3d InverseInertiaTensor3d Vec3 Proj4 where
  volume (Ball3d b)                            = ballVolume b 3
  objectFrameInertiaTensor (Ball3d (Ball r)) m = InertiaTensor3d $ scaling (Vec3 momentum momentum momentum)
                                                 where
                                                 momentum = 2.0 / 5.0 * m * r * r