{-# LANGUAGE MultiParamTypeClasses #-}

module Physics.Falling3d.Box
(
Box(..)
, boxVolume
, boxInertiaTensor
)
where

import Data.Vect.Double.Base
import Physics.Falling.Shape.ImplicitShape
import Physics.Falling3d.InertiaTensor3d

data Box = Box Double Double Double
           deriving(Show)

instance ImplicitShape Box Vec3 where
  supportPoint (Box rx ry rz) (Vec3 dx dy dz) = Vec3 (signum dx * rx)
                                                     (signum dy * ry)
                                                     (signum dz * rz)

boxVolume :: Box -> Double
boxVolume (Box rx ry rz) = 8.0 * rx * ry * rz

boxInertiaTensor :: Box -> Double -> InertiaTensor3d
boxInertiaTensor (Box rx ry rz) m = InertiaTensor3d
                                    $ scaling
                                    $ Vec3 (m_12 * (ry2 + rz2))
                                           (m_12 * (rx2 + rz2))
                                           (m_12 * (rx2 + ry2))
                                    where
                                    m_12 = m / 12.0
                                    rx2  = rx * rx
                                    ry2  = ry * ry
                                    rz2  = rz * rz
