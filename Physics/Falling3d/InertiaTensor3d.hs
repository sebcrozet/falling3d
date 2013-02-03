{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE TypeSynonymInstances  #-}

module Physics.Falling3d.InertiaTensor3d
(
InertiaTensor3d(..),
InverseInertiaTensor3d(..),
)
where

import Data.Vect.Double.Base
import Physics.Falling.Dynamics.InertiaTensor
import Physics.Falling.Math.Transform
import Physics.Falling3d.Transform3d

newtype InertiaTensor3d        = InertiaTensor3d        Transform3d
                                 deriving(Show)
newtype InverseInertiaTensor3d = InverseInertiaTensor3d Transform3d
                                 deriving(Show)

instance InertiaTensor InertiaTensor3d InverseInertiaTensor3d Vec3 Transform3d where
  inverseInertia (InertiaTensor3d inertia) = InverseInertiaTensor3d $ inverse inertia

instance InverseInertiaTensor InverseInertiaTensor3d Vec3 Transform3d where
  applyToVector (InverseInertiaTensor3d i) v         = i `deltaTransform` v
  toWorldSpaceTensor (InverseInertiaTensor3d i) t it = InverseInertiaTensor3d $ t .*. i .*. it
