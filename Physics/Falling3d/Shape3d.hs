{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE TypeSynonymInstances  #-}

module Physics.Falling3d.Shape3d
(
DynamicShape3d(..)
, StaticShape3d(..)
, TransformedDynamicShape3d(..)
, TransformedStaticShape3d(..)
)
where

import Data.Vect.Double.Base
import Physics.Falling.Shape.VolumetricShape
import Physics.Falling.Shape.TransformableShape
import Physics.Falling.Shape.TransformedShape
import Physics.Falling.Shape.Ball
import Physics.Falling.Shape.Plane
import Physics.Falling3d.Box
import Physics.Falling3d.Transform3d
import Physics.Falling3d.InertiaTensor3d

-- | The shapes of 3D moveable rigid bodies.
-- Most shapes are only wrappers around generic n-dimensional shapes provided by the /falling/
-- library.
--
-- /FIXME: CompoundShape3d and TransformedShape3d are not yet supported./
data DynamicShape3d = Ball3d               (Ball Vec3)
                      | Box3d              Box
                      | CompoundShape3d    [ DynamicShape3d ]
                      | TransformedShape3d (TransformedShape DynamicShape3d Transform3d Vec3)
                      deriving(Show)

-- | The shapes of 3D static rigid bodies.
--
-- /FIXME: CompoundStaticShape3d and TransformedStaticShape3d are not yet supported./
data StaticShape3d  = StaticBall3d               (Ball Vec3)
                      | StaticBox3d              Box
                      | Plane3d                  (Plane Vec3 Normal3)
                      | CompoundStaticShape3d    [ StaticShape3d ]
                      | TransformedStaticShape3d (TransformedShape StaticShape3d Transform3d Vec3)
                      deriving(Show)

-- | The shapes of 3D moveable rigid bodes expressed in world space coordinates.
data TransformedDynamicShape3d = TransformedBall3d        (Ball Vec3)
                                 | TransformedBox3d       (TransformedShape Box Transform3d Vec3)
                                 | TransformedCompound3d  [ TransformedDynamicShape3d ]
                                 deriving(Show)

-- | The shapes of 3D static rigid bodes expressed in world space coordinates.
data TransformedStaticShape3d = TransformedStaticBall3d        (Ball Vec3)
                                | TransformedPlane3d           (Plane Vec3 Normal3)
                                | TransformedStaticBox3d       (TransformedShape Box Transform3d Vec3)
                                | TransformedStaticCompound3d  [ TransformedStaticShape3d ]
                                deriving(Show)

instance VolumetricShape DynamicShape3d InertiaTensor3d InverseInertiaTensor3d Vec3 Transform3d where
  volume                   (Ball3d b)               = ballVolume b
  volume                   (Box3d r)                = boxVolume  r
  volume                   (TransformedShape3d s)   = volume s
  volume                   (CompoundShape3d cs)     = sum $ map volume cs
  objectFrameInertiaTensor (Ball3d (Ball _ r)) m    = InertiaTensor3d $ scaling (Vec3 momentum momentum momentum)
                                                    where
                                                    momentum = 2.0 / 5.0 * m * r * r
  objectFrameInertiaTensor (Box3d b)              m = boxInertiaTensor b m
  objectFrameInertiaTensor (TransformedShape3d s) m = objectFrameInertiaTensor s m -- FIXME: take the transform in account
  objectFrameInertiaTensor (CompoundShape3d _)    _ = undefined                    -- FIXME: «sum» the inertia tensors

instance TransformableShape DynamicShape3d Transform3d TransformedDynamicShape3d where
  transformShape t (Ball3d b)              = TransformedBall3d $ transformShape t b
  transformShape t (TransformedShape3d ts) = transformShape t ts
  transformShape t (Box3d r)               = TransformedBox3d $ TransformedShape r t 
  transformShape t (CompoundShape3d cs)    = TransformedCompound3d $ map (transformShape t) cs

instance TransformableShape StaticShape3d  Transform3d TransformedStaticShape3d  where
  transformShape t (StaticBall3d b)              = TransformedStaticBall3d  $ transformShape t b
  transformShape t (StaticBox3d r)               = TransformedStaticBox3d $ TransformedShape r t
  transformShape t (Plane3d p)                   = TransformedPlane3d $ transformShape t p
  transformShape t (TransformedStaticShape3d ts) = transformShape t ts
  transformShape t (CompoundStaticShape3d cs)    = TransformedStaticCompound3d $ map (transformShape t) cs
