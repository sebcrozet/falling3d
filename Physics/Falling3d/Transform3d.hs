{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE TypeSynonymInstances  #-}

module Physics.Falling3d.Transform3d
(
Transform3d
)
where

import Data.Vect.Double.Base
import Physics.Falling.Math.Transform
import Data.Vect.Double.Util.Projective

type Transform3d = Proj4

instance DeltaTransform Transform3d Vec3 where
  deltaTransform p v = v .* dt
                       where
                       t  = fromProjective p
                       dt = trim t :: Mat3
  deltaTransformTranspose p v = dt *. v
                                where
                                t  = fromProjective p
                                dt = trim t :: Mat3

instance Translation Transform3d Vec3 where
  translation p = trim t :: Vec3
                  where
                  (Mat4 _ _ _ t) = fromProjective p
  translate     = translate4

instance Rotation Transform3d Vec3 where
  rotate orientationVector = if magnitude /= 0.0 then
                               rotateProj4 magnitude normal
                             else
                               id
                             where
                             magnitude = len orientationVector
                             normal    = toNormalUnsafe $ orientationVector &* (1.0 / magnitude)

instance PerpProd Vec3 Vec3 where
  perp = crossprod

instance Transform       Transform3d Vec3
instance TransformSystem Transform3d Vec3 Vec3
