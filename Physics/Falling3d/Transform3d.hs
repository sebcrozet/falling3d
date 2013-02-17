{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE TypeSynonymInstances  #-}
{-# LANGUAGE TypeFamilies          #-}

module Physics.Falling3d.Transform3d
(
Transform3d
)
where

import Control.Monad(liftM)
import qualified Data.Vector.Generic as G
import qualified Data.Vector.Generic.Mutable as M
import Data.Vector.Unboxed
import Physics.Falling.Math.Transform hiding(Vector)
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

instance Translatable Transform3d Vec3 where
  translation p = trim t :: Vec3
                  where
                  (Mat4 _ _ _ t) = fromProjective p
  translate     = translate4

instance Rotatable Transform3d Vec3 where
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

newtype instance MVector s Vec3 = MV_Vec3 (MVector s (Double, Double, Double))
newtype instance Vector    Vec3 = V_Vec3  (Vector    (Double, Double, Double))
instance Unbox Vec3

instance M.MVector MVector Vec3 where
  {-# INLINE basicLength #-}
  {-# INLINE basicUnsafeSlice #-}
  {-# INLINE basicOverlaps #-}
  {-# INLINE basicUnsafeNew #-}
  {-# INLINE basicUnsafeReplicate #-}
  {-# INLINE basicUnsafeRead #-}
  {-# INLINE basicUnsafeWrite #-}
  {-# INLINE basicClear #-}
  {-# INLINE basicSet #-}
  {-# INLINE basicUnsafeCopy #-}
  {-# INLINE basicUnsafeGrow #-}
  basicLength (MV_Vec3 v)                     = M.basicLength v
  basicUnsafeSlice i n (MV_Vec3 v)            = MV_Vec3 $ M.basicUnsafeSlice i n v
  basicOverlaps (MV_Vec3 v1) (MV_Vec3 v2)     = M.basicOverlaps v1 v2
  basicUnsafeNew n                            = MV_Vec3 `liftM` M.basicUnsafeNew n
  basicUnsafeReplicate n (Vec3 x y z)         = MV_Vec3 `liftM` M.basicUnsafeReplicate n (x, y, z)
  basicUnsafeRead (MV_Vec3 v) i               = (\(x,y,z) -> Vec3 x y z) `liftM` M.basicUnsafeRead v i
  basicUnsafeWrite (MV_Vec3 v) i (Vec3 x y z) = M.basicUnsafeWrite v i (x, y, z)
  basicClear (MV_Vec3 v)                      = M.basicClear v
  basicSet (MV_Vec3 v) (Vec3 x y z)           = M.basicSet v (x, y, z)
  basicUnsafeCopy (MV_Vec3 v1) (MV_Vec3 v2)   = M.basicUnsafeCopy v1 v2
  basicUnsafeGrow (MV_Vec3 v) n               = MV_Vec3 `liftM` M.basicUnsafeGrow v n

instance G.Vector Vector Vec3 where
  {-# INLINE basicUnsafeFreeze #-}
  {-# INLINE basicUnsafeThaw #-}
  {-# INLINE basicLength #-}
  {-# INLINE basicUnsafeSlice #-}
  {-# INLINE basicUnsafeIndexM #-}
  {-# INLINE elemseq #-}
  basicUnsafeFreeze (MV_Vec3 v)           = V_Vec3 `liftM` G.basicUnsafeFreeze v
  basicUnsafeThaw (V_Vec3 v)              = MV_Vec3 `liftM` G.basicUnsafeThaw v
  basicLength (V_Vec3 v)                  = G.basicLength v
  basicUnsafeSlice i n (V_Vec3 v)         = V_Vec3 $ G.basicUnsafeSlice i n v
  basicUnsafeIndexM (V_Vec3 v) i          = (\(x,y,z) -> Vec3 x y z) `liftM` G.basicUnsafeIndexM v i
  basicUnsafeCopy (MV_Vec3 mv) (V_Vec3 v) = G.basicUnsafeCopy mv v
  elemseq _                               = seq
