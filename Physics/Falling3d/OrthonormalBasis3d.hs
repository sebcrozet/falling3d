{-# LANGUAGE MultiParamTypeClasses #-}

module Physics.Falling3d.OrthonormalBasis3d
(
)
where

import Data.Vect.Double.Base
import Physics.Falling.Math.OrthonormalBasis

instance OrthonormalBasis Vec3 Normal3 where
  canonicalBasis  = map toNormalUnsafe [ Vec3 1.0 0.0 0.0, Vec3 0.0 1.0 0.0, Vec3 0.0 0.0 1.0 ]
  completeBasis n = let Vec3 x y z = fromNormal n in
                    if abs x > abs y then
                      (n, [
                            mkNormal $ Vec3 z 0.0 (-x)
                            , mkNormal $ Vec3 (x * y) (-x * x - z * z) (z * y)
                          ])
                    else
                      (n, [
                            mkNormal $ Vec3 0.0 (-z) y
                            , mkNormal $ Vec3 (-z * z - y * y) (x * y) (x * z)
                          ])
