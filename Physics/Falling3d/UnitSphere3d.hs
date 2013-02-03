module Physics.Falling3d.UnitSphere3d
(
)
where

import System.Random
import Data.Random.Normal
import Data.Vect.Double.Base
import Physics.Falling.Math.UnitSphere

instance UnitSphere Normal3 where
  unitSphereSamples    = _randomSpherePoints 
  -- FIXME: implement uniform sampling for nUnitSphereSamples

_randomSpherePoints :: [Normal3]
_randomSpherePoints  = map mkNormal $ _doubleList2Vec3List $ normals $ mkStdGen 0

_doubleList2Vec3List :: [Double] -> [Vec3]
_doubleList2Vec3List (a:b:c:l) = Vec3 a b c : _doubleList2Vec3List l
_doubleList2Vec3List _       = []
